// fs1r/firmware/notes.cpp - note on and off, operator setup, and the 192 Hz tick. KNOWN: rewritten from the disassembly.
#include "fs1r/internal.h"

void Synth::init_system() {
    memset(sys, 0, sizeof sys);
    sys[0] = 64; sys[6] = 64; sys[8] = 0; sys[9] = 0; sys[0x0E] = 0;
    sys[0x10] = sys[0x12] = sys[0x13] = sys[0x14] = sys[0x15] = 1;
    static const uint8_t cc[12] = {16, 17, 18, 19, 20, 21, 22, 13, 4, 2, 80, 81};   // KN1-4 MC1-4 FC BC Formant FM
    memcpy(sys + 0x16, cc, 12);
    sys[0x47] = 0; sys[0x48] = 4; sys[0x49] = 0; sys[0x4A] = 0;
}

void Synth::note_on(int part, int note, int vel) {
    Part& pt = perf.part[part]; const Voice& V = pt.voice;
    int lo = pt.p[0x0F], hi = pt.p[0x10];
    if (hi < lo ? (note > hi && note < lo) : (note > hi || note < lo)) return;
    lo = pt.p[0x2A]; hi = pt.p[0x2B];
    if (hi < lo ? (vel > hi && vel < lo) : (vel > hi || vel < lo)) return;
    vel = clampi(((pt.p[0x0C] * VELCURVES[clampi(velCurve(), 0, 4) * 128 + vel]) >> 6) + (pt.p[0x0D] - 0x40) * 2, 1, 127);
    bool mono = pt.p[5] == 0;
    // mono: remember held notes, choose the sounding note by priority
    if (mono) {
        if (pt.nheld < 32) pt.held[pt.nheld++] = note;
        int prio = pt.p[6] & 3, target = note;
        if (prio == 1) { target = -1; for (int i = 0; i < pt.nheld; i++) target = std::max(target, pt.held[i]); }
        else if (prio == 2) { target = 128; for (int i = 0; i < pt.nheld; i++) target = std::min(target, pt.held[i]); }
        else if (prio == 3) target = pt.held[0];
        Chan* cur = nullptr; for (auto& c : ch) if (c.active && c.part == part && c.held) cur = &c;
        if (cur && pt.nheld > 1) {          // legato: retune the sounding channel (FUN_000119d0), no retrigger
            if (target != cur->note) retune(*cur, target);
            return;
        }
        note = target;
    }
    // FUN_0000f7dc, measured against the unit (FS1R.unlock 2026-09-25-7). Three things, none of which
    // the old "first free, else oldest" did:
    //  1. allocation is ROUND ROBIN from the last channel allocated, not the lowest free one. Filling an
    //     empty machine with 32 notes puts them on channels 1..31 then 0, which is what the unit did.
    //  2. when nothing is free, the part to take from is the one furthest over its own note reserve.
    //     The reserves cannot oversubscribe the 32 channels: writing 32 to part 2 with parts 3 and 4
    //     holding 8 each read back as 16, i.e. 32 - (0 + 8 + 8), so a part's effective reserve is what
    //     is left after the others. (FUN_00030796 clamps parameter writes against a per-parameter
    //     table; the sum rule is the reading that fits the one measured point, 2026-09-25-7.)
    //  3. within that part the walk starts after the part's own cursor and takes the first channel whose
    //     stored note is ABOVE the arriving one; failing that, the lowest stored note it saw.
    Chan* c = nullptr;
    for (int k = 1; k <= NCHAN; k++) { Chan& x = ch[(nextChan + k) % NCHAN]; if (!x.active) { c = &x; break; } }
    if (!c) {
        int best = part, excess = -1;
        for (int k = 0; k < 4; k++) {
            int p = (part + k) & 3, others = 0, n = 0;
            for (int q = 0; q < 4; q++) if (q != p) others += perf.part[q].p[0x00];
            int res = clampi(perf.part[p].p[0x00], 0, std::max(0, NCHAN - others));
            for (auto& x : ch) if (x.active && x.part == p) n++;
            if (n > res && n - res > excess) { excess = n - res; best = p; }
        }
        int thr = note, fb = -1;
        for (int k = 1; k <= NCHAN; k++) {
            int i = (partChan[best] + k) % NCHAN;
            if (!ch[i].active || ch[i].part != best) continue;
            if (thr < ch[i].note) { c = &ch[i]; break; }
            if (fb >= 0) { c = &ch[fb]; break; }
            fb = i; thr = ch[i].note;
        }
        if (!c) c = &ch[fb >= 0 ? fb : 0];
    }
    nextChan = int(c - ch); partChan[part] = nextChan;
    Chan& C = *c; bool sync = V.lfo1sync != 0; uint32_t keepPhase = C.lfoPhase;
    // FUN_00023000: the firmware damps the channel it is about to take, it does not silence it. The mask
    // it is called with comes out of the one-hot table at 0x35B260 indexed by the allocated channel, and
    // the call sets that channel's EG stage word to 4 (release) and writes register 0xFC/FD. So a note
    // landing on a channel that is still sounding fades the old waveform out over cal::DAMP_MS. Zeroing
    // the struct instead put a step the size of the old note's own output into the output between two
    // samples, which is a click the unit does not make.
    double dL = C.active ? C.lastL + C.dampL : 0.0, dR = C.active ? C.lastR + C.dampR : 0.0;
    C = Chan(); C.active = true; C.part = part; C.note = note; C.vel = vel; C.held = true; C.age = ++clock;
    C.dampL = dL; C.dampR = dR;
    C.lfoPhase = sync ? 0 : keepPhase;
    C.lfo2Phase = V.lfo2sync ? (uint32_t)(V.lfo2phase * 0x4000) : (uint32_t)(rand() & 0xFFFF);
    C.panBase = pt.p[0x0E] ? pt.p[0x0E] : (rand() % 128);       // part pan 0 = random per note; the rest is the table index
    compute_pitch(C, pt, note);
    // portamento start (FUN_000124fe)
    int porta = pt.p[0x24]; C.portaTarget = C.pitchNote; C.portaCur = C.pitchNote;
    if ((porta & 1) && pt.p[0x25] && pt.lastPitch >= 0 && ((porta & 2) || (mono && pt.nheld > 1))) C.portaCur = pt.lastPitch;
    C.portaOn = (porta & 1) && pt.p[0x25]; C.portaRate = PEGTIME[clampi((pt.p[0x25] * 100) >> 7, 0, 99)];
    pt.lastPitch = C.pitchNote;
    setup_ops(C, pt, vel);
    setup_peg(C, pt, vel);
    // Fseq trigger (FUN_0000fffa): first note only, or every note
    if (fseq.valid && fseqPart == part && ((perf.c[0x24] & 1) || !anyOther(C))) fseq_start(vel);
    // The filter EG is staged before the coefficients are written, as FUN_0000B0E8 does (FUN_0000D050
    // stages the EG, then writes cutoff, resonance and input gain in one pass). It used to run at the end
    // of note_on, after refresh_regs, so the note's first tick reached the chip with the EG of whatever
    // played on this channel last.
    start_filter(C, pt, vel);
    refresh_regs(C, pt);
    if (getenv("FS1R_DEBUG")) {
        printf("note %d vel %d -> note' %d pitch %d keyfact %d C0 %d\n", note, vel, C.noteP, C.pitchNote, C.keyfact, C.regC0);
        for (int o = 0; o < 8; o++) printf("  op%d lvl %3d ulvl %3d egb %3d freq %5d ufreq %5d frmt %5d bw %2d egL %2d %2d %2d %2d egR %2d %2d %2d %2d hold %d\n", o + 1, C.levelOff[o], C.ulevelOff[o], C.egbias[o], C.freqWord[o], C.ufreqWord[o], C.frmtWord[o], C.bwReg[o], C.egL[o][0], C.egL[o][1], C.egL[o][2], C.egL[o][3], C.egR[o][0], C.egR[o][1], C.egR[o][2], C.egR[o][3], C.egHold[o]);
        printf("  peg lvl %d %d %d %d %d rate %d %d %d %d %d velP %d\n", C.pegLvl[0], C.pegLvl[1], C.pegLvl[2], C.pegLvl[3], C.pegLvl[4], C.pegRt[0], C.pegRt[1], C.pegRt[2], C.pegRt[3], C.pegRt[4], C.velP);
        int sp = eb86(clampi(V.lfo1speed + pt.p[0x15] - 64, 0, 99)); uint32_t inc = (sp == 0 ? 0xB : sp * (sp < 0xA0 ? 0xB : 0xB + ((sp - 0xA0) >> 2))) << 1;
        int q = 99 - clampi(V.lfo1delay + pt.p[0x17] - 64, 0, 99); uint32_t dinc = ((16 + (q & 15)) << 10) >> (7 - (q >> 4));
        printf("  lfo wave %d speed %d -> inc %u (%.2f Hz) delay %d -> inc %u (%.2f s) pmd %d amd %d fmd %d porta %d rate %d\n", V.lfo1wave, V.lfo1speed, inc, inc * TICK_HZ / 65536.0, V.lfo1delay, dinc, 65536.0 / (dinc ? dinc : 1) / TICK_HZ, V.pmd, V.amd, V.fmd, C.portaOn, C.portaRate);
    }
    for (int o = 0; o < 8; o++) {
        OpState& s = C.op[o]; const OpV& v = V.v[o]; const OpU& u = V.u[o];
        s.eg.start(C.egL[o], C.egR[o], C.egHold[o], eg_ratescale(v.tscale, C.regC0));
        s.feg.start(v.fegInit, v.fegAtt, v.fegAttT, v.fegDecT);
        s.phase = v.keysync ? 0.0 : (double)rand() / RAND_MAX;
        s.ueg.start(C.uegL[o], C.uegR[o], C.uegHold[o], eg_ratescale(u.tscale, C.regC0));
        s.ufeg.start(u.fegInit, u.fegAtt, u.fegAttT, u.fegDecT);
        s.rng = 0x9E3779B9u * (o + 1) ^ clock; if (!s.rng) s.rng = 1;
        s.attS = -1e9;                    // the level glide starts at the new note's own level
    }
}

void Synth::start_filter(Chan& C, const Part& pt, int vel) {
    const Voice& V = pt.voice;
    C.flt.clear();
    // FUN_0000D050 / FUN_0000CB6C / FUN_0000D7B0: the level words are (L - 50) * 256 / 50 and each
    // time is clamped to 0..99 after the part offset, then shortened by the time scaling term
    // (note - 60) * time * tscale / 0x57F and, on the attack alone, by the velocity term
    // (vel - 127) * time * atkvel / 0x6F2. Both divide toward zero, as the firmware's sdiv does.
    double lv[4]; int rt[4];
    for (int i = 0; i < 4; i++) {
        lv[i] = (V.fltL[i] - 50) * 256 / 50;
        int t = clampi(V.fltT[i], 0, 99), t2 = t - ((C.noteP - 60) * t * V.fltTscale) / 0x57F;
        if (i == 0) t2 -= ((vel - 127) * t * V.fltAtkVel) / 0x6F2;
        rt[i] = clampi(t2, 0, 99);
    }
    C.feg.start(lv, rt);
}

void Synth::compute_pitch(Chan& C, const Part& pt, int note) {
    int n = clampi(note + (sysNoteShift() & 0x7F) - 0x40, 0, 127);
    n = clampi(n + (perf.c[0x12] & 0x3F) - 24, 0, 127);
    n = clampi(n + (pt.p[8] & 0x3F) - 24, 0, 127);
    n = clampi(n + (pt.voice.raw[0x1E] & 0x3F) - 24, 0, 127);
    C.noteP = n; C.keyfact = KEYFACT[n] >> 2;
    int pitch = NOTETAB[n];
    int d = pt.p[9] - 0x40;
    if (d) { int band = (n >> 3) & 0xF; int prod = std::abs(d) * (15 - band); if (!prod) prod = 1; pitch += (d < 0 ? -prod : prod) / 15; }
    pitch += (sysTune() & 0x7F) - 0x40;
    C.pitchNote = pitch;
}

// A voice parameter change reaches the notes already sounding, as the unit's own edit buffer does:
// the per-channel words are rebuilt from the voice at the note's own pitch and velocity. The EGs in
// flight keep the segments they started with, and the filter EG likewise; everything the tick
// reads through refresh_regs (frequency, level, form, bandwidth, detune, algorithm, LFOs, filter)
// moves at once.
void Synth::voice_changed(int part) {
    const Part& pt = perf.part[part];
    for (auto& C : ch) if (C.active && C.part == part) { setup_ops(C, pt, C.vel); refresh_regs(C, pt); }
}

void Synth::retune(Chan& C, int note) {
    const Part& pt = perf.part[C.part];
    C.note = note; compute_pitch(C, pt, note);
    C.portaTarget = C.pitchNote; if (!C.portaOn) C.portaCur = C.pitchNote;
    perf.part[C.part].lastPitch = C.pitchNote;
    setup_ops(C, pt, C.vel); refresh_regs(C, pt);
}

void Synth::setup_ops(Chan& C, const Part& pt, int vel) {
    const Voice& V = pt.voice; int n = C.noteP; int pm = C.pitchNote - 0x53AA; int part = C.part;
    int band = (C.pitchNote >> 8) + 10; band = band < 0x50 ? 0 : band < 0x70 ? ((band ^ 0x10) & 0x1F) : 0x1F;
    int fv = (fseq.valid && fseqPart == part) ? V.fseqV : 0, fu = (fseq.valid && fseqPart == part) ? V.fseqU : 0;
    for (int o = 0; o < 8; o++) {
        const OpV& v = V.v[o]; const OpU& u = V.u[o];
        C.fseqOp[o] = (fv >> o) & 1; C.fseqUOp[o] = (fu >> o) & 1;
        // voiced level offset: velocity + level + key scaling (0.375 dB units)
        int ks;
        {
            int base = LEVTAB[std::min(99, v.level)];
            int dist = keygroup(n) - keygroup(v.bp + 21) + 1;
            int att;
            if (dist <= 0) { const unsigned char* t = (v.lc == 1 || v.lc == 2) ? KSEXP : KSLIN; int x = std::min(127, (t[std::min(39, -dist)] * eb86(v.ld)) >> 8); att = (v.lc == 2 || v.lc == 3) ? base - x : base + x; }
            else { const unsigned char* t = (v.rc == 1 || v.rc == 2) ? KSEXP : KSLIN; int x = std::min(127, (t[std::min(39, dist)] * eb86(v.rd)) >> 8); att = (v.rc == 2 || v.rc == 3) ? base - x : base + x; }
            ks = clampi(att, 0, 127) << 1;
        }
        C.levelOff[o] = std::min(255, vel_att(v.amsb, vel) + ks);
        // unvoiced: 2*LEVTAB + velocity, then level key scaling
        {
            int a = std::min(255, LEVTAB[std::min(99, u.level)] * 2 + vel_att(u.amsb, vel));
            a += -(u.lks * 64 * (n - 60)) >> 8;
            C.ulevelOff[o] = clampi(a, 0, 255);
        }
        // frequency words. The bias the controller matrix adds to them is not here: it moves while
        // the note sounds, and refresh_bias rebuilds it every tick.
        if (v.form == 7 || v.fixed) C.freqWord[o] = 8 * (v.coarse * 128 + v.fine) + 0x28ED + keytrack(v.notescale, pm) + fvs_term(v.fmsb, vel);
        else C.freqWord[o] = 0x1243 + COARSE[v.coarse] + FINE[std::min(99, v.fine)];
        C.ufreqWord[o] = std::min(0x7F00, ((u.coarse & 0x1F) * 256 + u.fine * 2) * 4 + 0x28ED + keytrack(u.notescale, pm) + fvs_term(u.fmsb, vel));
        // Detune, in pitch word units, from the ROM's own key scaled table. MEASURED: the chip does
        // the same thing to a ratio operator's register 0x80 that the CPU does to a formant
        // operator's transpose word, so both read FRMDET and the amount shrinks as the note rises.
        {
            int d = v.detune; int dd = d < 15 ? (~d) : d - 15; int idx = (dd & 0x1F) * 32 + band;
            C.detW[o] = (idx & 0x200) ? -FRMDET[idx & 0x1FF] : FRMDET[idx & 0x1FF];
        }
        // formant transpose word (register 0x230): 0x1243 + TRANS + that detune for frmt ops, else the raw byte 6
        if (v.form == 7) C.frmtWord[o] = 0x1243 + TRANS[clampi(v.transpose + 24, 0, 48)] + C.detW[o];
        else C.frmtWord[o] = v.bw;
        // EG registers: levels LEVTAB >> 1, rates ((99-T)*0xA4)>>8, part EG offsets on attack/decay/release (assumed T1/T2/T4)
        // FUN_00019414 walks the part's EG bytes 0x1A, 0x1B, 0x1B, 0x1C against voice times T1-T4,
        // so the decay offset reaches both T2 and T3, not T2 alone.
        int atk = ctrl_part(part, 24, pt.p[0x1A]), dec = ctrl_part(part, 25, pt.p[0x1B]), rel = ctrl_part(part, 26, pt.p[0x1C]);
        int offs[4] = {atk - 64, dec - 64, dec - 64, rel - 64};
        for (int i = 0; i < 4; i++) {
            C.egL[o][i] = LEVTAB[std::min(99, v.L[i])] >> 1; C.egR[o][i] = egrate(v.T[i] + offs[i]);
            C.uegL[o][i] = LEVTAB[std::min(99, u.L[i])] >> 1; C.uegR[o][i] = egrate(u.T[i] + offs[i]);
        }
        C.egHold[o] = v.hold; C.uegHold[o] = u.hold;
    }
}

void Synth::setup_peg(Chan& C, const Part& pt, int vel) {
    const Voice& V = pt.voice;
    C.velP = V.pegVel ? vel + 1 : 128;        // firmware overrides the graded PEGVEL table with this (see docs)
    int Lp[5], Tp[5];
    for (int i = 0; i < 5; i++) Lp[i] = V.pegL[i];
    Lp[0] = clampi(Lp[0] + ctrl_part(C.part, 27, pt.p[0x20]) - 64, 0, 100);
    Lp[4] = clampi(Lp[4] + ctrl_part(C.part, 29, pt.p[0x22]) - 64, 0, 100);
    Tp[0] = 255; for (int i = 0; i < 4; i++) Tp[i + 1] = V.pegT[i];
    Tp[1] = clampi(Tp[1] + ctrl_part(C.part, 28, pt.p[0x21]) - 64, 0, 99);
    Tp[4] = clampi(Tp[4] + ctrl_part(C.part, 30, pt.p[0x23]) - 64, 0, 99);
    for (int i = 0; i < 5; i++) {
        int w = ((int)PEGLVL[clampi(Lp[i], 0, 100)] - 128) << 7;
        w = (w * C.velP) >> 7; if (V.pegRange) w >>= V.pegRange + 1;
        C.pegLvl[i] = w;
        int r = i ? PEGTIME[clampi(Tp[i], 0, 99)] : 255;
        C.pegRt[i] = r == 255 ? 255 : std::min(254, ((r * (C.velP + 1)) >> 7) + std::min(255, V.pegTscale * C.keyfact));
    }
    if (C.pegRt[1] == 255) { C.pegStage = 1; C.pegCur = C.pegLvl[1]; C.pegTarget = C.pegLvl[2]; C.pegRate = C.pegRt[2]; }
    else { C.pegStage = 0; C.pegCur = C.pegLvl[0]; C.pegTarget = C.pegLvl[1]; C.pegRate = C.pegRt[1]; }
}

void Synth::note_off(int part, int note) {
    Part& pt = perf.part[part];
    if (pt.p[5] == 0) {                       // mono
        for (int i = 0; i < pt.nheld; i++) if (pt.held[i] == note) { memmove(pt.held + i, pt.held + i + 1, (pt.nheld - i - 1) * sizeof(int)); pt.nheld--; break; }
        Chan* cur = nullptr; for (auto& c : ch) if (c.active && c.part == part && c.held) cur = &c;
        if (!cur) return;
        if (pt.nheld > 0) {                   // another key still down: go back to it (legato)
            int prio = pt.p[6] & 3, target = pt.held[pt.nheld - 1];
            if (prio == 1) { target = -1; for (int i = 0; i < pt.nheld; i++) target = std::max(target, pt.held[i]); }
            else if (prio == 2) { target = 128; for (int i = 0; i < pt.nheld; i++) target = std::min(target, pt.held[i]); }
            else if (prio == 3) target = pt.held[0];
            if (target != cur->note) retune(*cur, target);
            return;
        }
        cur->held = false; if (pt.sustain && (pt.p[0x2D] & 1)) { cur->sustained = true; return; }
        release(*cur); return;
    }
    for (auto& c : ch) if (c.active && c.part == part && c.note == note && c.held) {
        c.held = false;
        if (pt.sustain && (pt.p[0x2D] & 1)) { c.sustained = true; continue; }
        release(c);
    }
}

void Synth::release(Chan& C) {
    for (auto& s : C.op) { s.eg.release(); s.ueg.release(); }
    C.feg.release();
    C.pegStage = 4; C.pegTarget = C.pegLvl[4]; C.pegRate = C.pegRt[4];
    fseq_keys();
}

void Synth::lfo_tick(Chan& C, const Part& pt) {
    const Voice& V = pt.voice; int part = C.part;
    int dly = clampi(V.lfo1delay + pt.p[0x17] - 64, 0, 99), q = 99 - dly;
    uint32_t dinc = ((16 + (q & 15)) << 10) >> (7 - (q >> 4));
    C.lfoDelay = std::min<uint32_t>(0xFFFF, C.lfoDelay + dinc);
    if (C.lfoDelay == 0xFFFF) C.lfoFade = std::min<uint32_t>(0xFFFF, C.lfoFade + dinc);
    int sp = eb86(clampi(V.lfo1speed + pt.p[0x15] - 64, 0, 99)) + clampi(ctrl_offset(part, 43) * 2, -255, 255);
    sp = clampi(sp, 0, 255);
    uint32_t inc = sp == 0 ? 0xB : sp * (sp < 0xA0 ? 0xB : 0xB + ((sp - 0xA0) >> 2)); inc = (inc & 0xFFFF) << 1;
    uint32_t ph = C.lfoPhase + inc; bool wrapped = ph > 0xFFFF; C.lfoPhase = ph & 0xFFFF;
    int hi = C.lfoPhase >> 8, val;
    switch (V.lfo1wave) {
    case 1: val = (int8_t)(~hi); break;
    case 2: val = (int8_t)hi; break;
    case 3: val = (hi & 0x80) ? -128 : 127; break;
    case 4: { int idx = hi & 0x3F; if (hi & 0x40) idx ^= 0x3F; int s = SINE64[idx]; val = (hi & 0x80) ? (int8_t)(~s) : s; break; }
    case 5: if (wrapped) { C.lfoSH = SHTAB[((C.lfoSH & 0xFF) * 0xB3 + (rand() & 0xFF)) & 0x7FF]; } val = C.lfoSH; break;
    default: { uint32_t t = (C.lfoPhase << 1); if (t > 0xFFFF) t = ~t & 0xFFFF; val = (int)((t >> 8) & 0xFF) - 0x80; }
    }
    C.lfoVal = val;
}

void Synth::lfo2_tick(Chan& C, const Part& pt) {
    // LFO2 is not the CPU's: FUN_0000D050 / FUN_0000E2A0 hand VOP3-1 the waveform (FUN_0000C1AC, the
    // 0x20-per-step field the docs had as the filter type) and the speed word LFO2SPD[speed] (FUN_0000C130)
    // and the chip runs it. The speed byte is voice + part - 64 + the destination 45 word, clamped 0..127.
    // What the chip makes of the increment is INFERRED: read as a 16 bit phase per tick it puts speed 64
    // at 0.19 Hz and 127 at 1.6 Hz, and the register run in FS1R.unlock fs1r_capture_session3 `lfo2` is what settles it.
    const Voice& V = pt.voice;
    int sp = clampi(V.lfo2speed + pt.p[0x2E] - 64 + ctrl_offset(C.part, 45), 0, 127);
    uint32_t inc = LFO2SPD[sp] * cal::LFO2_INC_K;
    uint32_t ph = C.lfo2Phase + inc; bool wrapped = ph > 0xFFFF; C.lfo2Phase = ph & 0xFFFF;
    int hi = C.lfo2Phase >> 8;
    switch (V.lfo2wave) {
    case 1: C.lfo2Val = (int8_t)(~hi); break;
    case 2: C.lfo2Val = (int8_t)hi; break;
    case 3: C.lfo2Val = (hi & 0x80) ? -128 : 127; break;
    case 4: { int idx = hi & 0x3F; if (hi & 0x40) idx ^= 0x3F; int v = SINE64[idx]; C.lfo2Val = (hi & 0x80) ? (int8_t)(~v) : v; break; }
    case 5: if (wrapped) C.lfo2SH = SHTAB[((C.lfo2SH & 0xFF) * 0xB3 + (rand() & 0xFF)) & 0x7FF]; C.lfo2Val = C.lfo2SH; break;
    default: { uint32_t t = (C.lfo2Phase << 1); if (t > 0xFFFF) t = ~t & 0xFFFF; C.lfo2Val = (int)((t >> 8) & 0xFF) - 0x80; }
    }
}

void Synth::peg_tick(Chan& C) {
    int s = C.pegStage; if (s == 3 || s == 5) return;
    if (s > 5) { C.pegStage = 3; return; }
    if (C.pegRate < 255) {
        if (C.pegCur < C.pegTarget) C.pegCur = std::min(C.pegTarget, C.pegCur + C.pegRate);
        else if (C.pegCur > C.pegTarget) C.pegCur = std::max(C.pegTarget, C.pegCur - C.pegRate);
        if (C.pegCur != C.pegTarget) return;
    } else C.pegCur = C.pegTarget;
    C.pegStage = s + 1;                       // next segment: rate T[s+2], target L[s+2] (stage 3 holds L3 until key off)
    if (s + 1 < 5) { C.pegRate = C.pegRt[std::min(4, s + 2)]; C.pegTarget = C.pegLvl[std::min(4, s + 2)]; }
}

void Synth::porta_tick(Chan& C) {
    if (!C.portaOn || !C.portaRate) { C.portaCur = C.portaTarget; return; }
    int d = C.portaTarget - C.portaCur; if (!d) return;
    if (d < 0) { int n = C.portaCur - (((-d) >> 10) + 1) * C.portaRate; C.portaCur = std::max(C.portaTarget, n); }
    else { int n = C.portaCur + ((d >> 10) + 1) * C.portaRate; C.portaCur = std::min(C.portaTarget, n); }
}

void Synth::refresh_regs(Chan& C, const Part& pt) {
    const Voice& V = pt.voice; int part = C.part;
    voice_ctrl(C, pt); refresh_bias(C, pt);
    int fseqPitch = 0;
    if (fseq_on(part) && !(perf.c[0x23] & 1)) {
        const uint8_t* f = fseq.frame[fseqStep];
        fseqPitch = (f[0] * 256 + f[1] * 2) - (NOTETAB[fseq.noteAssign & 0x7F] + 0x1243) + (fseq.tuning - 63);
    }
    C.regPitch = C.portaCur + (C.pegCur >> 2) + bend_word(pt) + fseqPitch;
    C.regC0 = (C.pitchNote >> 8) + 10;
    int fade = C.lfoFade >> 8;
    int pmd = clampi(eb86(clampi(V.pmd + pt.p[0x16] - 64, 0, 99)) + ctrl_offset(part, 39) * 2, 0, 255);   // ctrlDest_39 doubles it
    C.regPM = clampi((C.lfoVal * ((pmd * fade) >> 8 & 0xFF)) >> 4, -0x7FF, 0x7FF);
    int fmd = clampi(eb86(V.fmd) + ctrl_offset(part, 41), 0, 255);
    C.regFM = clampi((C.lfoVal * ((fmd * fade) >> 8 & 0xFF)) >> 4, -0x7FF, 0x7FF);
    int amc = ctrl_offset(part, 40);
    int amd = clampi(eb86(V.amd) + amc, 0, 255), fadeA = clampi(fade + amc, 0, 255);
    C.regAM = EGBIAS[(((amd * fadeA) >> 8) * ((127 - C.lfoVal) & 0xFF)) >> 8];
    int lvVel = 0x80;                                     // Fseq level velocity multiplier (FUN_000125f8)
    if (fseqPart == part && perf.c[0x27] != 0x40) {
        int s = perf.c[0x27]; int mul;
        if (s & 0x40) mul = ((s & 0x3F) + 1) * VELW[VELCURVE[fseqVel] & 0x7F]; else mul = ((~s) & 0x3F) * VELW[127 - (VELCURVE[fseqVel] & 0x7F)];
        lvVel = (~(mul >> 6)) & 0xFF;
    }
    // No formant tracking term: FUN_000127FC applies the whole key-to-Fseq difference to the channel
    // pitch (the fseqPitch above, whose constants it confirms exactly) and nothing shifts the frame's
    // formant frequencies. Formants staying put while the fundamental moves is the point of the format.
    for (int o = 0; o < 8; o++) {
        int l = C.levelOff[o], ul = C.ulevelOff[o];
        if (fseq_on(part)) {
            const uint8_t* f = fseq.frame[fseqStep]; int t = V.v[o].fseqtrk;
            if (C.fseqOp[o]) { l = f[0x12 + t] << 1; if (lvVel != 0x80) l = (~(((~l) & 0xFF) * lvVel >> 8)) & 0xFF; C.fqWord[o] = f[2 + t] * 256 + f[0xA + t] * 2 + C.fbW[o]; }
            if (C.fseqUOp[o]) { ul = f[0x2A + t] << 1; if (lvVel != 0x80) ul = (~(((~ul) & 0xFF) * lvVel >> 8)) & 0xFF; C.fquWord[o] = f[0x1A + t] * 256 + f[0x22 + t] * 2 + C.ufbW[o]; }
        }
        C.regLevel[o] = clampi(l + C.egbias[o] + C.vcLvl[o][0], 0, 255);
        C.regULevel[o] = clampi(ul + C.uegbias[o] + C.vcLvl[o][1], 0, 255);
    }
    int vAtt, uAtt; part_levels(part, vAtt, uAtt);
    C.partV = db2lin(-LEVEL_DB * vAtt); C.partU = db2lin(-LEVEL_DB * uAtt);
    refresh_pan(C, pt); refresh_filter(C, pt);
}

void Synth::voice_ctrl(Chan& C, const Part& pt) {
    // FUN_00017454 walks the ten control slots with amt = clamp(src' * dep' * 2 >> 7) (FUN_00016FF0, the
    // same bias-and-scale the controller sets use), src being the part's FORMANT or FM byte as
    // (v - 64) * 2 clamped to a signed byte. The handlers at flash 0x3DE04: "out" stores -2 * amt as a
    // level offset (FUN_00017A4A), "freq" stores amt << 5 as a frequency word offset (FUN_00017B02), and
    // "width" stores amt into the per-op bandwidth offset that FUN_0001F6BC then scales by the op's own
    // BWBIAS, exactly as destination 37 does (FUN_00017B6E). The engine had roughly a quarter, an eighth
    // and a quarter of those, and a cal constant for a scale that is a shift in the flash.
    memset(C.vcLvl, 0, sizeof C.vcLvl); memset(C.vcFreq, 0, sizeof C.vcFreq); memset(C.vcBw, 0, sizeof C.vcBw);
    const uint8_t* b = pt.voice.raw;
    int src[2] = {clampi((ctrl_part(C.part, 32, pt.p[0x1D]) - 64) * 2, -128, 127), clampi((ctrl_part(C.part, 33, pt.p[0x1E]) - 64) * 2, -128, 127)};
    for (int k = 0; k < 10; k++) {
        int d = b[k < 5 ? 0x40 + k : 0x4A + (k - 5)];
        int dep = (int)b[k < 5 ? 0x45 + k : 0x4F + (k - 5)] - 64;
        int dd = (d >> 4) & 3, v = (d >> 3) & 1, o = d & 7;
        if (dd < 1 || dd > 3) continue;
        int amt = clampi((ctrl_bias(src[k < 5 ? 0 : 1]) * ctrl_bias(dep) * 2) >> 7, -128, 127);
        if (dd == 1) C.vcLvl[o][v] += clampi(-2 * amt, -255, 255);
        else if (dd == 2) C.vcFreq[o][v] += amt << 5;
        else { const uint8_t* op = pt.voice.raw + 112 + o * 62; int bias = v ? (op[35 + 5] & 0xF) : ((op[4] >> 3) & 0xF); C.vcBw[o][v] += clampi((amt * BWBIAS[bias]) >> 3, -128, 127); }
    }
}

void Synth::refresh_pan(Chan& C, const Part& pt) {
    int base = pt.p[0x0E] ? ctrl_part(C.part, 18, pt.p[0x0E]) : C.panBase;             // Panpot edits the part byte
    int idx = pan_index(base, pt.p[0x28], C.noteP);
    idx += ((C.lfoVal * std::max(1, eb86(clampi(pt.p[0x29], 0, 99))) * (int)(C.lfoFade >> 8)) >> 16) / 2; // pan LFO depth: image +0x1F is eb86(byte), 1 at 0 (FUN_00019362 case 0x29)
    if (perf.c[0x11]) idx += (perf.c[0x11] - 64) / 2;                              // performance pan
    idx = clampi(idx, 0, 127);
    C.panL = db2lin(-LEVEL_DB * PANL[idx]); C.panR = db2lin(-LEVEL_DB * PANR[idx]);
}

void Synth::refresh_filter(Chan& C, const Part& pt) {
    const Voice& V = pt.voice; int part = C.part;
    // The switch, the type and the input gain are read on the tick rather than latched at note-on, so a
    // held note follows them as it already follows cutoff and resonance. Ours, and not the unit's:
    // FUN_0000B0E8 hands the switch to the filter channel at note-on only (FUN_00010B48 reads part 0x07
    // into DAT_01029160 and gates the whole call on it), so on hardware a sounding note keeps whatever it
    // started with. docs/Differences.md.
    bool on = (pt.p[7] & 1) != 0;
    if (on && !C.fltOn) C.flt.clear();          // switched in mid-note: start from rest, not from stale state
    C.fltOn = on; C.fltType = V.fltType;
    C.fltInGain = db2lin(V.fltInGain); C.fltGain = C.fltInGain * cal::FLT_LOSS;
    if (!on) return;
    // FUN_0000DB3C, FUN_0000D050 and FUN_0000E170, with the firmware's own integer arithmetic. Every
    // division is the flash's sdiv, which truncates toward zero.
    auto sdiv = [](long long n, long long d) { return (int)(n / d); };
    int vel = C.vel;
    // Destinations 42 and 44 add to the LFO depths, not to the cutoff: the controller word is scaled by
    // 99/127 and added to the voice's own depth before the clamp.
    int lfo1d = clampi(V.fltLfo1 + sdiv(ctrl_offset(part, 42) * 99, 127), 0, 99);
    int lfo2d = clampi(V.fltLfo2 + pt.p[0x2F] - 64 + sdiv(ctrl_offset(part, 44) * 99, 127), 0, 99);
    // Cutoff: voice byte plus the part offset, plus LFO1 * depth * 0x3000 / 0x2076F9, clamped 0..127.
    // The LFO2 term is not here: LFO2 runs on VOP3-1 itself (FUN_0000C130 and FUN_0000CA30 hand it
    // the speed and depth) and the engine adds it below as the chip would, INFERRED in shape.
    // The LFO word the CPU keeps per channel (FUN_0002C36C, +0x14) is lfo * fade >> 8 as a signed byte,
    // and the depth byte goes in raw, not through eb86.
    int lfoW = (int8_t)((C.lfoVal * clampi(C.lfoFade >> 8, 0, 255)) >> 8);
    int cut = V.fltCut + (ctrl_part(part, 21, pt.p[0x18]) - 64) + sdiv((long long)lfoW * lfo1d * 0x3000, 0x2076F9);
    cut = clampi(cut, 0, 127);
    // FUN_0000C36C: the key scaling reaches the coefficient as ((ks - 64) << 11) / 0x180 per semitone
    // from the breakpoint, in the same units as 0xA9 per cutoff byte, so in cutoff bytes it is
    // (ks - 64) * (note - bp) * 16 / (3 * 0xA9), about a byte per 32 semitone-steps at full depth.
    double ks = (double)((V.fltKsDepth << 11) / 0x180) * (C.noteP - clampi(V.fltKsPoint, 0, 127)) / 0xA9;
    // Filter EG depth: FUN_0000D050 and FUN_0000E4AC. depth = voice + part - 128, then the velocity term
    // multiplies it: depth * egvel_sens * (vel - 127) / 0x379 (or * vel for a negative sense), and the
    // sum is clamped to -64..64 and shipped as 0x120 * depth (FUN_0000CB1C). How many cutoff bytes the
    // chip makes of that word against the EG level is cal::FEG_DEPTH_BYTES.
    int egd = V.fegDepth + (ctrl_part(part, 23, pt.p[0x1F]) - 64);
    int egv = V.fltEgVel;
    egd = clampi(egd + sdiv((long long)(egv > 0 ? vel - 127 : vel) * egd * egv, 0x379), -64, 64);
    double cutf = cut + ks + C.feg.cur / 256.0 * egd / 64.0 * cal::FEG_DEPTH_BYTES;
    cutf += C.lfo2Val * eb86(lfo2d) / 16384.0 * 64.0;                              // LFO2 filter mod, INFERRED
    // FUN_0000DB3C: the part's resonance offset counts double, and the velocity sense scales
    // (vel - 127) * 0x74 / 0x379 (or vel * 0x74 / 0x379 for a negative sense) by the sense, then the
    // sum is clamped to the raw 0..116 the chip takes. fltReso here is the byte less 16, which reso_q
    // and reso_fb put back.
    int rv = V.fltResoVel;
    int reso = V.fltReso + 16 + (ctrl_part(part, 22, pt.p[0x19]) - 64) * 2 + sdiv((long long)(rv > 0 ? vel - 127 : vel) * 0x74, 0x379) * rv;
    reso = clampi(reso, 0, 116) - 16;
    C.flt.setup(C.fltType, cut_hz(cutf), reso);
}

void Synth::tick() {
    double dt = 1.0 / TICK_HZ;
    fseq_tick(dt);
    for (auto& C : ch) {
        if (!C.active) continue;
        const Part& pt = perf.part[C.part];
        lfo_tick(C, pt); lfo2_tick(C, pt); peg_tick(C); porta_tick(C); C.feg.tick(); refresh_regs(C, pt);
    }
}
