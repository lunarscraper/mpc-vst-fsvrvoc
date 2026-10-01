// fs1r/chips/ymp706.cpp - the tone generator, modelled. INFERRED: the constants are in chips/cal.h.
#include "fs1r/internal.h"

float g_win[2][8][1025];       // [asymmetric?][skirt]: the two window families
float g_winDC[2][8];           // each window's mean, which a grain under a DC carrier does not carry

void init_tables() {
    fm::init();
    // MEASURED 2026-09-19 (FS1R.unlock's skirt sweep, all sixty partials) and 2026-09-24 (the same take
    // read again, plus 04_formant_2): the grain window is sin^p with p = 2 * 2^skirt on every form, and
    // there are two families. The "2" forms (all2, odd2, res2) are the symmetric window, whose line
    // amplitudes are the binomial row C(p, p/2 - k) to 0.4 to 0.7 dB over eight skirts. The "1" forms
    // (all1, odd1, res1) and the formant are the SAME exponent on the rising half only; the falling
    // half stays sin^2 whatever the skirt. That is what the period-averaged waveform shows directly
    // (a rise that sharpens with the skirt on a fall that never moves) and it puts all1, odd1 and res1
    // within 0.4 to 1.3 dB over every skirt where a sin^p at any exponent left 6 to 17. The formant is
    // that same asymmetric window stretched over its bandwidth: 04_formant_2's sixteen skirt segments
    // land inside 0.4 to 1.8 dB where the sqrt(2)-per-step symmetric fit could not get under 2.5.
    // docs/skirt.md, docs/formant.md.
    for (int f = 0; f < 2; f++)
        for (int s = 0; s < 8; s++) {
            double p = cal::WIN_SKIRT * pow(cal::WIN_SKIRT_STEP, s);
            double acc = 0;
            for (int i = 0; i <= 1024; i++) {
                double x = sin(PI * i / 1024.0);
                g_win[f][s][i] = (float)pow(x, f && i > 512 ? cal::WIN_SKIRT : p);
                if (i < 1024) acc += g_win[f][s][i];
            }
            g_winDC[f][s] = (float)(acc / 1024.0);
        }
}

double Synth::op_sample(OpState& s, const OpV& v, double f0, double fop, int ratio, double pm, double gain) {
    if (v.form <= 2) {
        s.phase += fop / SR; if (s.phase >= 1) s.phase -= 1;
        if (v.form == 0) return gain * fsin(s.phase + pm);
        // all1 and all2 are a stored waveform read by phase, the same way the sine is: one period of
        // the skirt's window with its mean taken out, so a modulator reaches it as phase like any other
        // form. MEASURED 2026-09-24 off the skirt sweep's sixty partials: all2's lines are the symmetric
        // window's own Fourier row C(p, p/2 - k) to 0.4 to 0.7 dB at every skirt, and all1's the
        // asymmetric window's to 0.4 to 1.3, where the two-period grain under a carrier at fop that this
        // replaces was 36 to 66 dB out by skirt 7. At skirt 0 the DC-free sin^2 is -cos / 2, the single
        // partial the unit gives, and the mean is gone because the unit's is: every all1/all2 take reads
        // mean / rms under 0.03 where a raw sin^2 pulse train sits at 0.7. docs/skirt.md.
        double x = s.phase + pm; x -= floor(x);
        return gain * cal::FORM_LEVEL * 2.0 * (fwin(v.form == 1, v.skirt, x) - g_winDC[v.form == 1][v.skirt]);
    }
    double fw, fc, wl;
    if (v.form == 7) { fw = f0; fc = fop; wl = s.wl7; }                                                          // formant: window at the fundamental (INFERRED bw curve)
    // odd1/odd2 are a group that starts at the operator's own frequency, so they are the same windowed
    // carrier the formant is, with the carrier at fop. MEASURED: the unit gives partials 1 and 3 on odd1
    // and odd2, where the fixed quarter-period window at DC this replaces gave a full harmonic series.
    // Retriggering at twice fop is what leaves the odd partials: a grain train at that spacing under a
    // carrier at fop puts its lines at fop, 3 fop, 5 fop and nowhere else.
    //
    // The window is FIXED, and the bandwidth has nothing to do with it. MEASURED on 2026-09-19 by
    // sweeping register 0x218 through all hundred values on each form in turn: the formant's window
    // opens and every one of the other six is flat to the byte, width and peak both. Voice byte 6 does
    // reach those forms, but as register 0x230, which the Data List calls the "freq. ratio of band
    // spectrum" and which the voice image confirms carries the raw byte for them and the formant
    // transpose word for the formant. What it does there is unmeasured: the sweep meant to settle it
    // wrote 0x218. INFERRED, FS1R.unlock/docs/unknowns.md experiment 9.
    // res1 and res2 put a three-partial group on harmonic ratio + 1 and are otherwise identical.
    // MEASURED on 2026-09-19 by sweeping register 0x230 through all hundred values on each: the peak
    // lands on partial ratio + 1 at every one of the hundred, on both forms, with the two partials
    // either side 6 dB down and everything else at the noise floor, and the peak level flat across
    // the whole range. The 1 + ratio * 31 / 99 this replaces topped out at 32 times the fundamental
    // where the unit reaches 100. docs/formant.md.
    // The window is one grain period for the odd and resonant forms; the "1" and "2" of each pair
    // differ in the window's shape (g_win), not its length. MEASURED off the sideband ratios, which a
    // sin^2 window fixes exactly: a grain exactly one period long makes the two partials either side
    // of the peak 6.02 dB down and kills everything beyond them, which is what the unit gives res1
    // and res2 at every one of a hundred settings.
    else if (v.form == 5 || v.form == 6) { fw = fop; fc = fop * (ratio + 1); wl = 1.0; }
    else { fw = 2 * fop; fc = fop; wl = 1.0; }
    wl = std::min(wl, 2.0);
    s.fphase += fw / SR;
    if (s.fphase >= 1) {
        s.fphase -= 1; WinGen& g = s.g[s.nextGen]; g.on = true; g.w = 0; g.fc = fc;
        // The formant restarts its carrier every grain, which is what holds its peak still while the
        // fundamental moves. The odd forms retrigger twice a period, so restarting there would put
        // half a cycle of alternation into the grain train and land the group on the even partials;
        // half a cycle back is the same carrier running on.
        g.c = (v.form == 3 || v.form == 4) && (s.halfCount & 1) ? 0.5 : 0.0;
        s.nextGen ^= 1; s.halfCount++;
    }
    double winc = fw / (wl * SR), y = 0;
    // The "1" forms and the formant take the asymmetric window, the "2" forms the symmetric one.
    int fam = v.form == 3 || v.form == 5 || v.form == 7;
    for (int k = 0; k < 2; k++) {
        WinGen& g = s.g[k]; if (!g.on) continue;
        g.w += winc; if (g.w >= 1) { g.on = false; continue; }
        y += gain * fwin(fam, v.skirt, g.w) * fsin(g.c + pm); g.c += g.fc / SR;
    }
    if (v.form == 7) { if (cal::FRMT_NORM != 0.0) y *= pow(1.0 / wl, cal::FRMT_NORM); }
    else y *= cal::FORM_LEVEL * 2.0 / wl;    // every form but sine and frmt; a shorter grain carries less
    return y;
}

void Synth::refresh_ctl(Chan& C) {
    const Part& pt = perf.part[C.part]; const Voice& V = pt.voice; const unsigned char* alg = FS1R_ALG[V.alg];
    bool fs = fseq_on(C.part);
    C.f0 = word_hz(C.regPitch + 0x1243 + C.regPM);                 // channel fundamental incl. LFO pitch mod (pms 7)
    C.fbGain = V.fb ? cal::FEEDBACK * pow(2.0, V.fb - 7) : 0.0;    // INFERRED feedback scale
    for (int o = 0; o < 8; o++) {
        OpState& s = C.op[o]; const OpV& v = V.v[o]; const OpU& u = V.u[o];
        s.att = C.regLevel[o] * LEVEL_DB + am_att(C.regAM, v.ams);
        if (alg[2 * o + 1] & 1) s.att += cal::CARRIER_DB * V.corr[o];         // carrier level correction (bits in the 0x200 word), 1.5 dB steps INFERRED
        // Ours, and bit-identical: an operator whose EG is over, with no grain in flight and no level
        // boost that could lift it back over the -100 dB gate before the next refresh, never reads its
        // frequency maths again. The carrier correction can reach -22.5 dB, hence the -20 dB bound on
        // both the target attenuation and the slewed one. MEASURED 2026-09-28 on the Fat Line demo:
        // refresh_ctl was a quarter of the render and half of that was for finished operators.
        if (s.eg.done() && !s.g[0].on && !s.g[1].on && s.att >= -20 && s.attS >= -20) goto unvoiced;
        {
        int pmw = (int)(C.regPM * cal::PMS_FRAC[v.pms]) + C.vcFreq[o][0];
        // The detune word goes into the frequency word rather than onto the result: it is a pitch
        // word offset on the chip, and the formant branch already carries it inside frmtWord.
        int det = v.form == 7 ? 0 : C.detW[o];
        double fop;
        // A frame's frequency word goes into the operator's own frequency register, it does not stand
        // in for the whole register chain. A ratio operator still has the channel pitch added on top,
        // which is what the format means for one: the word is an absolute formant centre, so a ratio
        // operator handed one runs off the end of the register. MEASURED 2026-09-21: 12_fseqlevel's
        // three sine segments sum to 39632..45056 at notes 36, 60 and 84 and the unit gives the same
        // 23982 Hz line in all three, the saturated word, where the engine read the frame word alone
        // and sang at 275 Hz. Formant and fixed operators take the word as it stands.
        if (fs && C.fseqOp[o]) fop = word_hz(C.fqWord[o] + pmw + det + (v.form != 7 && !v.fixed ? C.regPitch : 0));
        else if (v.form == 7) fop = word_hz(C.freqWord[o] + C.fbW[o] + (C.frmtWord[o] - 0x1243) + pmw + (C.regFM * v.fms) / 7);
        else if (!v.fixed) fop = word_hz(C.freqWord[o] + C.regPitch + pmw + det);
        else fop = word_hz(C.freqWord[o] + C.fbW[o] + pmw + det + (C.regFM * v.fms) / 7);
        s.fop = fop;
        // The formant's window runs at the channel pitch through the operator's own pitch mod sense, not
        // through the full LFO word: rgwan's demo take of 15_Human (pms 1..4, pmd 12) wobbles its formant
        // harmonics 33..48 cents, the same as its pms 2 sine carriers, where a window on C.f0 gave 541.
        s.fw = word_hz(C.regPitch + 0x1243 + pmw - C.vcFreq[o][0]);
        s.bw = clampi(C.bwReg[o] + C.vcBw[o][0], 0, 99);                 // register 0x218, the formant's
        s.ratio = v.form == 7 ? 0 : clampi(C.frmtWord[o], 0, 99);         // register 0x230, every other form's
        s.wl7 = std::min(cal::FRMT_WL_MAX, C.f0 / (cal::FRMT_BW_HZ0 * fm::exp2(s.bw / cal::FRMT_BW_DB)));
        }
        unvoiced:
        if (s.ueg.done()) continue;   // the same, and simpler: the unvoiced attenuation is never negative, so a finished noise EG is gated for good
        // unvoiced (noise formant) operator
        s.uatt = C.regULevel[o] * LEVEL_DB + am_att(C.regAM, u.ams);
        double nf;
        if (fs && C.fseqUOp[o]) nf = word_hz(C.fquWord[o]);
        else if (u.mode == 2 && v.form == 7) nf = word_hz(C.freqWord[o] + (C.frmtWord[o] - 0x1243));
        // MEASURED: link-ff on a voiced partner that is not a formant operator reaches register 0x300 as
        // link-fo, not as itself. uv-linkff-sine's image reads mode 1 where the sysex asked for 2.
        else if (u.mode) nf = C.f0;
        else nf = word_hz(C.ufreqWord[o] + C.ufbW[o] + (C.regFM * u.fms) / 7 + C.vcFreq[o][1]);
        s.nf = nf * fm::exp2(u.transpose / 12.0);
        // MEASURED noise formant, docs/noise.md and cal.h: two unequal digital one-poles on white noise,
        // their coefficients and the band's peak gain read off the tables against the register and the
        // skirt. The gain is the band's PEAK, not its RMS, which is what the fit measured; the RMS follows
        // from the coefficients and is what the resonance carrier is set against.
        int ureg = clampi(C.ubwReg[o] + C.vcBw[o][1], 0, 127);
        NoiseBand nb = noise_band(ureg, u.skirt);
        s.na = nb.a1; s.na2 = nb.a2;
        double resDc, resNoise; ures(ureg, u.res, resDc, resNoise);
        s.nscale = db2lin(nb.gdb + cal::NOISE_LEVEL_DB) * resNoise;
        // Bandwidth register 0 silences the noise and leaves the resonance carrier alone. The capture set
        // cannot see the split, since every segment it has at bandwidth 0 is at resonance 0 too, and the
        // demo settles it: Kalimba runs two unvoiced operators at bandwidth 0 with resonance 5 and 7, and
        // taking the carrier down with the noise costs it 3.9 dB of band tilt where leaving it alone costs
        // nothing anywhere else. So the carrier is set against the band the register would give at 5.
        NoiseBand nc = ureg == 0 ? noise_band(5, u.skirt) : nb;
        s.nres = db2lin(nc.gdb + cal::NOISE_LEVEL_DB) * sqrt(noise_band_var(nc.a1, nc.a2)) * resDc;
    }
}

void Synth::render_chan(Chan& C, double& outL, double& outR) {
    if (C.silent) {
        // Ours. The carriers and the noise are over and the filter has rung down, so the channel's output
        // is zero from here to its end; what is left is when the allocator may treat it as free, which
        // is the sample its last envelope passes -120 dB. Every envelope still running is in its release
        // and falling linearly, so that sample is arithmetic, and the channel costs nothing until then.
        // The hardware renders these channels in full, at no cost; on the Fat Line demo they were half
        // of the 32 channels for the whole song (docs/performance.md, 2026-09-28).
        if (C.freezeLeft < 0) {
            long long worst = 0; bool can = true;
            for (auto& s : C.op) for (const EG* e : {&s.eg, &s.ueg}) {
                if (e->done()) continue;
                if (e->stage != 4 || e->rising || e->rate <= 0) { can = false; break; }
                long long n = (long long)((e->cur + 120.0) / e->rate) + 2; if (n > worst) worst = n;
            }
            if (can) C.freezeLeft = worst;
        }
        if (C.freezeLeft >= 0) { if (--C.freezeLeft <= 0) C.active = false; }
        else {
            // An envelope still rising, or holding: tick them until they are all falling.
            bool alive = false;
            for (auto& s : C.op) { s.eg.tick(); s.ueg.tick(); if (!s.eg.done() || !s.ueg.done()) alive = true; }
            if (!alive) C.active = false;
        }
        outL = outR = 0.0; C.lastL = C.lastR = 0.0;
        return;
    }
    if (C.ctlLeft-- <= 0) { C.ctlLeft = tuning::CTL_DECIMATION - 1; refresh_ctl(C); }
    const Voice& V = perf.part[C.part].voice; const unsigned char* alg = FS1R_ALG[V.alg];
    double partV = C.partV, partU = C.partU;
    double Cb = 0, H = 0, S = 0, fbNew = 0, mix = 0;
    for (int o = 0; o < 8; o++) {
        OpState& s = C.op[o]; const OpV& v = V.v[o];
        unsigned char t0 = alg[2 * o], t1 = alg[2 * o + 1]; int F = (t0 >> 3) & 7;
        double in = F == 3 ? C.fbBus * C.fbGain : F == 4 ? Cb : F == 5 ? H : F == 6 ? S : 0.0;
        if (F == 6) S = 0;
        double egdb = s.eg.tick();
        double y = 0;
        // The level register glides rather than stepping, so a frame write reaches the output over
        // about a millisecond instead of cutting the waveform. A silenced operator still runs while a
        // grain it already fired is in flight, so the last one fades out under its own window.
        if (s.attS < -1e8) s.attS = s.att; else s.attS += (s.att - s.attS) * LEVEL_SLEW_K;
        double gain = egdb - s.attS > -100 ? db2lin_fast(egdb - s.attS) : 0.0;
        if (gain != 0.0 || s.g[0].on || s.g[1].on) {
            double fop = s.fop;
            // A FIXED-frequency operator gets no frequency EG. MEASURED 2026-09-25 off 24_onset's
            // onset-feg segment: init +25 with attack time 20 on a fixed carrier holds 1007.6 Hz flat
            // on the unit through the whole sweep, while the engine started 939 cents sharp and took
            // 25 ms to come down; the two agree to 1 cent once the ramp is over. That is consistent
            // with the ratio-mode measurement on 2026-09-25 (07_modulation_2, the six feg-* segments,
            // 1 to 5 cents), which used sine() carriers: the FEG rides the note's pitch word, and a
            // fixed operator does not take that word at all, it takes its own coarse/fine bytes.
            // Formant operators are NOT covered either way by any capture, so they keep the EG here
            // rather than being changed on a guess.
            if (!v.fixed && s.feg.stage < 2) fop *= fm::exp2(s.feg.tick() / 12.0);
            y = op_sample(s, v, s.fw, fop, s.ratio, in * FM_INDEX, gain);
        }
        Cb = y; if (t0 & 2) H = y; if (t1 & 4) S += y; if (t0 & 4) fbNew = y;
        if (t1 & 1) mix += y * partV;
        // unvoiced (noise formant) operator
        const OpU& u = V.u[o];
        double uegdb = s.ueg.tick();
        if (uegdb - s.uatt > -100) {
            double nf = s.nf;
            if (s.ufeg.stage < 2) nf *= fm::exp2(s.ufeg.tick() / 12.0);
            s.rng ^= s.rng << 13; s.rng ^= s.rng >> 17; s.rng ^= s.rng << 5;
            // Uniform in [-1, 1) scaled to unit variance, so the tables in cal.h read in the band's own RMS.
            double nz = ((int32_t)s.rng) * (1.7320508 / 2147483648.0);
            s.lp[0] += (nz - s.lp[0]) * s.na; s.lp[1] += (s.lp[0] - s.lp[1]) * s.na2;
            nz = s.lp[1] * s.nscale + s.nres;
            s.nphase += nf / SR; if (s.nphase >= 1) s.nphase -= 1;
            // The unvoiced output is the mean of this sample and the last, see cal.h: the unit's noise
            // falls off above 8 kHz on a cos^2 that no voiced operator shows, whichever centre it is at.
            double u1 = nz * fsin(s.nphase);
            mix += 0.5 * (u1 + s.uprev) * db2lin_fast(uegdb - s.uatt) * partU;
            s.uprev = u1;
        }
    }
    C.fbBus = (fbNew + C.fbPrev) * 0.5; C.fbPrev = fbNew;
    bool alive = false;
    for (auto& s : C.op) if (!s.eg.done() || !s.ueg.done()) { alive = true; break; }
    if (!alive) C.active = false;
    // The channel goes silent once every carrier's EG is over with no grain in flight, every noise EG
    // is over, and the filter has rung down: 64 samples of nothing, with no damp left from the note before.
    bool carriersDone = true;
    for (int o = 0; o < 8; o++) { const OpState& s = C.op[o]; if (((alg[2 * o + 1] & 1) && (!s.eg.done() || s.g[0].on || s.g[1].on)) || !s.ueg.done()) { carriersDone = false; break; } }
    if (carriersDone && fabs(mix) < 1e-9 && C.dampL == 0.0 && C.dampR == 0.0) { if (++C.quiet >= 64) C.silent = true; } else C.quiet = 0;
    // The channel accumulator saturates before the filter loop, which is where the capture puts it:
    // eight carriers stacked in one channel clip flat, while the filter's own output on ingain-12 rides
    // 4 dB above that ceiling, so nothing downstream of CHOUT can be what clips.
    mix = std::clamp(mix, -cal::CHAN_CLIP, cal::CHAN_CLIP);
    if (C.fltOn) mix = C.flt.run(C.fltType, mix * C.fltGain);
    outL = mix * C.panL; outR = mix * C.panR;
    // Remember this channel's own output, so a note-on that takes this channel has something to fade,
    // and add whatever the previous note left behind (FUN_00023000, see note_on).
    C.lastL = outL; C.lastR = outR;
    if (C.dampL != 0.0 || C.dampR != 0.0) {
        outL += C.dampL; outR += C.dampR;
        C.dampL *= DAMP_K; C.dampR *= DAMP_K;
        if (fabs(C.dampL) < 1e-7 && fabs(C.dampR) < 1e-7) C.dampL = C.dampR = 0.0;
        else C.active = true;                      // a damp still sounding keeps the channel rendering
    }
}

void Synth::render(float* outL, float* outR, int frames) {
    std::lock_guard<std::mutex> lk(mtx);
    fx.configure(perf.fx);
    double insLvl = sendlvl(perf.fx[0x63]), insRev = sendlvl(perf.fx[0x61]), insVar = sendlvl(perf.fx[0x62]);
    double varRev = sendlvl(perf.fx[0x5E]), varRet = sendlvl(perf.fx[0x5D]), revRet = sendlvl(perf.fx[0x5A]);
    int vp = clampi(perf.fx[0x5C] - 1, 0, 126), rp = clampi(perf.fx[0x59] - 1, 0, 126);
    double vpl = db2lin(-LEVEL_DB * PANL[vp]), vpr = db2lin(-LEVEL_DB * PANR[vp]);
    double rpl = db2lin(-LEVEL_DB * PANL[rp]), rpr = db2lin(-LEVEL_DB * PANR[rp]);
    double pvol = perf.c[0x10] / 127.0;
    double dry[4], varS[4], revS[4]; bool insSw[4];
    for (int p = 0; p < 4; p++) {
        const uint8_t* q = perf.part[p].p;
        insSw[p] = (q[0x14] & 1) != 0;
        dry[p] = sendlvl(q[0x11]);
        varS[p] = sendlvl(ctrl_part(p, 20, q[0x12])); revS[p] = sendlvl(ctrl_part(p, 19, q[0x13]));
    }
    sense_tick(frames / (double)SR);
    for (int i = 0; i < frames; i++) {
        tickAcc += TICK_HZ / SR; while (tickAcc >= 1) { tickAcc -= 1; tick(); }
        double pl[4] = {}, pr[4] = {};
        for (auto& c : ch) if (c.active) { double a, b; render_chan(c, a, b); pl[c.part] += a; pr[c.part] += b; }
        double dL = 0, dR = 0, iL = 0, iR = 0, vL = 0, vR = 0, rL = 0, rR = 0;
        for (int p = 0; p < 4; p++) {
            if (insSw[p]) { iL += pl[p]; iR += pr[p]; continue; }
            dL += pl[p] * dry[p]; dR += pr[p] * dry[p];
            vL += pl[p] * varS[p]; vR += pr[p] * varS[p];
            rL += pl[p] * revS[p]; rR += pr[p] * revS[p];
        }
        // An effect block handed parameters no preset would use can run away. Left alone, one bad
        // block poisons the master EQ and silences everything after it, so a block whose output
        // leaves the sane range is emptied and muted for that sample instead.
        auto sane = [](FxBlock& blk, double& a, double& b) {
            if (!(a > -1e6 && a < 1e6) || !(b > -1e6 && b < 1e6)) { blk.clearState(); a = b = 0.0; }
        };
        double oL, oR;
        fx.ins.process(iL, iR, oL, oR);
        sane(fx.ins, oL, oR);
        dL += oL * insLvl; dR += oR * insLvl;
        vL += oL * insVar; vR += oR * insVar;
        rL += oL * insRev; rR += oR * insRev;
        fx.var.process(vL, vR, oL, oR);
        sane(fx.var, oL, oR);
        rL += oL * varRev; rR += oR * varRev;
        double l = dL + oL * varRet * vpl, r = dR + oR * varRet * vpr;
        fx.rev.process(rL, rR, oL, oR);
        sane(fx.rev, oL, oR);
        l += oL * revRet * rpl; r += oR * revRet * rpr;
        if (!(l > -1e6 && l < 1e6) || !(r > -1e6 && r < 1e6)) {
            for (auto& q : fx.eq) q.reset();
            l = r = 0.0;
        }
        fx.master(l, r);
        // Down to the scale of the digital output board, which is what the capture measured, and then
        // the main DAC's own ceiling. gain is ours: it stands in for the analogue volume pot, which on
        // the hardware sits after the tap and so cannot be part of the measurement.
        l *= cal::OUT_GAIN * pvol; r *= cal::OUT_GAIN * pvol;
        // An effect block given parameters no preset would use can run away or go non-finite. The
        // engine is a plugin: whatever happens upstream, it must not hand the host a NaN.
        if (!(l > -1e6 && l < 1e6)) l = 0.0;
        if (!(r > -1e6 && r < 1e6)) r = 0.0;
        l = std::clamp(l, -1.0, 1.0) * gain; r = std::clamp(r, -1.0, 1.0) * gain;
        outL[i] = (float)l; outR[i] = (float)r;
    }
}
