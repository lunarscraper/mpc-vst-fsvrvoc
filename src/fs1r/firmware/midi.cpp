// fs1r/firmware/midi.cpp - MIDI in, sysex in and out. KNOWN: rewritten from the disassembly.
#include "fs1r/internal.h"

int Synth::ccSource(int cc) const {
    static const int sysOf[14] = {0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, -1, -1, -1, 0x1E, 0x1F, 0x1C, -2, 0x1D};
    for (int i = 0; i < 14; i++) {
        if (sysOf[i] == -2) { if (cc == 1) return i; continue; }
        if (sysOf[i] < 0) continue;
        if (i < 4 && !sys[0x14]) continue;   // knob receive switch off
        if (sys[sysOf[i]] == cc) return i;
    }
    return -1;
}

void Synth::set_source(Part& pt, int slot, int v) {
    pt.src[slot] = (SRC_RAW & (1 << slot)) ? v : ((v - 64) * 2 == 126 ? 127 : (v - 64) * 2);
}

bool Synth::part_listens(int p, int chn) const {
    int rc = perf.part[p].rcv();
    if (rc == 0x7F) return false;
    if (forceChannel >= 0) return chn == forceChannel;
    int pc = perfChannel();
    if (rc == 0x10) return pc == 0x10 || (pc != 0x7F && chn == pc);
    return rc == chn;
}

void Synth::midi_in(int st, int d1, int d2) {
    int chn = st & 0x0F; st &= 0xF0;
    if (st == 0xF0) {                        // system real time
        if (chn == 0x08) midi_clock();
        else if (chn == 0x0E) senseTimer = 0.5;
        else if (chn == 0x0A || chn == 0x0B) { fseqAcc = 0; fseqClockAcc = 0; }
        return;
    }
    int pc = perfChannel();
    bool onPerfCh = pc == 0x10 || (pc != 0x7F && chn == pc);
    if (st == 0xC0 && onPerfCh && sys[0x13] && perfBank >= 0x40) { load_perf_bank(perfBank, d1); return; }
    for (int p = 0; p < 4; p++) {
        if (!part_listens(p, chn)) continue;
        Part& pt = perf.part[p];
        switch (st) {
        case 0x90: if (d2) { note_on(p, d1, d2); fseq_keys(); break; }   // velocity 0 falls through
        case 0x80: note_off(p, d1); fseq_keys(); break;
        case 0xA0: pt.src[SRC_PAT] = d2; break;
        case 0xD0: pt.src[SRC_CAT] = d1; break;
        case 0xE0: pt.bend = (d2 - 64) * 16; pt.src[SRC_PB] = d2 - 64; break;
        case 0xC0: if (rom_ready(rom) && sys[0x13] && pt.p[1] && sys[8]) { pt.p[2] = (uint8_t)d1; rom_voice(*rom, bank_voice_index(pt.p[1], d1), pt.voice); } break;
        case 0xB0: control_change(p, d1, d2); break;
        }
    }
}

void Synth::control_change(int p, int cc, int v) {
    Part& pt = perf.part[p];
    int src = ccSource(cc);
    if (src >= 0) { set_source(pt, src, v); return; }
    if (cc == sys[0x20]) { pt.p[0x1D] = (uint8_t)v; return; }    // Formant knob control number
    if (cc == sys[0x21]) { pt.p[0x1E] = (uint8_t)v; return; }    // FM knob control number
    switch (cc) {
    case 0: if (sys[0x12]) bankMsb = v; break;
    case 32: if (sys[0x12] && bankMsb == 0x3F) { if (v <= 0x0B) pt.p[1] = (uint8_t)(v + 1); else if (v >= 0x40 && v <= 0x43) perfBank = v; } break;
    case 6: data_entry(p, v); break;
    case 7: pt.p[0x0B] = (uint8_t)v; break;
    case 11: pt.expr = std::max((int)pt.p[0x2C], std::min(254, v * 2)); break;
    case 64: set_sustain(p, v >= 64); break;
    case 65: pt.p[0x24] = (uint8_t)((pt.p[0x24] & 2) | (v >= 64)); break;
    case 5: pt.p[0x25] = (uint8_t)v; break;
    case 10: pt.p[0x0E] = (uint8_t)std::max(1, v); break;
    // The five sound controllers and the two sends, read out of the firmware's own per-CC handler
    // table at flash 0x3DA04: every entry that is not the ignore stub at 0x151C0 writes one part
    // parameter, and the handler's immediate is the address low it writes to. 71 -> 0x19, 72 ->
    // 0x1C, 73 -> 0x1A, 74 -> 0x18, 91 -> 0x13, 93 -> 0x12. The MIDI View screens in the EPROM
    // print the same pairs: RevSend "Bn 5B", VarSend "Bn 5D", Filter "Bn 4A".
    case 71: pt.p[0x19] = (uint8_t)v; break;    // filter resonance
    case 72: pt.p[0x1C] = (uint8_t)v; break;    // EG release time
    case 73: pt.p[0x1A] = (uint8_t)v; break;    // EG attack time
    case 74: pt.p[0x18] = (uint8_t)v; break;    // filter cutoff
    case 91: pt.p[0x13] = (uint8_t)v; break;    // reverb send
    case 93: pt.p[0x12] = (uint8_t)v; break;    // variation send, not 94
    case 98: pt.rpnL = v; pt.nrpnSel = true; break;
    case 99: pt.rpnM = v; pt.nrpnSel = true; break;
    case 100: pt.rpnL = v; pt.nrpnSel = false; break;
    case 101: pt.rpnM = v; pt.nrpnSel = false; break;
    case 120: all_off(); break;
    case 121: pt.reset_ctl(); break;
    case 123: all_release(); break;
    case 126: pt.p[5] = 0; break;
    case 127: pt.p[5] = 1; break;
    default: break;
    }
}

void Synth::data_entry(int p, int v) {
    Part& pt = perf.part[p];
    if (pt.nrpnSel) {
        if (pt.rpnM != 1) return;
        switch (pt.rpnL) {                   // NRPN 01 xx, Data List section 2.2.2
        case 0x08: pt.p[0x15] = (uint8_t)v; break;   // LFO1 speed
        case 0x09: pt.p[0x16] = (uint8_t)v; break;   // LFO1 pitch mod
        case 0x0A: pt.p[0x17] = (uint8_t)v; break;   // LFO1 delay
        case 0x0B: pt.p[0x2E] = (uint8_t)v; break;   // LFO2 speed
        case 0x0C: pt.p[0x2F] = (uint8_t)v; break;   // LFO2 filter mod
        case 0x20: pt.p[0x18] = (uint8_t)v; break;   // filter cutoff
        case 0x21: pt.p[0x19] = (uint8_t)v; break;   // filter resonance
        case 0x63: pt.p[0x1A] = (uint8_t)v; break;   // EG attack
        case 0x64: pt.p[0x1B] = (uint8_t)v; break;   // EG decay
        case 0x66: pt.p[0x1C] = (uint8_t)v; break;   // EG release
        }
        return;
    }
    if (pt.rpnM != 0) return;
    switch (pt.rpnL) {                       // RPN 00 00-02
    case 0: { int r = clampi(v, 0, 24); pt.p[0x26] = (uint8_t)(0x40 + r); pt.p[0x27] = (uint8_t)(0x40 - r); break; }
    case 1: pt.p[9] = (uint8_t)clampi(v, 0x0F, 0x70); break;
    case 2: pt.p[8] = (uint8_t)clampi(v - 0x28, 0, 0x30); break;
    }
}

void Synth::push_bulk(int ah, int am, int al, const uint8_t* d, int n) {
    std::vector<uint8_t> m{0xF0, 0x43, (uint8_t)(devNumber() & 0x0F), 0x5E,
                           (uint8_t)(n >> 7 & 0x7F), (uint8_t)(n & 0x7F),
                           (uint8_t)ah, (uint8_t)am, (uint8_t)al};
    int sum = (n >> 7 & 0x7F) + (n & 0x7F) + ah + am + al;
    for (int i = 0; i < n; i++) { uint8_t b = d[i] & 0x7F; m.push_back(b); sum += b; }
    m.push_back((uint8_t)((-sum) & 0x7F)); m.push_back(0xF7);
    push_sysex(m);
}

void Synth::push_param(int ah, int am, int al, int val) {
    std::vector<uint8_t> m{0xF0, 0x43, (uint8_t)(0x10 | (devNumber() & 0x0F)), 0x5E,
                           (uint8_t)ah, (uint8_t)am, (uint8_t)al,
                           (uint8_t)(val >> 7 & 0x7F), (uint8_t)(val & 0x7F), 0xF7};
    push_sysex(m);
}

int Synth::read_param(int ah, int am, int al) const {
    if (ah == 0x00 && am == 0 && al < 76) return sys[al];
    if (ah == 0x10 && am == 0 && al < 80) return perf.c[al];
    if (ah == 0x10 && ((am == 0 && al >= 0x50) || am == 1)) { int i = am ? 48 + al : al - 0x50; return i < 112 ? perf.fx[i] : -1; }
    if (ah >= 0x30 && ah <= 0x33 && am == 0 && al < 52) return perf.part[ah - 0x30].p[al];
    if (ah >= 0x40 && ah <= 0x43 && am == 0 && al < 112) return perf.part[ah - 0x40].voice.raw[al];
    if (ah >= 0x60 && ah <= 0x63 && am < 8 && al < 62) return perf.part[ah - 0x60].voice.raw[112 + am * 62 + al];
    return -1;
}

void Synth::current_perf_bytes(uint8_t* out) const {
    memcpy(out, perf.c, 80); memcpy(out + 80, perf.fx, 112);
    for (int i = 0; i < 4; i++) memcpy(out + 192 + 52 * i, perf.part[i].p, 52);
}

void Synth::dump_request(int ah, int am, int al) {
    if (ah == 0x00) { push_bulk(0x00, 0, 0, sys, 76); return; }
    if (ah == 0x10 || ah == 0x11) { uint8_t d[400]; current_perf_bytes(d); push_bulk(ah, am, al, d, 400); return; }
    if (ah >= 0x40 && ah <= 0x43) { push_bulk(ah, 0, 0, perf.part[ah - 0x40].voice.raw, 608); return; }
    if (ah == 0x51) { push_bulk(0x51, am, al, perf.part[0].voice.raw, 608); return; }
    if ((ah & 0xF0) == 0x60 && (ah & 0x0F) <= 1 && fseq.valid) {   // 6b 00 nn, b = bank
        std::vector<uint8_t> d; fseq_bytes(d);
        push_bulk(ah, am, al, d.data(), (int)d.size());
    }
}
