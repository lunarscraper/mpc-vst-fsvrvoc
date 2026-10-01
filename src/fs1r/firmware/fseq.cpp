// fs1r/firmware/fseq.cpp - formant sequence playback. KNOWN: rewritten from the disassembly.
#include "fs1r/internal.h"

bool Synth::fseq_on(int part) const {
    int m = perf.c[0x21] & 3;
    return fseq.valid && fseqPart == part && (m == 1 || m == 2) && fseqDelay <= 0;
}

void Synth::fseq_keys() {
    if (fseqPart < 0) { fseqHeld = false; return; }
    for (auto& c : ch) if (c.active && c.part == fseqPart && (c.held || c.sustained)) { fseqHeld = true; return; }
    fseqHeld = false;
}

void Synth::fseq_loop(int& lo, int& hi, int& dir) const {
    int ls = perf.c[0x1C] << 7 | perf.c[0x1D], le = perf.c[0x1E] << 7 | perf.c[0x1F];
    dir = le >= ls ? 1 : -1;
    lo = clampi(std::min(ls, le), 0, fseq.endStep); hi = clampi(std::max(ls, le), 0, fseq.endStep);
}

void Synth::fseq_start(int vel) {
    int ratio = perf.c[0x18] << 7 | perf.c[0x19];
    fseqClock = ratio < 100;                           // below 10.0 % the word is a MIDI clock division 0..4
    if (!fseqClock) {
        int sens = perf.c[0x22] & 7;
        ratio = clampi(ratio, 100, 5000);
        if (sens && ratio > 100) { int x = ((ratio - 100) * sens * (127 - vel)) / 7 / 127; ratio = clampi(ratio - x, 100, 5000); }
        ratio = clampi(ratio + ((ctrl_offset(fseqPart, 46) * 0x1324) >> 7), 100, 5000);   // ctrlDest_46
        double counts = (double)VELW[clampi(fseq.speedAdj, 0, 127)] * 84.0 * 1000.0 / ratio + 2884.0;
        fseqPeriod = counts * 32.0 / CPU_HZ;           // CMT1 at clock/32
    }
    int lo, hi, dir; fseq_loop(lo, hi, dir);
    int off = clampi(perf.c[0x1A] << 7 | perf.c[0x1B], 0, fseq.endStep);
    fseqDir = dir;
    fseqStep = dir > 0 ? off : clampi(fseq.endStep - off, 0, fseq.endStep);
    fseqAcc = 0; fseqClockAcc = 0; fseqRun = true; fseqHeld = true; fseqVel = vel;
    // FUN_0001A560: the delay is FSEQDLY[byte] frames of a fixed 7000-count CMT1 period (8 ms at clock/32),
    // counted down by FUN_0001A47A, and the real frame rate starts when it runs out.
    fseqDelay = FSEQDLY[clampi(perf.c[0x26], 0, 99)] * 7000.0 * 32.0 / CPU_HZ;
}

void Synth::fseq_step() {
    int lo, hi, dir; fseq_loop(lo, hi, dir); (void)dir;
    fseqStep += fseqDir;
    if ((perf.c[0x20] & 1) != 0) {
        if (fseqStep >= hi) { fseqStep = hi; fseqDir = -1; }
        else if (fseqStep <= lo) { fseqStep = lo; fseqDir = 1; }
    } else if (fseqHeld) {
        // FUN_0001A894 and FUN_0001A8E0 wrap to the far loop point and clear the run flags where the
        // two points are the same step, so a zero length loop advances once and then holds that frame.
        if (fseqDir > 0 && fseqStep > hi) { fseqStep = lo; if (lo == hi) fseqRun = false; }
        else if (fseqDir < 0 && fseqStep < lo) { fseqStep = hi; if (lo == hi) fseqRun = false; }
    } else {
        if (fseqStep >= fseq.endStep) { fseqStep = fseq.endStep; fseqRun = false; }
        else if (fseqStep <= 0) { fseqStep = 0; fseqRun = false; }
    }
}

void Synth::fseq_tick(double dt) {
    if (!fseqRun || !fseq.valid) return;
    if (fseqDelay > 0) { fseqDelay -= dt; if (fseqDelay > 0) return; }
    if ((perf.c[0x21] & 3) == 1) {                     // scratch: the controller is the transport
        int v = clampi(ctrl_offset(fseqPart, 47), -128, 127);       // ctrlDest_47, a signed byte
        fseqStep = clampi(((v + 128) * fseq.endStep) / 255, 0, fseq.endStep);
        return;
    }
    if (fseqClock) return;                             // MIDI clock drives it from midi_clock()
    fseqAcc += dt; if (fseqAcc < fseqPeriod) return;
    fseqAcc -= fseqPeriod;
    fseq_step();
}

void Synth::midi_clock() {
    if (!fseqRun || !fseqClock || !fseq.valid || fseqDelay > 0 || (perf.c[0x21] & 3) == 1) return;
    static const double rate[5] = {0.25, 0.5, 1.0, 2.0, 4.0};   // FUN_0001ACB6, see internal.h
    fseqClockAcc += rate[clampi(perf.c[0x18] << 7 | perf.c[0x19], 0, 4)];
    while (fseqClockAcc >= 1.0) { fseqClockAcc -= 1.0; fseq_step(); }
}

void Synth::fseq_bytes(std::vector<uint8_t>& d) const {
    d.assign(32 + (size_t)fseq.nframes * 50, 0);
    memcpy(d.data(), fseq.name, 8);
    d[0x10] = (uint8_t)(fseq.loopStart >> 7); d[0x11] = (uint8_t)(fseq.loopStart & 0x7F);
    d[0x12] = (uint8_t)(fseq.loopEnd >> 7); d[0x13] = (uint8_t)(fseq.loopEnd & 0x7F);
    d[0x14] = (uint8_t)fseq.loopMode; d[0x15] = (uint8_t)fseq.speedAdj; d[0x16] = (uint8_t)fseq.velTempo;
    d[0x17] = (uint8_t)fseq.pitchMode; d[0x18] = (uint8_t)fseq.noteAssign; d[0x19] = (uint8_t)fseq.tuning;
    d[0x1A] = (uint8_t)fseq.delay; d[0x1B] = (uint8_t)clampi(fseq.nframes / 128 - 1, 0, 3);
    d[0x1E] = (uint8_t)(fseq.endStep >> 7); d[0x1F] = (uint8_t)(fseq.endStep & 0x7F);
    memcpy(d.data() + 32, fseq.frame, (size_t)fseq.nframes * 50);
}
