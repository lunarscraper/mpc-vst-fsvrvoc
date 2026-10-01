// fs1r/internal.h - the engine's shared declarations. Not a public header; src/fs1r.h is.
//
// Everything the engine's translation units need to see of each other: the data structures the
// firmware's own formats decode into, the chip-side structures they drive, and struct Synth, which is
// the whole machine. Method bodies live beside the thing they belong to, in fs1r/firmware for what is
// rewritten from the disassembly and fs1r/chips for what is modelled from measurement.
#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include "../fs1r.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>
#include "firmware/algorithms.h"
#include "firmware/tables.h"
#include "hardware.h"
#include "chips/cal.h"
#include "../fsvr/tuning.h"
#include "../fsvr/fastmath.h"

// ------------------------------------------------------------------------------------------ firmware helpers
static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }
// The chip's LFO amplitude modulation, measured off 14_sens (three LFO depths by eight sensitivities):
// the channel AM word is 7 bits, so it floors at 127 steps (48 dB) where the CPU can send 255, and the
// per-operator sensitivity weights it {0,1,2,4,5,6,7,8}/8, not k/7. docs/findings.md 2026-09-23.
static const int AMS_W[8] = {0, 1, 2, 4, 5, 6, 7, 8};
static inline double am_att(int regAM, int ams) { return std::min(regAM, 127) * LEVEL_DB * AMS_W[ams & 7] / 8.0; }
static inline int eb86(int v) { return std::min(255, (((v & 0xFF) << 1) * 0xA5) >> 7); }   // 0..99 -> 0..127
static inline int eb70(int v) { return (((v & 0xFF) << 1) * 0xA5) >> 8; }                   // 0..99 -> 0..127 (bandwidth)
// The unvoiced resonance carrier and the noise it takes with it, cal::URES_*, interpolated over the register.
static inline void ures(int reg, int res, double& dc, double& noiseGain) {
    if ((res & 7) < 4) { dc = 0; noiseGain = 1; return; }
    double r = std::clamp((double)reg, (double)cal::URES_REG[0], (double)cal::URES_REG[4]);
    int i = 0; while (i < 3 && cal::URES_REG[i + 1] <= r) i++;
    double f = (r - cal::URES_REG[i]) / (cal::URES_REG[i + 1] - cal::URES_REG[i]); int k = (res & 7) - 4;
    dc = cal::URES_DC[i][k] + (cal::URES_DC[i + 1][k] - cal::URES_DC[i][k]) * f;
    noiseGain = pow(10.0, (cal::URES_NOISE_DB[i][k] + (cal::URES_NOISE_DB[i + 1][k] - cal::URES_NOISE_DB[i][k]) * f) / 20.0);
}
static inline int egrate(int t) { return ((99 - clampi(t, 0, 99)) * 0xA4) >> 8; }          // EG time -> chip rate 0..63
// The chip's own rate scaling: register 0x50 is the operator's time scaling 0..7 and register 0xC0 the key code.
// MEASURED 2026-09-24 off 21_keycode: every semitone 12..120 at time scaling 7, decaying at nominal rate 34,
// each slope landing on the (4 + (q & 3)) << (q >> 2) ladder (fit error 0.06 over 109 notes). With the twenty
// tscale-3 points of 10_envelope2 that pins the chip's key offset x at every key code 77..113: the rate moves
// by trunc(tscale * x / 8), x saturates at -13 below 82 and +13 above 109, and it is symmetric about 95.5.
// Key codes 86/87 and 104/105 admit 8 or 9; 8 is what 19_drums' note 41 wanted. docs/aeg.md.
static const signed char EG_KEYOFF[37] = {   // key codes 77 .. 113
    -13, -13, -13, -13, -13, -12, -12, -10, -10, -8, -8, -5, -5, -4, -4, -2, -2, -2, -1,
      1,   2,   2,   2,   4,   4,   5,   5,   8,   8, 10, 10, 12, 12, 13, 13, 13, 13};
static inline int eg_keyoff(int c0) { return EG_KEYOFF[clampi(c0 - 77, 0, 36)]; }
// The noise band's two coefficients and its peak gain against the bandwidth register and the skirt,
// read off cal.h's tables: piecewise linear in the register, and the skirt a per-step multiplier that
// is itself interpolated across the three registers it was measured at.
struct NoiseBand { double a1, a2, gdb; };
static inline NoiseBand noise_band(int reg, int skirt) {
    reg = clampi(reg, 0, cal::NOISE_BW_CLAMP);
    int i = 0; while (i < 14 && cal::NOISE_REG[i + 1] <= reg) i++;
    double f = (double)(reg - cal::NOISE_REG[i]) / (cal::NOISE_REG[i + 1] - cal::NOISE_REG[i]);
    NoiseBand b = {cal::NOISE_A1[i] + (cal::NOISE_A1[i + 1] - cal::NOISE_A1[i]) * f,
                   cal::NOISE_A2[i] + (cal::NOISE_A2[i + 1] - cal::NOISE_A2[i]) * f,
                   cal::NOISE_G_DB[i] + (cal::NOISE_G_DB[i + 1] - cal::NOISE_G_DB[i]) * f};
    if (skirt > 0) {
        double r = std::clamp((double)reg, cal::NOISE_SK_REG[0], cal::NOISE_SK_REG[2]);
        int k = r < cal::NOISE_SK_REG[1] ? 0 : 1;
        double g = (r - cal::NOISE_SK_REG[k]) / (cal::NOISE_SK_REG[k + 1] - cal::NOISE_SK_REG[k]);
        double m1 = cal::NOISE_SK_M1[k] + (cal::NOISE_SK_M1[k + 1] - cal::NOISE_SK_M1[k]) * g;
        double m2 = cal::NOISE_SK_M2[k] + (cal::NOISE_SK_M2[k + 1] - cal::NOISE_SK_M2[k]) * g;
        double dg = cal::NOISE_SK_DG[k] + (cal::NOISE_SK_DG[k + 1] - cal::NOISE_SK_DG[k]) * g;
        b.a1 = std::min(1.0, b.a1 * pow(m1, skirt)); b.a2 = std::min(1.0, b.a2 * pow(m2, skirt)); b.gdb += dg * skirt;
    }
    return b;
}
// Variance of two one-poles in series, each with unit DC gain, driven by unit variance white noise.
static inline double noise_band_var(double a1, double a2) {
    double p1 = 1 - a1, p2 = 1 - a2;
    if (std::abs(p1 - p2) < 1e-6) { double q = 1 - p1 * p1; return a1 * a1 * a1 * a1 * (1 + p1 * p1) / (q * q * q); }
    double s = p1 * p1 / (1 - p1 * p1) + p2 * p2 / (1 - p2 * p2) - 2 * p1 * p2 / (1 - p1 * p2);
    return a1 * a1 * a2 * a2 / ((p1 - p2) * (p1 - p2)) * s;
}
static inline int eg_ratescale(int tscale, int c0) {
    int x = (tscale & 7) * eg_keyoff(c0);
    return x < 0 ? -((-x) >> 3) : x >> 3;                                                 // truncates toward zero
}
// Pan key scaling, MEASURED off 10_envelope2 on 2026-09-19. Every request file leaves the performance's
// PAN SCALING byte at 0, the extreme, so the recording sweeps the pan right across the keyboard and reads
// the law out: the index is 64 - 4 * (note - 60) / 3, hard right at note 12 and hard left from note 108
// up, which matches all ten notes and both ends of the table. That is 64 index steps per 48 semitones at
// byte 0, where the engine had 50, so every pan-scaled patch was 28 % shy of the unit. In the firmware's
// own 0..255 pan domain (docs/ymp706_registers.md, Pan) a byte of 0 is the whole 128 of one side.
static inline int pan_index(int base, int scaling, int note) {
    return base + ((clampi(scaling, 0, 100) - 50) * 64 / 50) * (note - 60) / 48;
}
static inline int keygroup(int n) { return std::max(0, (KEYFACT[clampi(n, 0, 127)] >> 2) - 3); }
// FUN_00010d7a: key tracking of fixed/formant frequencies, notescale 0..99, pm = pitch word - C3
static inline int keytrack(int ns, int pm) { int k = eb70(ns); return k ? ((k + 1) * pm) / 128 : 0; }
// FUN_00013596: frequency velocity sensitivity, b = raw sysex nibble (sens + 7)
static inline int fvs_term(int b, int vel) {
    b &= 0xF;
    int m = (b & 8) ? FVSTAB[1 + (b & 7)] : FVSTAB[7 - (b & 7)];
    int v = (vel - 64) * m; if (!(b & 8)) v = -v;
    return v >> 3;
}
// FUN_00012a32: velocity attenuation, b = raw sysex nibble (sens + 7), 0.375 dB units
static inline int vel_att(int b, int vel) {
    b &= 0xF; int s, t;
    if (b & 8) { s = (b & 7) + 1; t = VELW[vel]; } else { s = 7 - (b & 7); t = VELW[127 - vel]; }
    return std::min(255, (15 - 2 * s) + ((s * 32 * t) >> 8));
}
static inline double db2lin(double db) { return db <= -150 ? 0.0 : fm::db2lin(db); }
// One pole per sample coefficient for the voiced level register, cal::LEVEL_SLEW_MS as a rate.
const double LEVEL_SLEW_K = 1.0 - exp(-1.0 / (cal::LEVEL_SLEW_MS * 0.001 * SR));
// The note-on damp, same shape: what is left of the previous note in a reused channel decays at this
// rate instead of being cut to zero. cal::DAMP_MS.
const double DAMP_K = exp(-1.0 / (cal::DAMP_MS * 0.001 * SR));
// 1024 units per octave, and the word is a 16 bit register that saturates rather than wrapping.
// MEASURED 2026-09-21 off 12_fseqlevel: a ratio operator driven by an Fseq lands past the ceiling at
// every note and the unit answers with one line at 23982 Hz, which is word 32767 to a tenth.
static inline double word_hz(int w) { return 440.0 * fm::exp2((std::clamp(w, 0, 0x7FFF) - 26861) / 1024.0); }

// ------------------------------------------------------------------------------------------ tables (chip side)
extern float g_win[2][8][1025];
extern float g_winDC[2][8];
void init_tables();
// The sine and the level path go through fsvr/fastmath.h, which is where the backend is chosen.
static inline double fsin(double ph) { return fm::sin_turns(ph); }
static inline float fwin(int fam, int s, double x) { double y = x * 1024.0; int i = (int)y; if (i >= 1024) return 0.f; float f = (float)(y - i); const float* w = g_win[fam][s]; return w[i] + (w[i + 1] - w[i]) * f; }
// The per-sample level path: anything at or under -128 dB is silence, the rest is the backend's.
static inline double db2lin_fast(double db) { return db <= -128 ? 0.0 : fm::db2lin(db); }
// INFERRED: chip EG rate 0..63 -> seconds for a full 96 dB traverse. The firmware maps time T to rate (99-T)*0xA4>>8, the
// exact inverse of the DX7's (R*41)>>6, and its DX7 converter uses T = 99 - R, so the chip is assumed to time its EG like the
// DX7 EGS: increment (4 + (q & 3)) << (q >> 2) per 64 samples on a 2^28 = 96 dB scale (Dexed). 6.6 ms at 63, 380 s at 0.
static inline double rate_secs(int q) { q = clampi(q, 0, 63); return pow(2.0, 26 - (q >> 2)) / (4 + (q & 3)) / SR; }
#include "fs1r/chips/vop3_effects.h"

// FUN_0000C36C: the cutoff byte reaches VOP3-1 as a coefficient, 0xC0D + 0xA9 per step capped at
// 0x6000, so it is linear in the coefficient rather than in octaves. Reading that coefficient as a
// one-pole's a = 1 - e^(-2 pi f / fs) puts byte 0 at 755 Hz and byte 127 at 10.6 kHz. The formula is
// the firmware's; the reading is INFERRED and is what a recording would calibrate.
static inline double cut_hz(double c) {
    return cal::CUT_HZ0 * pow(2.0, cal::CUT_OCT * std::clamp(c, cal::CUT_BYTE_MIN, 127.0));
}
// FUN_0000C3D0 sends TWO resonance coefficients per channel, not one. Table 0x374B24 is
// A = 1 - 2^(-raw/16) = 1 - r, and 0x374C24 is B = max(0, 0.5 - 2r^2), quadratic in the same damping and
// zeroed by the firmware for HPF and BEF. Two coefficients around a single shared cutoff coefficient is a
// ladder with feedback and passband compensation, which is also what FUN_0000CA44's per-type input scaler
// (0x40 vs 0x7F) and +-0x4000 tap mix want. Reading A alone as 1/Q is what forced RESO_PER_OCT to 32.
// INFERRED is what the two become: A scales the ladder's feedback and B is available as passband lift.
// The demo settled both scalings, and neither landed on the textbook value. See docs/findings.md, the 2026-09-19 ladder entry.
static inline double reso_r(int r) { return pow(2.0, -clampi(r + 16, 0, 116) / 16.0); }  // fltReso = sysex byte - 16, so r+16 is the byte: A[byte] = 1 - 2^(-byte/16), measured on hardware (fs1r_capture_session3's convhand)
static inline double reso_q(int r) { return cal::RESO_Q0 * pow(2.0, clampi(r + 16, 0, 116) / cal::RESO_PER_OCT); }
// The feedback goes as the CUBE of resonance table A, not as A itself. MEASURED: driving the engine's own
// ladder with noise gives a k-to-peak-lift map, and pushing 15_filter's eleven measured lifts back through
// it asks for k = 0.00, 0.91, 1.70, 2.07, 2.46, 2.70, 3.04, 3.13, 3.20, 3.25, 3.37 over resonance 0 to
// 100. Against table A that is flat in k / A^3 at 3.22 to 3.45 from resonance 20 up, where k / A climbs
// from 2.15 to 3.39 and k / A^2 from 2.72 to 3.42. The cube is empirical, and the CPU side is not in
// question: the 2026-09-19 register retake matched table A to the integer, so this is what the chip makes
// of a number we know it is sent.
static inline double reso_fb(int r) { double a = 1.0 - reso_r(r); return cal::LADDER_K * a * a * a; }
#include "chips/vop3_filter.h"   // SVF, Ladder, VFilter and the filter EG
// ------------------------------------------------------------------------------------------ voice data
struct OpV {
    int keysync, transpose, coarse, fine, notescale, bwbias, form, fixed, skirt, fseqtrk, bw, detune;
    int fegInit, fegAtt, fegAttT, fegDecT;
    int L[4], T[4], hold, tscale, level, bp, ld, rd, lc, rc;
    int fbias, pms, fmsb, amsb, ams, fms, egbias;   // fmsb/amsb = raw sense nibbles (sens+7)
};
struct OpU {
    int transpose, mode, coarse, fine, notescale, bw, bwbias, res, skirt;
    int fegInit, fegAtt, fegAttT, fegDecT, level, lks;
    int L[4], T[4], hold, tscale;
    int fbias, fmsb, amsb, ams, fms, egbias;
};
struct Voice {
    uint8_t raw[608];
    char name[11]; int cat;
    int lfo1wave, lfo1speed, lfo1delay, lfo1sync, pmd, amd, fmd;
    int lfo2wave, lfo2speed, lfo2phase, lfo2sync;
    int fltType, fltReso, fltResoVel, fltCut, fltEgVel, fltLfo1, fltLfo2, fltKsDepth, fltKsPoint, fltInGain, fegDepth;
    int fltL[4], fltT[4], fltAtkVel, fltTscale;
    int noteshift, pegL[5], pegT[4], pegVel, pegRange, pegTscale;
    int fseqV, fseqU, alg, corr[8], fb;
    OpV v[8]; OpU u[8];
};
// DX7 ACED (49 bytes, DX7II/DX7S/TX802 additions). The Data List: "ACED bulk data is not interpreted
// until its following VCED bulk data is received", so it is held here and applied after the conversion.
// Only nine of the 49 bytes carry anything: six per-operator amplitude mod senses, the pitch EG range,
// its velocity switch and its rate scaling.
extern uint8_t g_aced[49];   // defined in firmware/patch.cpp
extern bool g_acedValid;
// DX7 VCED (155 bytes) -> native voice: algorithm k -> k+8, DX op 6..1 -> FS op 3..8


// Defined in firmware/patch.cpp.
void decode_voice(Voice& V);
void init_blank_voice(uint8_t* b);
void init_default_voice(Voice& V);
void apply_aced(uint8_t* out);
void convert_dx7(const uint8_t* v, uint8_t* out);
// ------------------------------------------------------------------------------------------ performance / part / Fseq
struct Fseq {
    bool valid = false; char name[9] = "";
    int nframes = 0, loopStart = 0, loopEnd = 0, loopMode = 0, speedAdj = 64, velTempo = 0, pitchMode = 0, noteAssign = 60, tuning = 63, delay = 0, endStep = 0;
    uint8_t frame[512][50];   // pitch hi/lo, 8 voiced freq hi, 8 lo, 8 voiced level, 8 unvoiced hi, 8 lo, 8 unvoiced level
    void from_bytes(const uint8_t* h, const uint8_t* f, int frames) {
        memcpy(name, h, 8); name[8] = 0;
        loopStart = h[0x10] << 7 | h[0x11]; loopEnd = h[0x12] << 7 | h[0x13]; loopMode = h[0x14] & 1; speedAdj = h[0x15] & 0x7F;
        velTempo = h[0x16] & 7; pitchMode = h[0x17] & 1; noteAssign = h[0x18] & 0x7F; tuning = h[0x19] & 0x7F; delay = h[0x1A];
        nframes = std::min(frames, 128 * ((h[0x1B] & 3) + 1)); endStep = std::min(nframes - 1, h[0x1E] << 7 | h[0x1F]);
        memcpy(frame, f, (size_t)nframes * 50); valid = true;
    }
};
struct Rom;                                  // defined with the loaders below
void rom_voice(const Rom& R, int idx, Voice& V);
int bank_voice_index(int bank, int prog);
bool rom_ready(const Rom* R);
// Performance banks in the EPROM table of 384: the three preset banks, 128 each, in order A, B, C.
// The Data List's performance lists match them entry for entry - "Zap !" is Preset A 1, "Sweepy Voice"
// Preset B 1 and "UprightPiano" Preset C 1 - and the manual's own description of Preset C settles that
// last one on its own: it is the bank for the G50 guitar controller, "the maximum MIDI receive channel
// for these voices is 6, and the pitch bend range is -12 ... +12" (owner's manual page 21), which is
// exactly what entries 256-383 hold and nothing else does. INTERNAL is the user's own battery-backed
// bank, not ROM, so a bank select for it lands on Preset A, which is what the unit ships holding.

struct Part {
    uint8_t p[52]; Voice voice;
    // controller state (MIDI)
    int bend = 0;            // (msb - 64) * 16, the firmware keeps the MSB only
    int expr = 254;          // DAT_010297fa
    // FUN_0000edf0, the per-part controller reset: expression back to 0xFE, bend and every controller
    // source to zero. Reset All Controllers runs it, and so does loading a performance, which walks the
    // four parts calling it (FUN_0000f2f0). MEASURED on the unit the same way: 08_panlevel_2 sweeps
    // expression down to 112 and then sends a performance bulk, and every balance segment after it reads
    // full level on the hardware where the engine stayed 3.01 dB down for the rest of the file.
    void reset_ctl();
    // Controller sources in the firmware's own bit order (FUN_000191C0): KN1-4, MC1, MC2, PB, CAT,
    // PAT, FC, BC, MC3, MW, MC4. The knobs and MIDI controls are stored bipolar as (v - 64) * 2, the
    // physical controllers as the raw value, so the range is -128..127 either way.
    int src[14] = {};
    int held[32]; int nheld = 0; int lastPitch = -1;   // mono handling and portamento start
    int rpnM = 127, rpnL = 127; bool nrpnSel = false;   // RPN / NRPN selection state
    bool sustain = false;
    int rcv() const { return p[1] == 0 ? 0x7F : p[4]; }   // Voice Bank "off" (byte 1) receives nothing, whatever the channel (owner's manual p.63)
};
struct Perf {
    uint8_t c[80]; uint8_t fx[112]; Part part[4];
    char name[13];
};

int bank_perf_index(int lsb, int prog);
void init_perf(Perf& P);
// ------------------------------------------------------------------------------------------ channel state
struct EG {                      // amplitude EG on the chip: hold, 4 segments. INFERRED shape/timing, DX7 style
    int stage = 5; double cur = -200, target = -200, rate = 0, holdLeft = 0; bool rising = false;
    int L[4] = {}, R[4] = {}; int hold = 0; int rs = 0;
    static double lvl_db(int a) { return a >= 63 ? -200.0 : -cal::EG_LEVEL_DB * a; }   // 6-bit attenuation (LEVTAB >> 1), 1.5 dB per step INFERRED
    void start(const int* lv, const int* rt, int h, int rateScale) {
        for (int i = 0; i < 4; i++) { L[i] = lv[i]; R[i] = rt[i]; } hold = h; rs = rateScale; cur = lvl_db(L[3]); stage = 0;
        int hr = egrate(h); if (hr < 0x3F) hr = std::min(hr + 4, 0x3E);   // FUN_00019414: +4 only below 0x3F
        // 0x3F is the firmware's "no hold", the one value it does not offset. Anything else holds for half a
        // traverse at that rate plus a fixed lag, both measured.
        holdLeft = hr < 0x3F ? (rate_secs(hr) * cal::EG_HOLD_FRAC + cal::EG_HOLD_LAG) * SR : 0;   // the key code does not reach the hold: 21_keycode holds 135 ms at every note 12..120 (MEASURED 2026-09-24)
        if (holdLeft < 1) next(1);
    }
    void next(int s) {
        stage = s; if (s > 4) { target = -200; return; }
        target = lvl_db(L[s - 1]);
        double secs = rate_secs(R[s - 1] + rs);
        rising = target > cur;
        // A rising segment does not crawl up from silence: the chip is already at EG_ATTACK_FLOOR one
        // millisecond in, so the climb starts there, or at the target if that is lower still.
        if (rising && cur < cal::EG_ATTACK_FLOOR) cur = std::min(target, cal::EG_ATTACK_FLOOR);
        rate = rising ? 1.0 - exp(-1.0 / (secs * cal::EG_ATTACK_K * SR + 1)) : 96.0 / (secs * SR + 1);
    }
    void release() { if (stage < 4) next(4); }
    inline double tick() {
        if (stage == 0) { if (--holdLeft <= 0) next(1); return cur; }
        if (stage > 4) return cur;
        if (rising) { cur += (cal::EG_OVERSHOOT - cur) * rate; if (cur >= target) { cur = target; if (stage < 3) next(stage + 1); else if (stage == 4) stage = 5; } }
        else { cur -= rate; if (cur <= target) { cur = target; if (stage < 3) next(stage + 1); else if (stage == 4) stage = 5; } }
        return cur;
    }
    bool done() const { return stage == 5 || (stage == 4 && cur <= -120); }
};
struct FreqEG {                  // init -> attack level -> 0. The level curve and the times are the firmware's, read
                                 // back off the voice images (FEGLVL and the EG rate conversion, registers 0x60-0x78).
    double cur = 0, target = 0, katt = 0, kdec = 0; int stage = 2;
    // The sysex byte is not linear in the register: FEGLVL maps 0..100 onto 0..255 with 128 the centre, and
    // half the displayed depth is only a fifth of the register offset. 128 register steps = FEG_SEMIS.
    static double feg_semis(int v) { return (FEGLVL[clampi(v + 50, 0, 100)] - 128) * cal::FEG_SEMIS / 128.0; }
    // A LINEAR RAMP in semitones, and the slope does not depend on the swing. MEASURED 2026-09-25 off
    // 07_modulation_2's four moving frequency EG segments, tracking the carrier rather than its envelope:
    // init 25 and init -25 at attack time 40 slide 946 cents in 220 ms, and attack +-50 at the same time
    // slides 4762 and 4800 cents at the same 4390 cents per second, holding a straight line to 0.3 to 3.8
    // cents rms where the best exponential over the same points leaves 39 to 90. Four times the swing
    // takes four times as long, which is what a ramp does and an approach cannot.
    //
    // The slope is the amplitude EG's own rate ladder read as pitch: a traverse at rate_secs(word) covers
    // cal::FEG_TRAVERSE semitones, which the two long segments put at 23.99 and 23.98 against the 24.0
    // that a 96-unit scale quartered gives. So the chip runs one stepper for both EGs and only the unit of
    // the step differs, 0.375 dB on the amplitude side and a quarter tone here.
    void start(int init, int att, int attT, int decT) {
        cur = feg_semis(init); target = feg_semis(att); stage = (init == 0 && att == 0) ? 2 : 0;
        katt = cal::FEG_TRAVERSE / (rate_secs(egrate(attT)) * SR);
        kdec = cal::FEG_TRAVERSE / (rate_secs(egrate(decT)) * SR);
    }
    inline double tick() {
        if (stage == 0) {
            double d = target - cur;
            if (fabs(d) <= katt) { cur = target; stage = 1; } else cur += d < 0 ? -katt : katt;
        } else if (stage == 1) {
            if (fabs(cur) <= kdec) { cur = 0; stage = 2; } else cur += cur < 0 ? kdec : -kdec;
        }
        return cur;
    }
};
// A grain runs at the carrier frequency it was fired with, latched beside the carrier phase the grain
// already resets. The level is not latched with it, it follows cal::LEVEL_SLEW_MS. MEASURED 2026-09-20
// and 2026-09-21, see op_sample.
struct WinGen { double w = 1.0, c = 0.0, fc = 0.0; bool on = false; };
struct OpState {
    double phase = 0; WinGen g[2]; int nextGen = 0; double fphase = 0; int halfCount = 0;
    EG eg; FreqEG feg;
    EG ueg; FreqEG ufeg; double nphase = 0; double lp[8] = {}; uint32_t rng = 0x12345678;
    double att = 0, attS = 0, fop = 0, fw = 0, wl7 = 1, uatt = 0, nf = 0, na = 1, na2 = 1, nscale = 0, nres = 0, uprev = 0; int bw = 0, ratio = 0;   // refresh_ctl
};
struct Chan {
    bool active = false; int part = 0, note = 0, vel = 0; bool held = false, sustained = false; uint32_t age = 0;
    // Ours: a channel whose carriers and noise are over stops rendering (render_chan). quiet counts the
    // zero samples that decide it, and freezeLeft is the samples until its last envelope is done, once
    // that is arithmetic, so the allocator sees it freed on the same sample it would have been.
    bool silent = false; int quiet = 0; long long freezeLeft = -1;
    // The channel's last output, and the damp left over from the note this channel was playing before.
    // FUN_00023000: note-on damps the channel it is about to take (the mask table at 0x35B260 is one hot
    // per channel) rather than silencing it, so the previous note's waveform fades instead of being cut
    // between two samples. cal::DAMP_MS is how fast.
    double lastL = 0, lastR = 0, dampL = 0, dampR = 0;
    // CPU side, computed at note-on
    int noteP = 60;            // note after all shifts (DAT_0103937a)
    int pitchNote = 0;         // NOTETAB + detune + tune (DAT_01028a84)
    int keyfact = 0;           // KEYFACT >> 2
    int levelOff[8], ulevelOff[8], freqWord[8], ufreqWord[8], frmtWord[8], bwReg[8], ubwReg[8];
    int detW[8] = {};                // detune, key scaled, in pitch word units (register 0x80)
    int egbias[8], uegbias[8]; // image 0x1C0/0x1C8
    int fbW[8] = {}, ufbW[8] = {};   // frequency bias words, DAT_010282bc / DAT_010283bc
    int egL[8][4], egR[8][4], egHold[8], uegL[8][4], uegR[8][4], uegHold[8];
    // pitch EG (software, FUN_00028c8e)
    int pegStage = 5, pegCur = 0, pegTarget = 0, pegRate = 255, velP = 128;
    int pegLvl[5], pegRt[5];
    // portamento (FUN_00029048)
    int portaCur = 0, portaTarget = 0, portaRate = 0; bool portaOn = false;
    // LFO1 (FUN_00028828)
    uint32_t lfoPhase = 0, lfoDelay = 0, lfoFade = 0; int lfoVal = 0; int lfoSH = 0;
    // LFO2 (filter only) and the pan / filter registers (0x22A-0x22F, 0x270)
    uint32_t lfo2Phase = 0; int lfo2Val = 0, lfo2SH = 0;
    int panBase = 63; double panL = 1, panR = 1;
    StepEG feg; VFilter flt; int fltType = 0; bool fltOn = false; double fltGain = 1, fltInGain = 1;
    int vcLvl[8][2] = {}, vcFreq[8][2] = {}, vcBw[8][2] = {};   // voice Formant/FM control offsets, [op][voiced=0]
    // registers refreshed each tick
    int regPitch = 0, regPM = 0, regFM = 0, regAM = 0, regLevel[8], regULevel[8], regC0 = 73;
    int fqWord[8] = {}, fquWord[8] = {}; double partV = 1, partU = 1;
    OpState op[8]; double fbBus = 0, fbPrev = 0;
    double f0 = 0, fbGain = 0; int ctlLeft = 0;                    // control-rate cache, see refresh_ctl
    bool fseqOp[8] = {}, fseqUOp[8] = {};
};

struct Synth {
    Perf perf; Chan ch[NCHAN]; uint32_t clock = 0; double gain = 1.0;   // the analogue volume pot, after the tap
    // FUN_0000f7dc's cursors: the last channel allocated, and the last one each part took. Allocation
    // walks from these rather than from channel 0, which is what fixes the channel a note lands on.
    int nextChan = 0, partChan[4] = {0, 0, 0, 0};
    FxSection fx;
    const Rom* rom = nullptr;
    uint8_t sys[76] = {};                      // system parameters, sysex table 4
    int sysTune() const { return sys[0]; }
    int sysNoteShift() const { return sys[6]; }
    int velCurve() const { return sys[0x0E]; }
    int perfChannel() const { return sys[9]; }             // 0-15, 0x10 = all, 0x7F = off
    int devNumber() const { return sys[0x49]; }
    std::vector<std::vector<uint8_t>> outQ;                // sysex the engine sends back (dump / parameter replies)
    mutable std::mutex mtx;
    // Fseq playback
    Fseq fseq; bool fseqRun = false; double fseqAcc = 0, fseqPeriod = 0.01; int fseqStep = 0, fseqDir = 1; int fseqVel = 100; int fseqPart = -1;
    bool fseqHeld = false, fseqClock = false; double fseqDelay = 0, fseqClockAcc = 0;
    double tickAcc = 0;

    Synth() { init_perf(perf); fx.init(SR); init_system(); }
    void init_system();
    static inline double sendlvl(int v) { return db2lin(-LEVEL_DB * SENDTAB[clampi(v, 0, 127)]); }

    // ---------------------------------------------------------------- per-part derived values
    // ---------------------------------------------------------------- controller sets (FUN_00014DDC)
    // Each of the eight sets sums the sources its 14-bit bitmap enables, then hands the sum and the
    // set's depth to a per-destination handler out of the table at flash 0x3DC04. There are three
    // scalers, and which one a destination uses is exactly the split the owner's manual describes:
    // destinations that "overwrite the edit buffer" edit a part byte, destinations that "directly
    // control the tone generator" return a signed offset.
    //   FUN_00016F9C  clamp((v' * d' * 2) >> 6, -128, 127)             destinations 1-16 and 34-47
    //   FUN_00016F58  clamp(base + ((v' * d' * 2) >> 7), 0, 127)       destinations 18 and 21-33
    //   FUN_00017032  clamp(base + adj((v' * d' * 2) >> 6), 0, 127)    destinations 17, 19, 20
    // with v' = v + (v > 0), d' = d + (d > 0), and adj taking one off a positive result.
    // The part switch at performance common 0x28 + n gates destinations 17-45 only; 1-16 (the insertion
    // parameters and its sends) and 46-47 (the Fseq) are performance-wide and ignore it.
    static int ctrl_bias(int v) { return v + (v > 0 ? 1 : 0); }
    static int scale_f9c(int v, int d) { return clampi((ctrl_bias(v) * ctrl_bias(d) * 2) >> 6, -128, 127); }
    static int scale_f58(int v, int d, int base) { return clampi(base + ((ctrl_bias(v) * ctrl_bias(d) * 2) >> 7), 0, 127); }
    static int scale_f032(int v, int d, int base);
    // The sum for one set: walk the bitmap from bit 0 up, clamping to a signed byte after each source.
    int ctrl_set_sum(int part, int set) const;
    bool ctrl_set_active(int part, int set, int dest) const;
    // A "direct" destination. Every one of these handlers stores a per-part byte rather than adding to
    // one (ctrlDest_36 writes DAT_01029748[part], ctrlDest_37 DAT_01029758, ctrlDest_39 DAT_0102979a),
    // and FUN_00014DDC walks the sets 0 to 7, so two sets on one destination do not compose: the last
    // one evaluated wins, exactly as for the destinations that edit a part byte.
    int ctrl_offset(int part, int dest) const;
    // A destination that edits a part byte: the firmware writes the byte, so the last set that selects
    // it wins rather than the offsets adding up.
    int ctrl_part(int part, int dest, int stored) const;
    // Destination 34 is its own shape: the depth is the bend range in semitones, not a scaler
    // (ctrlDest_34, and the manual: "if Vcn depth is set to +2 the maximum control value is +2
    // semitones"). The summed source drives it like a bend wheel.
    int ctrl_pitch_bias(int part) const;
    // Destination 35 writes one per-part value that event 0x205 then scales by each operator's own EG
    // bias sense: ~((|scaled| + table[|depth|]) * 2), the table being flash 0x3DDC4 (ctrlDest_35).
    int ctrl_eg_bias(int part) const;
    int bend_word(const Part& pt) const;
    void part_levels(int part, int& vAtt, int& uAtt) const;

    // ---------------------------------------------------------------- note on (FUN_00010b48 / FUN_000112c0 / FUN_000125f8)
    void note_on(int part, int note, int vel);
    // per-voice filter (voice common 0x54-0x6E, part 0x07/0x18/0x19/0x1F). Chip side, so INFERRED.
    void start_filter(Chan& C, const Part& pt, int vel);
    // The Fseq retrigger test is the part's own held-note count, not the machine's: FUN_0001228c
    // increments DAT_010288dc[part] and calls the trigger only when that count reaches 1.
    bool anyOther(const Chan& C) const { for (auto& x : ch) if (&x != &C && x.active && x.held && x.part == C.part) return true; return false; }
    // The frame writer (FUN_000197b4) runs whenever the mode byte is 1 or 2 and the part matches; it is
    // not gated on the sequence still advancing, so a sequence that has played out holds its last frame
    // on the registers. The start delay is the one thing that does gate it: FUN_000125f8 clears the
    // track masks while DAT_010291ca is counting, so until the delay expires the operators are the
    // voice's own.
    bool fseq_on(int part) const;

    // Everything the controller matrix feeds an operator, rebuilt on the tick rather than at note-on:
    // FUN_00014DDC re-runs the destination handler every time one of its sources moves, and
    // FUN_0001B334 (voiced) and FUN_0001D056 (unvoiced) then rebuild the whole per-part array, so a
    // held note follows the wheel. The frequency bias reaches the formant and fixed operators only,
    // since the ratio branch of FUN_0001E838 adds nothing, and the unvoiced one only in normal mode.
    void refresh_bias(Chan& C, const Part& pt);

    // FUN_00010dc4: note shifts, note table, part detune, master tune
    void compute_pitch(Chan& C, const Part& pt, int note);
    void retune(Chan& C, int note);
    void voice_changed(int part);              // rebuild the sounding channels' words after a voice edit
    // per-operator note-dependent values: levels (FUN_00012a32/FUN_00012fcc/FUN_00012e2c), frequency words (FUN_0001e838/FUN_000135d8),
    // formant transpose words (FUN_00013bc6), bandwidth registers (FUN_0001ba06), EG rates with part offsets
    void setup_ops(Chan& C, const Part& pt, int vel);
    // FUN_00026c5a: pitch EG per note. Levels PEGLVL (v-128)<<7 * velocity >> range, rates PEGTIME scaled by velocity and key
    void setup_peg(Chan& C, const Part& pt, int vel);
    void note_off(int part, int note);
    void fseq_keys();
    void release(Chan& C);
    void set_sustain(int part, bool on) { Part& pt = perf.part[part]; pt.sustain = on; if (!on) for (auto& c : ch) if (c.active && c.part == part && c.sustained) { c.sustained = false; release(c); } }
    void all_off() { for (auto& c : ch) c.active = false; for (auto& p : perf.part) { p.nheld = 0; } fseqRun = false; fseqHeld = false; }
    void all_release() { for (auto& c : ch) if (c.active && (c.held || c.sustained)) { c.held = c.sustained = false; release(c); } for (auto& p : perf.part) p.nheld = 0; }

    // ---------------------------------------------------------------- Fseq (FUN_0000fffa / FUN_0001a59e)
    // The loop points are the performance's own bytes and nothing else. FUN_0000fffa reads 0x1C-0x1F
    // straight into DAT_010291CE and DAT_010291D0 at every trigger; the Fseq header's own pair is the
    // default the panel copies into a performance when you select the Fseq, not a fallback the player
    // reaches for. This used to fall back when the two were equal, which turned a performance that the
    // unit freezes on its first frame into one that plays. Eight factory performances set them equal on
    // purpose, "Zap !" at 127 and "Replicant" at 94, and so did the first draft of 12_fseqlevel, which
    // is how it was found: the unit recorded digital silence where the engine played.
    void fseq_loop(int& lo, int& hi, int& dir) const;
    void fseq_start(int vel);
    // One frame forward. oneway loops the section between the loop points while a key is held and then
    // plays out the rest; round ping-pongs between them (owner's manual page 34).
    void fseq_step();
    void fseq_tick(double dt);
    // 24 ppqn in; the speed word 0..4 selects 1/4, 1/2, 1/1, 2/1, 4/1 frames per clock. KNOWN: FUN_0001ACB6 takes the
    // clocks since the last frame (0x01029206, bumped per 0xF8 in FUN_0002F760) times 100, shifts >>2, >>1, x1, x2, x4
    // by the word, and divides by 100 for the frames to step.
    void midi_clock();

    // ---------------------------------------------------------------- tick: LFO, PEG, portamento, register refresh
    void lfo_tick(Chan& C, const Part& pt);
    // LFO2: the same stepper as LFO1, but the speed byte is 0..127 straight from the voice and the
    // output only reaches the filter cutoff.
    void lfo2_tick(Chan& C, const Part& pt);
    void peg_tick(Chan& C);
    void porta_tick(Chan& C);
    void refresh_regs(Chan& C, const Part& pt);
    // Voice common 0x40-0x53: five Formant control destinations and five FM control destinations, each
    // [--ddvooo] where dd = off/out/freq/width, v picks the voiced or unvoiced operator and ooo the
    // operator. The paired depth byte scales the part's FORMANT (0x1D) or FM (0x1E) value.
    // INFERRED scaling: a full-depth knob moves the centre frequency about six semitones.
    void voice_ctrl(Chan& C, const Part& pt);
    // registers 0x22A-0x22F: the pan index (part pan, pan scaling, pan LFO, performance pan, the Panpot
    // controller) read through the firmware's own pan tables as a 0.375 dB attenuation per side.
    // FUN_00025bc8 and FUN_0002c36c keep the pan as 0..255 and index the tables at pan >> 1, and the voice
    // image carries 2 * the part byte at +0x2E, so the table index is the part byte itself, not one below it.
    // The firmware keeps the whole sum in the 0..255 domain and halves it once with pan >> 1 (docs/
    // ymp706_registers.md, Pan). pan_index returns the part-pan-plus-scaling in the index domain, so the two
    // terms that follow — the pan LFO and the performance pan — are each summed at 0..255 and land halved.
    void refresh_pan(Chan& C, const Part& pt);
    void refresh_filter(Chan& C, const Part& pt);
    void tick();

    // ---------------------------------------------------------------- MIDI in (Data List sections 2 to 4)
    // One entry point for channel messages so the plugin layer can feed the engine the same way the
    // console does. Sysex goes through midi_sysex(); replies land in outQ.
    int forceChannel = -1;                       // console -c: every part listens on this channel
    double senseTimer = 0;                       // active sensing: mute if 0xFE stops arriving
    int bankMsb = 0x3F, perfBank = -1;
    // FUN_000191C0 fills the source table from the system control numbers, in this order. The four knob
    // slots are disabled when the system's knob receive switch is off.
    enum { SRC_KN1, SRC_KN2, SRC_KN3, SRC_KN4, SRC_MC1, SRC_MC2, SRC_PB, SRC_CAT,
           SRC_PAT, SRC_FC, SRC_BC, SRC_MC3, SRC_MW, SRC_MC4 };
    static const int SRC_RAW = 0x17C0;           // PB, CAT, PAT, FC, BC, MW keep the raw 0..127 value
    int ccSource(int cc) const;
    // FUN_00014AD0: (v - 64) * 2 for the knobs and MIDI controls, raised so full scale reaches 127;
    // the physical controllers are stored as they come.
    void set_source(Part& pt, int slot, int v);
    bool part_listens(int p, int chn) const;
    void load_perf_bank(int lsb, int prog);      // defined with the ROM loaders
    void midi_in(int st, int d1, int d2);
    void control_change(int p, int cc, int v);
    void data_entry(int p, int v);
    void sense_tick(double dt) { if (senseTimer > 0 && (senseTimer -= dt) <= 0) { senseTimer = 0; all_off(); } }

    // ---------------------------------------------------------------- sysex out (dump and parameter replies)
    void push_sysex(std::vector<uint8_t>& m) { if ((int)outQ.size() < tuning::MIDI_OUT_QUEUE) outQ.push_back(m); }
    void push_bulk(int ah, int am, int al, const uint8_t* d, int n);
    void push_param(int ah, int am, int al, int val);
    int read_param(int ah, int am, int al) const;
    void current_perf_bytes(uint8_t* out) const;
    void fseq_bytes(std::vector<uint8_t>& d) const;
    void dump_request(int ah, int am, int al);

    // ---------------------------------------------------------------- chip model (INFERRED beyond the register values)
    // A grain runs at the carrier frequency it was fired with. The chip resets the carrier phase at every
    // grain, and the frequency it then runs at is the one latched with that reset, so a write lands on the
    // grain after it rather than bending the one in flight.
    //
    // `gain` is the operator's level and it is not latched here: it arrives already slewed, one pole at
    // cal::LEVEL_SLEW_MS. MEASURED 2026-09-20 and 2026-09-21 against two recordings that disagree with
    // each other unless the smoothing is a fixed time. Vokodrone's intro is part 4 alone under a 35.6 Hz
    // Fseq, and a level applied per sample cut the grain in flight there, stepping the operator output
    // from 0.149 to 0.033 between two samples: 12.3 dB of click above its own floor at the frame
    // boundary where rgwan's recording has 1.2. Latching the level to the grain fixed that and went too
    // far. 12_fseqlevel drives one formant operator with an Fseq over five octaves and three frame
    // rates, and a latched level is 10.6 dB of octave band error against the unit, worse than the 9.7
    // of no smoothing at all: at a low fundamental the grain is tens of milliseconds and the level
    // cannot wait that long. A 1.6 ms glide is 4.74 dB, and still holds Vokodrone's boundary to 2.7 dB.
    //
    // The unvoiced operator's level is left stepping. Its own segments in the same file sit 15 to 26 dB
    // dark above 640 Hz before any smoothing is applied, which is the noise formant's bandwidth law and
    // not this, so they cannot judge a glide; slewing it there only makes them darker.
    inline double op_sample(OpState& s, const OpV& v, double f0, double fop, int ratio, double pm, double gain);
    // The per-operator frequency and level maths runs every tuning::CTL_DECIMATION samples rather than
    // every sample. Its inputs only move on the 192 Hz register tick and the voice parameters, and doing
    // it per sample (a dozen pow() calls per operator) was the whole CPU bill. The one input that moves
    // every sample, an operator's own pitch EG, stays in the sample loop while it runs: its integral is
    // the phase. The rate itself is ours, not the hardware's, so it lives in fsvr/tuning.h.
    void refresh_ctl(Chan& C);
    inline void render_chan(Chan& C, double& outL, double& outR);
    // Part buses -> insertion / variation / reverb -> master EQ, the XG topology the FS1R uses.
    // ponytail: the performance's individual out (common 0x14) is folded into the main pair; the plugin
    // has one stereo output, so a second pair would be four channels nobody listens to yet.
    void render(float* outL, float* outR, int frames);
};

// Defined in firmware/rom.cpp.
struct Rom { std::vector<uint8_t> d; bool ok = false; };
bool load_rom(Rom& R, const char* path);
void rom_voice(const Rom& R, int idx, Voice& V);
int bank_voice_index(int bank, int prog);
void perf_from_bytes(Synth& S, const Rom* R, const uint8_t* d);
bool rom_perf(Synth& S, const Rom& R, int idx);
bool rom_ready(const Rom* R);
bool rom_fseq(Synth& S, const Rom& R, int n);
bool load_sysex(Synth& S, const Rom* R, const uint8_t* d, size_t len, int pick, int part);
bool param_is_wide(int ah, int am, int al);
bool write_param(Synth& S, int ah, int am, int al, int val);
bool apply_param_change_locked(Synth& S, const uint8_t* d, size_t len);
bool apply_param_change(Synth& S, const uint8_t* d, size_t len);
int selftest(Synth& S);   // fsvr/selftest.cpp
