// fs1r/chips/vop3_effects.h - the FS1R's reverb, variation and insertion blocks plus the master EQ.
//
// The algorithms themselves are VOP3 (YSS236) microcode in the EPROM and nobody has decoded that
// instruction set, so every algorithm below is a model of the behaviour documented in the Effect
// Parameter List (docs/FS1R_DataList_text.txt) and is marked INFERRED.
//
// The parameter *encoding* is not inferred. It was read back out of the 360 preset performances in the
// EPROM and checked against the documented per-type defaults, which match exactly:
//   frequency index  -> 20 * 2^(i/6) Hz        (32 Hz at 4, 1.1 kHz at 35, 9.0 kHz at 53, ...)
//   gain byte        -> v - 64 dB              (52..76 = -12..+12)
//   Q byte           -> v / 10                 (10 = 1.0)
//   delay word       -> 0.1 ms units           (3650 = 365.0 ms)
//   bipolar byte     -> v - 64                 (FB level, ER/Rev, dry/wet, pan)
//   threshold byte   -> v - 127 dB
//   LFO frequency    -> the XG table scaled to 0.000..43.21 Hz (see fx_lfo_hz)
//   reverb time      -> the XG 70-entry table (see fx_revtime_s)
// Send and return levels go through the firmware's own SENDTAB in 0.375 dB steps.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include "../../fsvr/fastmath.h"   // the per-sample sine, 2^x and tanh

// ------------------------------------------------------------------------------------------ parameter decoding
static inline double fx_freq_hz(int i) { return 20.0 * pow(2.0, clampi(i, 0, 60) / 6.0); }
static inline double fx_gain_db(int v) { return (double)(clampi(v, 52, 76) - 64); }
static inline double fx_q(int v) { return std::max(0.5, clampi(v, 1, 120) / 10.0); }
static inline double fx_ms(int w) { return clampi(w, 0, 16383) * 0.1; }
static inline double fx_bip(int v) { return (clampi(v, 0, 127) - 64) / 63.0; }        // -1..+1
static inline double fx_unit(int v) { return clampi(v, 0, 127) / 127.0; }             // 0..1
static inline double fx_thresh_db(int v) { return (double)(clampi(v, 0, 127) - 127); }
static inline double fx_damp(int v) { return clampi(v, 1, 10) / 10.0; }
static inline double fx_init_ms(int i) { return 0.1 + clampi(i, 0, 63) * 3.1565; }    // 0.1..200.0 ms
static inline double fx_revdly_ms(int i) { return 0.1 + clampi(i, 0, 63) * 1.5746; }  // 0.1..99.3 ms
static inline double fx_attack_ms(int i) { return 1.0 + clampi(i, 0, 39); }           // 1..40 ms
static inline double fx_release_ms(int i) { i = clampi(i, 0, 63); return i < 19 ? 10.0 + i * 5.0 : std::min(680.0, 100.0 + (i - 18) * 10.0); }
static inline double fx_ratio(int i) { static const double r[10] = {1, 1.5, 2, 2.5, 3, 4, 5, 7, 10, 20}; return r[clampi(i, 0, 9)]; }

// LFO frequency index -> Hz: the XG table, which is piecewise linear with the slope doubling at each
// break, scaled to the FS1R's documented 0.000..43.21 Hz. Every default in the Effect Parameter List
// lands on an index of this curve (chorus 0.229 at 5, tremolo 5.493 at 84, 2-way rotary 6.958 at 90).
static inline double fx_lfo_hz(int i) {
    static const double brk[7] = {0, 64, 76, 88, 101, 113, 127};
    static const double val[7] = {0.0, 2.69, 3.70, 5.72, 10.10, 18.20, 39.70};
    i = clampi(i, 0, 127);
    int s = 0; while (s < 5 && i > brk[s + 1]) s++;
    return (val[s] + (val[s + 1] - val[s]) * (i - brk[s]) / (brk[s + 1] - brk[s])) * (43.21 / 39.70);
}
// reverb time index 0..69 -> seconds (0.1 s steps to 5 s, then 0.5, then 1.0, then 25 and 30)
static inline double fx_revtime_s(int i) {
    i = clampi(i, 0, 69);
    if (i <= 47) return 0.3 + i * 0.1;
    if (i <= 57) return 5.0 + (i - 47) * 0.5;
    if (i <= 67) return 10.0 + (i - 57) * 1.0;
    return i == 68 ? 25.0 : 30.0;
}

// ------------------------------------------------------------------------------------------ building blocks
struct FxLine {                                  // delay line, fractional read
    std::vector<float> b; int mask = 0, w = 0;
    void init(int n) { int m = 1; while (m < n) m <<= 1; b.assign(m, 0.f); mask = m - 1; w = 0; }
    void clear() { std::fill(b.begin(), b.end(), 0.f); }
    inline void push(double x) { w = (w + 1) & mask; b[w] = (float)x; }
    inline double tap(int d) const { return b[(w - clampi(d, 0, mask)) & mask]; }
    inline double tapf(double d) const {
        if (d < 0) d = 0; if (d > mask - 1) d = mask - 1;
        int i = (int)d; double f = d - i;
        return b[(w - i) & mask] * (1 - f) + b[(w - i - 1) & mask] * f;
    }
};
struct Biq {                                     // RBJ biquad, transposed form 2, two channels of state
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1[2] = {}, z2[2] = {};
    void reset() { z1[0] = z1[1] = z2[0] = z2[1] = 0; }
    inline double run(int c, double x) { double y = b0 * x + z1[c]; z1[c] = b1 * x - a1 * y + z2[c]; z2[c] = b2 * x - a2 * y; return y; }
    void bypass() { b0 = 1; b1 = b2 = a1 = a2 = 0; }
    void set(int kind, double sr, double f, double q, double gdb) {   // 0 lp 1 hp 2 bp 3 peak 4 lowshelf 5 highshelf 6 notch
        f = std::min(std::max(f, 10.0), sr * 0.47);
        double w = 2 * PI * f / sr, cw = cos(w), sw = sin(w), al = sw / (2 * q), A = pow(10.0, gdb / 40.0), a0;
        switch (kind) {
        case 0: a0 = 1 + al; b0 = (1 - cw) / 2 / a0; b1 = (1 - cw) / a0; b2 = b0; a1 = -2 * cw / a0; a2 = (1 - al) / a0; break;
        case 1: a0 = 1 + al; b0 = (1 + cw) / 2 / a0; b1 = -(1 + cw) / a0; b2 = b0; a1 = -2 * cw / a0; a2 = (1 - al) / a0; break;
        case 2: a0 = 1 + al; b0 = al / a0; b1 = 0; b2 = -al / a0; a1 = -2 * cw / a0; a2 = (1 - al) / a0; break;
        case 3: a0 = 1 + al / A; b0 = (1 + al * A) / a0; b1 = -2 * cw / a0; b2 = (1 - al * A) / a0; a1 = b1; a2 = (1 - al / A) / a0; break;
        case 4: { double s2 = 2 * sqrt(A) * al; a0 = (A + 1) + (A - 1) * cw + s2;
                  b0 = A * ((A + 1) - (A - 1) * cw + s2) / a0; b1 = 2 * A * ((A - 1) - (A + 1) * cw) / a0; b2 = A * ((A + 1) - (A - 1) * cw - s2) / a0;
                  a1 = -2 * ((A - 1) + (A + 1) * cw) / a0; a2 = ((A + 1) + (A - 1) * cw - s2) / a0; break; }
        case 5: { double s2 = 2 * sqrt(A) * al; a0 = (A + 1) - (A - 1) * cw + s2;
                  b0 = A * ((A + 1) + (A - 1) * cw + s2) / a0; b1 = -2 * A * ((A - 1) + (A + 1) * cw) / a0; b2 = A * ((A + 1) + (A - 1) * cw - s2) / a0;
                  a1 = 2 * ((A - 1) - (A + 1) * cw) / a0; a2 = ((A + 1) - (A - 1) * cw - s2) / a0; break; }
        default: a0 = 1 + al; b0 = 1 / a0; b1 = -2 * cw / a0; b2 = b0; a1 = b1; a2 = (1 - al) / a0; break;
        }
    }
};
struct FxLfo { double ph = 0, inc = 0; void set(double hz, double sr) { inc = hz / sr; }
    inline void step() { ph += inc; if (ph >= 1) ph -= 1; }
    inline double sine(double off = 0) const { return fm::sin_turns(ph + off); }
    inline double tri(double off = 0) const { double p = ph + off; p -= floor(p); return p < 0.5 ? 4 * p - 1 : 3 - 4 * p; }
};
struct FxEnv { double e = 0, ka = 0, kr = 0;
    void set(double atkMs, double relMs, double sr) { ka = 1 - exp(-1.0 / (atkMs * 0.001 * sr + 1)); kr = 1 - exp(-1.0 / (relMs * 0.001 * sr + 1)); }
    inline double run(double x) { double a = fabs(x); e += (a - e) * (a > e ? ka : kr); return e; }
};
// Schroeder/Freeverb tail: 8 combs + 4 all-passes per channel, comb lengths scaled by density and diffusion.
struct FxTail {
    static const int NC = 8, NA = 4;
    FxLine c[2][NC], a[2][NA]; double lp[2][NC] = {}; int cl[NC] = {}, al[NA] = {};
    double fb = 0.8, damp = 0.3, apg = 0.6;
    void init(double sr) {
        static const int base[NC] = {1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617};
        static const int abase[NA] = {556, 441, 341, 225};
        double k = sr / 44100.0;
        for (int i = 0; i < NC; i++) { cl[i] = (int)(base[i] * k); for (int s = 0; s < 2; s++) c[s][i].init(cl[i] + 64 + (s ? 23 : 0)); }
        for (int i = 0; i < NA; i++) { al[i] = (int)(abase[i] * k); for (int s = 0; s < 2; s++) a[s][i].init(al[i] + 64 + (s ? 17 : 0)); }
    }
    void clear() { for (int s = 0; s < 2; s++) { for (int i = 0; i < NC; i++) { c[s][i].clear(); lp[s][i] = 0; } for (int i = 0; i < NA; i++) a[s][i].clear(); } }
    void set(double sr, double rtSec, double highDamp, int diffusion, int density) {
        double len = cl[NC / 2] / sr;
        fb = std::min(0.995, pow(10.0, -3.0 * len / std::max(0.05, rtSec)));   // -60 dB after rtSec
        damp = 1.0 - std::min(1.0, std::max(0.05, highDamp));
        apg = 0.35 + 0.045 * clampi(diffusion, 0, 10) + 0.02 * clampi(density, 0, 4);
    }
    inline double run(int s, double x) {
        double y = 0;
        for (int i = 0; i < NC; i++) {
            double v = c[s][i].tap(cl[i] + (s ? 23 : 0));
            lp[s][i] += (v - lp[s][i]) * (1 - damp);
            c[s][i].push(x + lp[s][i] * fb);
            y += v;
        }
        y *= 1.0 / NC;
        for (int i = 0; i < NA; i++) {
            double v = a[s][i].tap(al[i] + (s ? 17 : 0));
            double in = y + v * apg;
            a[s][i].push(in);
            y = v - in * apg;
        }
        return y;
    }
};

// ------------------------------------------------------------------------------------------ one effect block
// Effect type numbering follows the Effect Type List: reverb 0..16, variation 0..28, insertion 0..40.
enum FxSlot { FX_REVERB, FX_VARIATION, FX_INSERTION };

struct FxBlock {
    FxSlot slot = FX_REVERB; int type = 0; double sr = 44100;
    int w[16] = {}, b[8] = {};                 // decoded parameter words / bytes for this block
    // state
    FxLine dl[4]; FxLine er; FxTail tail;
    Biq eqLo, eqMid, eqHi, lpf, hpf, wah, extra;
    FxLfo lfo, lfo2; FxEnv env;
    double ap[2][12] = {}; double fbz[2] = {}, z[2] = {}, hold[2] = {}; int srCount = 0; double srHold[2] = {};
    double gain = 1;

    void init(double sampleRate) {
        sr = sampleRate;
        int big = (int)(1.5 * sr);              // 1365 ms of delay, the longest documented time
        for (int i = 0; i < 4; i++) dl[i].init(big);
        er.init((int)(0.25 * sr));
        tail.init(sr);
    }
    void clearState() {
        for (int i = 0; i < 4; i++) dl[i].clear();
        er.clear(); tail.clear();
        eqLo.reset(); eqMid.reset(); eqHi.reset(); lpf.reset(); hpf.reset(); wah.reset(); extra.reset();
        memset(ap, 0, sizeof ap); memset(fbz, 0, sizeof fbz); memset(z, 0, sizeof z); memset(hold, 0, sizeof hold);
        env.e = 0;
    }
    // two-band tone stack used by most variation/insertion types (low shelf + high shelf)
    void setShelves(int fLo, int gLo, int fHi, int gHi) {
        eqLo.set(4, sr, fx_freq_hz(fLo), 0.7, fx_gain_db(gLo));
        eqHi.set(5, sr, fx_freq_hz(fHi), 0.7, fx_gain_db(gHi));
    }
    inline double shelves(int c, double x) { return eqHi.run(c, eqLo.run(c, x)); }

    void configure(FxSlot s, int t, const int* words, const int* bytes) {
        bool changed = (s != slot || t != type);
        slot = s; type = t;
        memcpy(w, words, sizeof w);
        if (bytes) memcpy(b, bytes, sizeof b); else memset(b, 0, sizeof b);
        if (changed) clearState();
        gain = 1;
        eqLo.bypass(); eqMid.bypass(); eqHi.bypass(); lpf.bypass(); hpf.bypass(); wah.bypass(); extra.bypass();
        if (slot == FX_REVERB) configure_reverb();
        else configure_var_ins();
    }

    // -------------------------------------------------------------- reverb slot (17 types)
    void configure_reverb() {
        if (type == 0) return;
        if (type >= 13) {                       // Delay LCR / Delay L,R / Echo / CrossDelay
            if (type == 13 || type == 14) { if (w[7 - 1] || true) {} }
            hpf.bypass();
            return;
        }
        tail.set(sr, fx_revtime_s(w[0]), fx_damp(b[5]), w[1], b[3]);
        if (w[3] > 0) hpf.set(1, sr, fx_freq_hz(w[3]), 0.7, 0); else hpf.bypass();
        if (w[4] < 60) lpf.set(0, sr, fx_freq_hz(w[4]), 0.7, 0); else lpf.bypass();
    }
    // 0 = no effect, 1..12 reverbs, 13 Delay LCR, 14 Delay L,R, 15 Echo, 16 CrossDelay
    inline void run_reverb(double inL, double inR, double& outL, double& outR) {
        if (type == 0) { outL = inL; outR = inR; return; }
        if (type >= 13) { run_delay(inL, inR, outL, outR, w, /*lcr*/ type == 13, /*echo*/ type == 15, /*cross*/ type == 16); return; }
        double mono = (inL + inR) * 0.5;
        mono = hpf.run(0, mono); mono = lpf.run(0, mono);
        er.push(mono);
        int idly = (int)(fx_init_ms(w[2]) * 0.001 * sr);
        double fbl = fx_bip(b[6]);
        double pre = er.tap(idly) + er.tap(idly * 2 + 13) * fbl * 0.5;
        // early reflections: a fixed tap pattern spread by the room dimensions (INFERRED)
        double wid = 0.5 + w[5] * 0.1, hgt = 0.5 + w[6] * 0.1, dep = 0.5 + w[7] * 0.1;
        double scale = (type >= 9 && type <= 12) ? (wid + hgt + dep) / 30.0 + 0.2 : 1.0;
        static const double tapMs[6] = {4.7, 9.3, 14.1, 21.7, 28.3, 37.9};
        double eL = 0, eR = 0;
        for (int i = 0; i < 6; i++) {
            double t = (idly + tapMs[i] * scale * 0.001 * sr);
            double g = 0.7 / (1 + i * 0.55);
            if (i & 1) eR += er.tapf(t) * g; else eL += er.tapf(t * 1.07) * g;
        }
        int rdly = (int)(fx_revdly_ms(b[2]) * 0.001 * sr);
        double rin = er.tapf(idly + rdly) + pre * 0.2;
        double tL = tail.run(0, rin), tR = tail.run(1, rin);
        double bal = fx_bip(b[4]);                       // ER/Rev: -1 = early only, +1 = tail only
        double ge = 0.5 - bal * 0.5, gr = 0.5 + bal * 0.5;
        outL = eL * ge * 1.4 + tL * gr; outR = eR * ge * 1.4 + tR * gr;
    }

    // -------------------------------------------------------------- variation / insertion slots
    // Word index helpers: the two slots use the same order, so one configure covers both. For the
    // variation block word k is sysex 0x68 + 2k (k < 12) or 0x100 + 2(k-12); for the insertion block
    // word k is 0x108 + 2k. The Effect Parameter List indexes are folded into per-type reads below.
    void configure_var_ins() {
        int t = type;
        if (slot == FX_VARIATION) t = var_to_common(t); else t = ins_to_common(t);
        common = t;
        switch (t) {
        case C_CHORUS: case C_CELESTE: case C_FLANGER: case C_SYMPHONIC:
            lfo.set(fx_lfo_hz(w[0]), sr); lfo2.set(fx_lfo_hz(w[0]) * 1.0, sr);
            setShelves(wi(EQ_LO_F), wi(EQ_LO_G), wi(EQ_HI_F), wi(EQ_HI_G)); break;
        case C_PHASER1: case C_PHASER2:
            lfo.set(fx_lfo_hz(w[0]), sr); setShelves(wi(EQ_LO_F), wi(EQ_LO_G), wi(EQ_HI_F), wi(EQ_HI_G)); break;
        case C_ENSDETUNE:
            setShelves(wi(EQ_LO_F), wi(EQ_LO_G), wi(EQ_HI_F), wi(EQ_HI_G)); break;
        case C_ROTARY: case C_ROTARY2:
            lfo.set(fx_lfo_hz(w[0]), sr); lfo2.set(fx_lfo_hz(w[0]) * 0.83, sr);
            setShelves(wi(EQ_LO_F), wi(EQ_LO_G), wi(EQ_HI_F), wi(EQ_HI_G));
            extra.set(0, sr, t == C_ROTARY2 ? fx_freq_hz(w[14]) : 800.0, 0.7, 0); break;
        case C_TREMOLO: case C_AUTOPAN:
            lfo.set(fx_lfo_hz(w[0]), sr); setShelves(wi(EQ_LO_F), wi(EQ_LO_G), wi(EQ_HI_F), wi(EQ_HI_G)); break;
        case C_AUTOWAH:
            lfo.set(fx_lfo_hz(w[0]), sr); setShelves(wi(EQ_LO_F), wi(EQ_LO_G), wi(EQ_HI_F), wi(EQ_HI_G)); break;
        case C_TOUCHWAH:
            env.set(5.0, fx_release_ms(slot == FX_VARIATION ? 40 : w[9]), sr);
            setShelves(wi(EQ_LO_F), wi(EQ_LO_G), wi(EQ_HI_F), wi(EQ_HI_G)); break;
        case C_EQ3:
            eqLo.set(4, sr, fx_freq_hz(w[slot == FX_VARIATION ? 5 : 5]), 0.7, fx_gain_db(w[0]));
            eqMid.set(3, sr, fx_freq_hz(w[1]), fx_q(w[3]), fx_gain_db(w[2]));
            eqHi.set(5, sr, fx_freq_hz(w[6]), 0.7, fx_gain_db(w[4])); break;
        case C_ENHANCER:
            hpf.set(1, sr, fx_freq_hz(w[0]), 0.7, 0); break;
        case C_GATE:
            env.set(fx_attack_ms(w[0]), fx_release_ms(w[1]), sr); break;
        case C_COMP:
            env.set(fx_attack_ms(w[0]), fx_release_ms(w[1]), sr); break;
        case C_DIST: case C_OVERDRIVE: case C_AMPSIM:
            configure_dist(); break;
        case C_DELAY_LCR: case C_DELAY_LR: case C_ECHO: case C_CROSS: case C_KARAOKE:
            configure_delay(); break;
        case C_REVERB:                              // variation 25..28 = Hall/Room/Stage/Plate
            tail.set(sr, fx_revtime_s(w[0]), fx_damp(w[14]), w[1], w[11]);
            if (w[3] > 0) hpf.set(1, sr, fx_freq_hz(w[3]), 0.7, 0);
            if (w[4] < 60) lpf.set(0, sr, fx_freq_hz(w[4]), 0.7, 0); break;
        case C_LOFI:
            if (w[3] < 60) lpf.set(0, sr, fx_freq_hz(w[3]), fx_q(w[5]), 0); break;
        case C_AMBIENCE: case C_ER: case C_GATEREV:
            tail.set(sr, 0.6, 0.8, 5, 2);
            if (slot == FX_INSERTION && w[6] < 60) lpf.set(0, sr, fx_freq_hz(w[6]), 0.7, 0); break;
        case C_PITCH: break;
        case C_WAHDIST:
            env.set(5.0, 170.0, sr); configure_dist(); break;
        case C_COMPDIST:                            // Comp+Dist: attack 0x11E; Cmp+DS+Dly, Cmp+OD+Dly: attack 0x11C (Data List p.30)
            if (type == 25) env.set(fx_attack_ms(w[11]), fx_release_ms(w[12]), sr); else env.set(fx_attack_ms(w[10]), fx_release_ms(w[11]), sr);
            configure_dist(); break;
        default: break;
        }
    }
    void configure_dist() {
        int lf = slot == FX_VARIATION ? 1 : 1, lg = 2, mf = slot == FX_VARIATION ? 6 : 6, mg = 7, mq = 8, lp = 3;
        if (common == C_AMPSIM) { if (w[2] < 60) lpf.set(0, sr, fx_freq_hz(w[2]), 0.7, 0); return; }
        if (common == C_COMPDIST && type != 25) {   // Cmp+DS+Dly / Cmp+OD+Dly: DS Low Gain 0x112, DS Mid Gain 0x114, the frequencies are Comp+Dist's defaults
            eqLo.set(4, sr, 250.0, 0.7, fx_gain_db(w[5])); eqMid.set(3, sr, 4000.0, 1.0, fx_gain_db(w[6])); return;
        }
        eqLo.set(4, sr, fx_freq_hz(w[lf]), 0.7, fx_gain_db(w[lg]));
        eqMid.set(3, sr, fx_freq_hz(w[mf]), fx_q(w[mq]), fx_gain_db(w[mg]));
        if (w[lp] < 60) lpf.set(0, sr, fx_freq_hz(w[lp]), 0.7, 0);
    }
    void configure_delay() {
        int hdIdx = common == C_DELAY_LCR ? 6 : common == C_DELAY_LR ? 5 : 4;
        double hd = fx_damp(w[hdIdx]);
        extra.set(0, sr, 200.0 + hd * 15000.0, 0.7, 0);
        if (common == C_KARAOKE) {
            if (w[2] > 0) hpf.set(1, sr, fx_freq_hz(w[2]), 0.7, 0);
            if (w[3] < 60) lpf.set(0, sr, fx_freq_hz(w[3]), 0.7, 0);
        }
    }

    // common algorithm ids (several sysex types share one algorithm with different defaults)
    enum { C_THRU, C_CHORUS, C_CELESTE, C_FLANGER, C_SYMPHONIC, C_PHASER1, C_PHASER2, C_PITCH, C_ENSDETUNE,
           C_ROTARY, C_ROTARY2, C_TREMOLO, C_AUTOPAN, C_AMBIENCE, C_AUTOWAH, C_TOUCHWAH, C_WAHDIST, C_LOFI,
           C_EQ3, C_ENHANCER, C_GATE, C_COMP, C_COMPDIST, C_DIST, C_OVERDRIVE, C_AMPSIM,
           C_DELAY_LCR, C_DELAY_LR, C_ECHO, C_CROSS, C_KARAOKE, C_REVERB, C_ER, C_GATEREV };
    int common = C_THRU;
    // shared EQ word slots differ between the variation and insertion layouts
    enum { EQ_LO_F, EQ_LO_G, EQ_HI_F, EQ_HI_G };
    int wi(int which) const {
        static const int varIdx[4] = {5, 6, 7, 8};        // 0x72 0x74 0x76 0x78
        static const int insIdx[4] = {5, 6, 7, 8};        // 0x112 0x114 0x116 0x118
        return w[(slot == FX_VARIATION ? varIdx : insIdx)[which]];
    }
    static int var_to_common(int t) {
        static const int m[29] = {C_THRU, C_CHORUS, C_CELESTE, C_FLANGER, C_SYMPHONIC, C_PHASER1, C_PHASER2,
            C_ENSDETUNE, C_ROTARY, C_TREMOLO, C_AUTOPAN, C_AUTOWAH, C_TOUCHWAH, C_EQ3, C_ENHANCER, C_GATE,
            C_COMP, C_DIST, C_OVERDRIVE, C_AMPSIM, C_DELAY_LCR, C_DELAY_LR, C_ECHO, C_CROSS, C_KARAOKE,
            C_REVERB, C_REVERB, C_REVERB, C_REVERB};
        return m[clampi(t, 0, 28)];
    }
    static int ins_to_common(int t) {
        static const int m[41] = {C_THRU, C_CHORUS, C_CELESTE, C_FLANGER, C_SYMPHONIC, C_PHASER1, C_PHASER2,
            C_PITCH, C_ENSDETUNE, C_ROTARY, C_ROTARY2, C_TREMOLO, C_AUTOPAN, C_AMBIENCE, C_WAHDIST, C_WAHDIST,
            C_WAHDIST, C_WAHDIST, C_WAHDIST, C_WAHDIST, C_LOFI, C_EQ3, C_ENHANCER, C_GATE, C_COMP, C_COMPDIST,
            C_COMPDIST, C_COMPDIST, C_DIST, C_DIST, C_OVERDRIVE, C_OVERDRIVE, C_AMPSIM, C_DELAY_LCR, C_DELAY_LR,
            C_ECHO, C_CROSS, C_ER, C_ER, C_GATEREV, C_GATEREV};
        return m[clampi(t, 0, 40)];
    }

    // -------------------------------------------------------------- per-sample processing
    inline void run_delay(double inL, double inR, double& outL, double& outR, const int* p, bool lcr, bool echo, bool cross) {
        double fb = 0;
        int dL, dR, dC = 0, dF = 0;
        if (echo) {
            dL = (int)(fx_ms(p[0]) * 0.001 * sr); dR = (int)(fx_ms(p[2]) * 0.001 * sr);
            double fL = fx_bip(p[1]), fR = fx_bip(p[3]);
            double d2l = fx_ms(p[5]) * 0.001 * sr, d2r = fx_ms(p[6]) * 0.001 * sr, l2 = fx_unit(p[7]);
            double yl = dl[0].tap(dL), yr = dl[1].tap(dR);
            double tl = extra.run(0, yl), tr = extra.run(1, yr);
            dl[0].push(inL + tl * fL); dl[1].push(inR + tr * fR);
            outL = yl + dl[0].tapf(d2l) * l2; outR = yr + dl[1].tapf(d2r) * l2;
            return;
        }
        if (cross) {
            dL = (int)(fx_ms(p[0]) * 0.001 * sr); dR = (int)(fx_ms(p[1]) * 0.001 * sr);
            fb = fx_bip(p[2]);
            double yl = dl[0].tap(dL), yr = dl[1].tap(dR);
            dl[0].push(inL + extra.run(1, yr) * fb);      // L line fed from the R line's output
            dl[1].push(inR + extra.run(0, yl) * fb);
            outL = yl; outR = yr; return;
        }
        dL = (int)(fx_ms(p[0]) * 0.001 * sr); dR = (int)(fx_ms(p[1]) * 0.001 * sr);
        if (lcr) { dC = (int)(fx_ms(p[2]) * 0.001 * sr); dF = (int)(fx_ms(p[3]) * 0.001 * sr); fb = fx_bip(p[4]); }
        else { dF = (int)(fx_ms(p[2]) * 0.001 * sr); dC = (int)(fx_ms(p[3]) * 0.001 * sr); fb = fx_bip(p[4]); }
        double mono = (inL + inR) * 0.5;
        double f = dl[2].tap(dF);
        dl[2].push(mono + extra.run(0, f) * fb);
        outL = dl[2].tap(dL); outR = dl[2].tap(dR);
        if (lcr) { double c = dl[2].tap(dC) * fx_unit(p[5]); outL += c; outR += c; }
        else { double c = dl[2].tap(dC); outL += c * 0.5; outR += c * 0.5; }
    }

    inline void run_var_ins(double inL, double inR, double& outL, double& outR) {
        double L = inL, R = inR;
        switch (common) {
        case C_THRU: break;
        case C_CHORUS: case C_CELESTE: case C_SYMPHONIC: case C_FLANGER: {
            bool sym = common == C_SYMPHONIC;
            double depth = fx_unit(w[1]);
            double off = (sym ? fx_ms(w[2]) : fx_ms(w[3])) * 0.001 * sr;
            double fb = sym ? 0.0 : fx_bip(w[2]);
            if (common == C_FLANGER) { fb = fx_bip(w[2]); off = fx_ms(w[3]) * 0.001 * sr; }
            double span = (common == C_FLANGER ? 0.004 : 0.012) * sr * depth;
            double phR = 0.5;                                        // LFO phase offset between sides
            if (common == C_FLANGER || common == C_PHASER2) phR = (fx_unit(w[12]) - 0.5);
            lfo.step();
            int nv = (common == C_CELESTE || sym) ? 3 : 1;
            double sumL = 0, sumR = 0;
            for (int v = 0; v < nv; v++) {
                double o = v / (double)nv;
                sumL += dl[0].tapf(off + span * (0.5 + 0.5 * lfo.sine(o)));
                sumR += dl[1].tapf(off + span * (0.5 + 0.5 * lfo.sine(o + phR)));
            }
            sumL /= nv; sumR /= nv;
            dl[0].push(inL + sumL * fb); dl[1].push(inR + sumR * fb);
            L = shelves(0, sumL); R = shelves(1, sumR);
            break; }
        case C_PHASER1: case C_PHASER2: {
            int stages = clampi(w[common == C_PHASER1 ? 11 : 11], 3, 12);
            double depth = fx_unit(w[1]), shift = fx_unit(w[2]), fb = fx_bip(w[3]);
            lfo.step();
            for (int s = 0; s < 2; s++) {
                double x = (s ? inR : inL) + fbz[s] * fb;
                double m = 0.5 + 0.5 * lfo.sine(s ? (common == C_PHASER2 ? fx_unit(w[12]) - 0.5 : 0.5) : 0.0);
                double f = 200.0 * fm::exp2(shift * 3.0 + m * depth * 4.0);
                double g = (1 - tan(PI * std::min(f, sr * 0.45) / sr)) / (1 + tan(PI * std::min(f, sr * 0.45) / sr));
                for (int i = 0; i < stages; i++) { double y = g * x + ap[s][i]; ap[s][i] = x - g * y; x = y; }
                fbz[s] = x;
                (s ? R : L) = shelves(s, ((s ? inR : inL) + x) * 0.5);
            }
            break; }
        case C_ENSDETUNE: {
            double cents = (int)w[0] - 64;                            // -50..+50 modelled as a slow flanger
            double dLms = fx_ms(w[1]) * 0.001 * sr, dRms = fx_ms(w[2]) * 0.001 * sr;
            lfo.set(fabs(cents) * 0.02 + 0.05, sr); lfo.step();
            double a = 0.0015 * sr * (cents >= 0 ? 1 : -1);
            L = shelves(0, dl[0].tapf(dLms + a * lfo.sine()));
            R = shelves(1, dl[1].tapf(dRms + a * lfo.sine(0.5)));
            dl[0].push(inL); dl[1].push(inR);
            break; }
        case C_ROTARY: case C_ROTARY2: {
            double depth = fx_unit(w[1]);
            lfo.step(); lfo2.step();
            double mL = lfo.sine(), mR = lfo.sine(0.25);
            double hL = extra.run(0, inL), hR = extra.run(1, inR);     // crossover: lows to the drum
            double rotL = dl[0].tapf(0.002 * sr * (1 + 0.5 * mL)), rotR = dl[1].tapf(0.002 * sr * (1 + 0.5 * mR));
            dl[0].push(inL - hL); dl[1].push(inR - hR);
            L = shelves(0, hL * (1 + depth * 0.5 * lfo2.sine()) + rotL * (1 + depth * mL));
            R = shelves(1, hR * (1 + depth * 0.5 * lfo2.sine(0.5)) + rotR * (1 + depth * mR));
            break; }
        case C_TREMOLO: {
            double am = fx_unit(w[1]), pm = fx_unit(w[2]);
            lfo.step();
            double phR = fx_unit(w[slot == FX_VARIATION ? 13 : 13]) - 0.5;
            double gL = 1 - am * (0.5 + 0.5 * lfo.sine()), gR = 1 - am * (0.5 + 0.5 * lfo.sine(phR));
            double pL = dl[0].tapf(0.002 * sr * (1 + pm * lfo.sine())), pR = dl[1].tapf(0.002 * sr * (1 + pm * lfo.sine(phR)));
            dl[0].push(inL); dl[1].push(inR);
            L = shelves(0, (pm > 0 ? pL : inL) * gL); R = shelves(1, (pm > 0 ? pR : inR) * gR);
            break; }
        case C_AUTOPAN: {
            double lr = fx_unit(w[1]), fr = fx_unit(w[2]); int dir = clampi(w[3], 0, 5);
            lfo.step();
            double m = dir == 5 ? (lfo.ph < 0.5 ? 1 : -1) : dir == 3 ? lfo.tri() : dir == 4 ? -lfo.tri() : lfo.sine();
            if (dir == 1) m = fabs(m); else if (dir == 2) m = -fabs(m);
            // The pan position glides to where the LFO puts it, the way the gate's gain does above. The
            // L/R direction is a square wave and the flip was one sample wide: on B016 Dyno Rose (L/R,
            // 4 Hz) that put a step in the return every 124 ms of a held note. INFERRED: the unit does
            // not click there, how fast it swings is not measured, and the reading is the gate's 0.002.
            z[0] += (m - z[0]) * 0.002; m = z[0];
            double gL = 1 - lr * 0.5 * (1 - m), gR = 1 - lr * 0.5 * (1 + m);
            double fb2 = 1 - fr * 0.3 * (0.5 + 0.5 * lfo.sine(0.25));
            L = shelves(0, inL * gL * fb2); R = shelves(1, inR * gR * fb2);
            break; }
        case C_AUTOWAH: case C_TOUCHWAH: case C_WAHDIST: {
            bool touch = common == C_TOUCHWAH || (common == C_WAHDIST && (type == 16 || type == 17 || type == 18 || type == 19));
            int cf, rs, sens;
            if (common == C_WAHDIST && (type == 18 || type == 19)) { sens = w[13]; cf = w[14]; rs = w[15]; }
            else if (touch && common != C_WAHDIST) { sens = w[0]; cf = w[1]; rs = w[2]; }
            else { sens = w[1]; cf = w[2]; rs = w[3]; }
            double mono = (inL + inR) * 0.5, m;
            if (touch) m = std::min(1.0, env.run(mono) * (1 + fx_unit(sens) * 12.0));
            else { lfo.step(); m = 0.5 + 0.5 * lfo.sine() * fx_unit(sens); }
            double f = 120.0 * fm::exp2(fx_unit(cf) * 4.0 + m * 3.5);
            wah.set(2, sr, f, fx_q(rs), 0);
            L = wah.run(0, inL) * 2.0; R = wah.run(1, inR) * 2.0;
            if (common == C_WAHDIST) {
                int drv = (type == 18 || type == 19) ? w[3] : w[11];
                int out = (type == 18 || type == 19) ? w[4] : w[13];
                L = dist_stage(0, L, drv, 0); R = dist_stage(1, R, drv, 0);
                L *= fx_unit(out) * 2; R *= fx_unit(out) * 2;
                if (type == 18 || type == 19) {                 // Wah+DS+Dly / Wah+OD+Dly
                    int d = (int)(fx_ms(w[0]) * 0.001 * sr); double fb = fx_bip(w[1]), mix = fx_unit(w[2]);
                    double yl = dl[0].tap(d), yr = dl[1].tap(d);
                    dl[0].push(L + yl * fb); dl[1].push(R + yr * fb);
                    L += yl * mix; R += yr * mix;
                }
            } else { L = shelves(0, L); R = shelves(1, R); }
            break; }
        case C_EQ3:
            L = eqHi.run(0, eqMid.run(0, eqLo.run(0, inL)));
            R = eqHi.run(1, eqMid.run(1, eqLo.run(1, inR)));
            break;
        case C_ENHANCER: {
            double drive = 1 + fx_unit(w[1]) * 20, mix = fx_unit(w[2]);
            double hL = hpf.run(0, inL), hR = hpf.run(1, inR);
            L = inL + fm::tanh(hL * drive) * mix; R = inR + fm::tanh(hR * drive) * mix;
            break; }
        case C_GATE: {
            double th = pow(10.0, fx_thresh_db(w[2]) / 20.0), lvl = fx_unit(w[3]) * 2;
            double e = env.run((fabs(inL) + fabs(inR)) * 0.5);
            double g = e > th ? 1.0 : 0.0;
            z[0] += (g - z[0]) * 0.002;
            L = inL * z[0] * lvl; R = inR * z[0] * lvl;
            break; }
        case C_COMP: case C_COMPDIST: {
            int ai, ri, ti, rt, drv = -1, outw = -1;
            if (common == C_COMP) { ai = 0; ri = 1; ti = 2; rt = 3; outw = 4; }
            else if (type == 25) { ai = 11; ri = 12; ti = 13; rt = 14; drv = 0; outw = 4; }
            else { ai = 10; ri = 11; ti = 12; rt = 13; drv = 3; outw = 4; }   // Cmp+DS+Dly / Cmp+OD+Dly (Data List p.30)
            (void)ai; (void)ri;
            double th = pow(10.0, fx_thresh_db(w[ti]) / 20.0), ratio = fx_ratio(w[rt]);
            double e = env.run((fabs(inL) + fabs(inR)) * 0.5) + 1e-9;
            double g = e > th ? pow(th / e, 1.0 - 1.0 / ratio) : 1.0;
            L = inL * g; R = inR * g;
            if (drv >= 0) {
                int edge = type == 25 ? w[10] : type == 26 ? 80 : 104;   // no Edge word on 26/27: Distortion's and Overdrive's defaults
                L = dist_stage(0, L, w[drv], edge); R = dist_stage(1, R, w[drv], edge);
                if (type == 26 || type == 27) {                   // Cmp+DS+Dly / Cmp+OD+Dly
                    int d = (int)(fx_ms(w[0]) * 0.001 * sr); double fb = fx_bip(w[1]), mix = fx_unit(w[2]);
                    double yl = dl[0].tap(d), yr = dl[1].tap(d);
                    dl[0].push(L + yl * fb); dl[1].push(R + yr * fb);
                    L += yl * mix; R += yr * mix;
                }
            }
            if (outw >= 0) { double o = fx_unit(w[outw]) * 2; L *= o; R *= o; }
            break; }
        case C_DIST: case C_OVERDRIVE: case C_AMPSIM: {
            int drv = 0, outw = common == C_AMPSIM ? 3 : 4, edge = common == C_AMPSIM ? 11 : 10;
            if (slot == FX_INSERTION && (type == 29 || type == 31)) {   // Dist+Delay / Odrv+Delay
                drv = 5; outw = 6; edge = -1;
                double dry = (inL + inR) * 0.5;
                double y = dist_stage(0, dry, w[drv], 0);
                int dLs = (int)(fx_ms(w[0]) * 0.001 * sr), dRs = (int)(fx_ms(w[1]) * 0.001 * sr), dF = (int)(fx_ms(w[2]) * 0.001 * sr);
                double fb = fx_bip(w[3]), mix = fx_unit(w[4]);
                dl[2].push(y + dl[2].tap(dF) * fb);
                L = y + dl[2].tap(dLs) * mix; R = y + dl[2].tap(dRs) * mix;
                double o = fx_unit(w[outw]) * 2; L *= o; R *= o;
                break;
            }
            L = dist_stage(0, inL, w[drv], edge >= 0 ? w[edge] : 0);
            R = dist_stage(1, inR, w[drv], edge >= 0 ? w[edge] : 0);
            double o = fx_unit(w[outw]) * 2; L *= o; R *= o;
            break; }
        case C_LOFI: {
            // Word Length (w[1]) is the DPCM differential length, not a sample depth: reading it as one
            // put the factory setting of 1 into a 1-bit quantizer, which rounds every sample to zero.
            // Bit Assign (w[6], 0-6) is the depth. ponytail: mapped onto a plain 4-10 bit quantizer
            // rather than modelled as DPCM, which is what the chip does. INFERRED either way.
            int bits = clampi(w[6], 0, 6) + 4; double q = pow(2.0, bits - 1);
            int div = std::max(1, (int)(sr / std::max(375.0, 48000.0 / std::max(1, (int)(w[0] + 1)))));
            if (++srCount >= div) { srCount = 0; srHold[0] = inL; srHold[1] = inR; }
            L = floor(srHold[0] * q + 0.5) / q; R = floor(srHold[1] * q + 0.5) / q;
            L = lpf.run(0, L); R = lpf.run(1, R);
            double o = pow(10.0, (w[2] - 6) / 20.0); L *= o; R *= o;   // Output Gain -6..+36 dB, not v-64
            break; }
        case C_AMBIENCE: {
            int d = (int)(fx_ms(w[0]) * 0.001 * sr); double s = w[1] ? -1.0 : 1.0;
            er.push((inL + inR) * 0.5);
            L = inL * 0.5 + er.tap(d) * 0.7; R = inR * 0.5 + er.tap(d + 37) * 0.7 * s;
            break; }
        case C_ER: case C_GATEREV: {
            double mono = (inL + inR) * 0.5;
            er.push(mono);
            int idly = (int)(fx_init_ms(w[3]) * 0.001 * sr);
            double size = std::max(0.1, w[1] * 0.1), live = clampi(w[11], 0, 10) / 10.0;
            bool rev = (type == 40);                                  // Revrs Gate: rising tap envelope
            double yL = 0, yR = 0;
            for (int i = 0; i < 12; i++) {
                double t = idly + (1.7 + i * 2.9) * size * 0.001 * sr;
                double g = rev ? (0.15 + i * 0.075) : (1.0 - i * 0.07) * (0.4 + live * 0.6);
                if (common == C_GATEREV && i > 9) g = 0;
                if (i & 1) yR += er.tapf(t) * g * 0.25; else yL += er.tapf(t * 1.05) * g * 0.25;
            }
            L = lpf.run(0, yL); R = lpf.run(1, yR);
            break; }
        case C_REVERB: {                                              // variation Hall/Room/Stage/Plate
            double mono = (inL + inR) * 0.5;
            mono = hpf.run(0, lpf.run(0, mono));
            er.push(mono);
            int idly = (int)(fx_init_ms(w[2]) * 0.001 * sr);
            double tL = tail.run(0, er.tap(idly)), tR = tail.run(1, er.tap(idly + 61));
            double bal = fx_bip(w[12]);
            L = er.tap(idly) * (0.5 - bal * 0.5) + tL * (0.5 + bal * 0.5);
            R = er.tap(idly + 61) * (0.5 - bal * 0.5) + tR * (0.5 + bal * 0.5);
            break; }
        case C_PITCH: {                                               // two detuned taps, crossfaded
            double c1 = ((int)w[2] - 64) + ((int)w[0] - 64) * 100.0, c2 = ((int)w[3] - 64) + ((int)w[0] - 64) * 100.0;
            int base = (int)(fx_ms(w[1]) * 0.001 * sr) + 1;
            dl[0].push((inL + inR) * 0.5);
            z[0] += fm::exp2(c1 / 1200.0) - 1.0; z[1] += fm::exp2(c2 / 1200.0) - 1.0;
            double win = 0.03 * sr;
            if (z[0] > win) z[0] -= win; if (z[0] < 0) z[0] += win;
            if (z[1] > win) z[1] -= win; if (z[1] < 0) z[1] += win;
            double a = dl[0].tapf(base + z[0]), b2 = dl[0].tapf(base + z[1]);
            // Pan1 0x11C, OutLevel1 0x11E, Pan2 0x120, OutLevel2 0x122, so words 10 to 13. Reading them
            // one index high put OutLevel2 beyond the block: the second unit was silent and the first
            // one was panned by its own level.
            double p1 = fx_unit(w[10]), p2 = fx_unit(w[12]);
            double g1 = fx_unit(w[11]), g2 = fx_unit(w[13]);
            L = a * g1 * (1 - p1) + b2 * g2 * (1 - p2); R = a * g1 * p1 + b2 * g2 * p2;
            break; }
        case C_DELAY_LCR: run_delay(inL, inR, L, R, w, true, false, false); L = shelves(0, L); R = shelves(1, R); break;
        case C_DELAY_LR:  run_delay(inL, inR, L, R, w, false, false, false); L = shelves(0, L); R = shelves(1, R); break;
        case C_ECHO:      run_delay(inL, inR, L, R, w, false, true, false); L = shelves(0, L); R = shelves(1, R); break;
        case C_CROSS:     run_delay(inL, inR, L, R, w, false, false, true); L = shelves(0, L); R = shelves(1, R); break;
        case C_KARAOKE: {
            int d = (int)(fx_ms(w[0]) * 0.001 * sr); double fb = fx_bip(w[1]);
            double mono = hpf.run(0, lpf.run(0, (inL + inR) * 0.5));
            double y = dl[2].tap(d);
            dl[2].push(mono + y * fb);
            L = y; R = dl[2].tap(d + 91);
            break; }
        default: break;
        }
        outL = L; outR = R;
    }
    // drive + edge waveshaper shared by the distortion family. Edge 0 = gradual, 127 = hard (Data List).
    inline double dist_stage(int c, double x, int drive, int edge) {
        double g = 1 + fx_unit(drive) * 60.0;
        double e = 0.3 + fx_unit(edge) * 3.0;
        double y = x * g;
        y = y / pow(1.0 + pow(fabs(y), e * 2), 1.0 / (e * 2));        // soft at low edge, clipped at high
        y = eqMid.run(c, eqLo.run(c, y));
        return lpf.run(c, y) * 0.5;
    }

    inline void process(double inL, double inR, double& outL, double& outR) {
        if (slot == FX_REVERB) run_reverb(inL, inR, outL, outR);
        else run_var_ins(inL, inR, outL, outR);
    }
};

// ------------------------------------------------------------------------------------------ the whole effect section
// Performance effect bytes (112): 0x00-0x17 reverb, 0x18-0x37 variation, 0x38-0x57 insertion,
// 0x58-0x63 types / pans / returns / sends, 0x64-0x6F the three-band master EQ.
struct FxSection {
    FxBlock rev, var, ins; Biq eq[3]; double sr = 44100;
    uint8_t cur[112]; bool haveCur = false;
    void init(double sampleRate) {
        sr = sampleRate;
        rev.init(sr); var.init(sr); ins.init(sr);
        memset(cur, 0xFF, sizeof cur); haveCur = false;
    }
    void configure(const uint8_t* fx) {
        if (haveCur && !memcmp(cur, fx, 112)) return;
        memcpy(cur, fx, 112); haveCur = true;
        int w[16], b[8];
        for (int i = 0; i < 8; i++) w[i] = fx[2 * i] << 7 | fx[2 * i + 1];
        for (int i = 8; i < 16; i++) w[i] = 0;
        for (int i = 0; i < 8; i++) b[i] = fx[0x10 + i];
        rev.configure(FX_REVERB, fx[0x58], w, b);
        for (int i = 0; i < 16; i++) w[i] = fx[0x18 + 2 * i] << 7 | fx[0x19 + 2 * i];
        var.configure(FX_VARIATION, fx[0x5B], w, nullptr);
        for (int i = 0; i < 16; i++) w[i] = fx[0x38 + 2 * i] << 7 | fx[0x39 + 2 * i];
        ins.configure(FX_INSERTION, fx[0x5F], w, nullptr);
        eq[0].set(fx[0x67] ? 3 : 4, sr, fx_freq_hz(fx[0x65]), fx_q(fx[0x66]), fx_gain_db(fx[0x64]));
        eq[1].set(3, sr, fx_freq_hz(fx[0x69]), fx_q(fx[0x6A]), fx_gain_db(fx[0x68]));
        eq[2].set(fx[0x6E] ? 3 : 5, sr, fx_freq_hz(fx[0x6C]), fx_q(fx[0x6D]), fx_gain_db(fx[0x6B]));
    }
    inline void master(double& l, double& r) {
        for (int i = 0; i < 3; i++) { l = eq[i].run(0, l); r = eq[i].run(1, r); }
    }
};
