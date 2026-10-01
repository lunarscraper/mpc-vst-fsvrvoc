// fsvr/fastmath.h - the sample loop's transcendentals. Ours: the hardware has no opinion.
//
// Everything the per-sample path asks of libm goes through here: a sine by phase in turns, 2^x,
// 10^(dB/20) and tanh. FSVR_MATH picks how they are computed, and the rule is fsvr/tuning.h's: the
// choice trades CPU for nothing else, so a backend that moves a measurement is wrong, not the unit.
//
//   FSVR_MATH=0  RAW     libm. Kept as the reference the others are checked against (fm::selfcheck)
//                        and for A/B renders; not built by default.
//   FSVR_MATH=1  LUT     tables with linear interpolation, the default: sin over 4096 steps, 10^(dB/20)
//                        in 1/16 dB steps, 2^x over 1024 steps of the fraction with the integer part as
//                        an exponent.
//   FSVR_MATH=2  CORDIC  shift-and-add rotations in Q40 fixed point from a unit start: circular for the
//                        sine, hyperbolic for the exponential. No table beyond the arctangents.
//   FSVR_MATH=3  HYBRID  the two together: the table gives the nearest grid point in Q40 and a CORDIC
//                        rotation covers the residual. The residual is under 2^-9, so the rotation
//                        starts at that step and needs fifteen steps instead of thirty, and it lands
//                        within 1.2e-7 of libm where the plain table's interpolation is 3e-7.
//
// MEASURED 2026-09-28, docs/performance.md: on a desktop x86 the table is the fastest of the four by a
// wide margin. Thirty seconds of A020 Vox Morph render in 1.9 s on LUT, 2.8 s on RAW, 6.8 s on HYBRID
// and 9.0 s on CORDIC. A CORDIC step is a serial chain of shifts and adds, and fifteen of them cost
// more than two loads and a multiply however the loop is written, so the hybrid is kept for a target
// without a fast FPU rather than for this one.
//
// The grain windows (g_win in chips/ymp706.cpp) are not here: they are a modelled waveform, not a
// function anyone would compute another way. The control-rate maths (refresh_ctl, the EG rates, the
// effect coefficients) keeps libm on every backend; it runs once per tuning::CTL_DECIMATION samples
// or less and is not where the time goes.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

#ifndef FSVR_MATH
#define FSVR_MATH 1
#endif

namespace fm {

// m * 2^n by writing n into a double's exponent field. ldexp() is a CRT call on MSVC and cost more than
// the pow() it replaced; n stays inside [-1000, 1000] on every path that gets here.
static inline double scale2n(double m, int n) { uint64_t u = (uint64_t)(n + 1023) << 52; double p; std::memcpy(&p, &u, 8); return m * p; }

enum { RAW = 0, LUT = 1, CORDIC = 2, HYBRID = 3 };
static const int BACKEND = FSVR_MATH;
const char* backend_name();
void init();       // fills the tables and the arctangents; idempotent, and run at load anyway
int selfcheck();   // the compiled backend against libm over a sweep; returns the failure count

extern float g_sin[4097];        // LUT: sin(2 pi i / 4096)
extern float g_db2lin[2305];     // LUT: 10^((i / 16 - 128) / 20), -128 .. +16 dB
extern double g_exp2f[1025];     // LUT: 2^(i / 1024)

double cordic_sin_turns(double t);
double cordic_exp2(double x);
double hybrid_sin_turns(double t);
double hybrid_exp2(double x);

#if FSVR_MATH == 0
static inline double sin_turns(double t) { return std::sin(6.283185307179586 * t); }
static inline double exp2(double x) { return std::exp2(x); }
static inline double db2lin(double db) { return std::pow(10.0, db / 20.0); }
static inline double tanh(double x) { return std::tanh(x); }

#elif FSVR_MATH == 1
// The float arithmetic in sin_turns and db2lin is the engine's original table read, kept as it was.
static inline double sin_turns(double t) {
    double x = (t - std::floor(t)) * 4096.0; int i = (int)x; float f = (float)(x - i);
    if (i >= 4096) { i = 4095; f = 1.0f; }   // mpc-vst-fsvr: t just under an integer rounds x to 4096
    return g_sin[i] + (g_sin[i + 1] - g_sin[i]) * f;
}
static inline double exp2(double x) {
    if (x < -1000.0) return 0.0;
    if (x > 1000.0) x = 1000.0;
    int n = (int)std::floor(x); double f = (x - n) * 1024.0; int i = (int)f; double t = f - i;
    if (i >= 1024) { i = 1023; t = 1.0; }   // mpc-vst-fsvr: x just under an integer rounds f to 1024
    return scale2n(g_exp2f[i] + (g_exp2f[i + 1] - g_exp2f[i]) * t, n);
}
static inline double db2lin(double db) {
    if (db < -128.0 || db >= 16.0) return std::pow(10.0, db / 20.0);   // off the table: rare enough to compute
    double x = (db + 128.0) * 16.0; int i = (int)x; float f = (float)(x - i);
    return g_db2lin[i] + (g_db2lin[i + 1] - g_db2lin[i]) * f;
}

#elif FSVR_MATH == 2
static inline double sin_turns(double t) { return cordic_sin_turns(t); }
static inline double exp2(double x) { return cordic_exp2(x); }
static inline double db2lin(double db) { return cordic_exp2(db * 0.16609640474436813); }   // log2(10) / 20

#elif FSVR_MATH == 3
static inline double sin_turns(double t) { return hybrid_sin_turns(t); }
static inline double exp2(double x) { return hybrid_exp2(x); }
static inline double db2lin(double db) { return hybrid_exp2(db * 0.16609640474436813); }

#else
#error "FSVR_MATH must be 0 (RAW), 1 (LUT), 2 (CORDIC) or 3 (HYBRID)"
#endif

#if FSVR_MATH != 0
// tanh from the exponential the backend already has: (e^2x - 1) / (e^2x + 1).
static inline double tanh(double x) {
    if (x > 20.0) return 1.0;
    if (x < -20.0) return -1.0;
    double e = exp2(x * 2.8853900817779268);   // 2 / ln 2
    return (e - 1.0) / (e + 1.0);
}
#endif

}  // namespace fm
