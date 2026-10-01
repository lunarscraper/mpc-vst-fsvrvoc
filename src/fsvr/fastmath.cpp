// fsvr/fastmath.cpp - the tables, the CORDIC rotations and the check that says a backend is libm to
// within its stated error. See fastmath.h for which backend the sample loop is using.
#include "fastmath.h"
#include <cstdio>

namespace fm {

float g_sin[4097];
float g_db2lin[2305];
double g_exp2f[1025];

// CORDIC in Q40: every value is an int64 with forty fraction bits, so a unit is 2^40 and the last of
// the thirty-odd iterations still moves a thousand times the rounding error of the answer.
static const double ONE = 1099511627776.0;    // 2^40
static const int NC = 32;                     // circular iterations, i = 0 .. NC-1
static int64_t ATAN[NC];                      // atan(2^-i)
static int64_t KC;                            // prod cos(atan(2^-i)): the rotation's gain, pre-divided out
// The hyperbolic sequence runs i = 1 .. NH and repeats 4, 13 and 40, which is what makes it converge
// (Walther 1971). Forty is past NH, so two repeats.
static const int NH = 32;
static int SEQ[NH + 2]; static int NSEQ = 0;
static int64_t ATANH[NH + 1];                 // atanh(2^-i)
static int64_t KH;                            // 1 / prod sqrt(1 - 2^-2i) over the sequence: the hyperbolic steps shrink, so the start is scaled up
// The hybrid's residual rotations: the grid point comes off a table already scaled by the partial gain,
// and the rotation runs from step I0 to the last step, so the table sizes are what set I0. A 4096-point
// sine has a residual under 2 pi / 4096 = 1.5e-3 rad, under atan(2^-9); a 1024-point 2^x has one under
// ln 2 / 1024 = 6.8e-4, under atanh(2^-10). Both stop at step 23, where the residual is 1.2e-7, the
// same order as the plain table's interpolation error; every step past that is CPU for nothing.
static const int HC0 = 9, HH0 = 10, HEND = 23;
static int64_t g_sinq[4097], g_cosq[4097];    // sin and cos of the grid, times the partial circular gain
static int64_t g_exp2q[1025];                 // 2^(i / 1024) over the partial hyperbolic gain
static int HSEQ[HEND + 2]; static int NHSEQ = 0;

static bool s_ready = false;
void init() {
    if (s_ready) return;
    s_ready = true;
    const double PI = 3.14159265358979323846;
    for (int i = 0; i <= 4096; i++) g_sin[i] = (float)std::sin(2 * PI * i / 4096.0);
    for (int i = 0; i <= 2304; i++) g_db2lin[i] = (float)std::pow(10.0, (i / 16.0 - 128.0) / 20.0);
    for (int i = 0; i <= 1024; i++) g_exp2f[i] = std::exp2(i / 1024.0);
    double k = 1.0, kp = 1.0;
    for (int i = 0; i < NC; i++) {
        double c = std::cos(std::atan(std::ldexp(1.0, -i)));
        ATAN[i] = (int64_t)std::llround(std::atan(std::ldexp(1.0, -i)) * ONE);
        k *= c; if (i >= HC0 && i <= HEND) kp *= c;
    }
    KC = (int64_t)std::llround(k * ONE);
    for (int i = 0; i <= 4096; i++) { g_sinq[i] = (int64_t)std::llround(std::sin(2 * PI * i / 4096.0) * kp * ONE); g_cosq[i] = (int64_t)std::llround(std::cos(2 * PI * i / 4096.0) * kp * ONE); }
    k = 1.0; kp = 1.0; NSEQ = 0; NHSEQ = 0;
    for (int i = 1; i <= NH; i++) {
        ATANH[i] = (int64_t)std::llround(std::atanh(std::ldexp(1.0, -i)) * ONE);
        double c = std::sqrt(1.0 - std::ldexp(1.0, -2 * i));
        int reps = i == 4 || i == 13 ? 2 : 1;
        for (int r = 0; r < reps; r++) {
            SEQ[NSEQ++] = i; k *= c;
            if (i >= HH0 && i <= HEND) { HSEQ[NHSEQ++] = i; kp *= c; }
        }
    }
    KH = (int64_t)std::llround(ONE / k);
    for (int i = 0; i <= 1024; i++) g_exp2q[i] = (int64_t)std::llround(std::exp2(i / 1024.0) / kp * ONE);
}
static const bool s_loaded = (init(), true);   // so a tool that never calls init_tables() still has tables

// Circular rotation mode: start at (K, 0), rotate through the angle by the arctangent steps, and the y
// that is left is the sine. The angle is folded into [0, pi/2] first, where the rotation converges.
double cordic_sin_turns(double t) {
    t -= std::floor(t);
    double sign = 1.0;
    if (t >= 0.5) { t -= 0.5; sign = -1.0; }
    if (t > 0.25) t = 0.5 - t;
    int64_t z = (int64_t)std::llround(t * 6.283185307179586 * ONE), x = KC, y = 0;
    for (int i = 0; i < NC; i++) {
        int64_t m = z >> 63, xs = x >> i, ys = y >> i;   // m is all ones when z < 0: the step is negated without a branch
        x -= (ys ^ m) - m; y += (xs ^ m) - m; z -= (ATAN[i] ^ m) - m;
    }
    return sign * (double)y / ONE;
}

// Hyperbolic rotation mode: start at (1/K, 0), rotate through r by the hyperbolic arctangent steps, and
// x + y is cosh r + sinh r = e^r. It converges for |r| under 1.118, so 2^x is split into an integer
// exponent and a fraction, whose r = f ln 2 is under 0.7.
double cordic_exp2(double v) {
    if (v < -1000.0) return 0.0;
    if (v > 1000.0) v = 1000.0;
    int n = (int)std::floor(v);
    int64_t z = (int64_t)std::llround((v - n) * 0.6931471805599453 * ONE), x = KH, y = 0;
    for (int s = 0; s < NSEQ; s++) {
        int i = SEQ[s]; int64_t m = z >> 63, xs = x >> i, ys = y >> i;
        x += (ys ^ m) - m; y += (xs ^ m) - m; z -= (ATANH[i] ^ m) - m;
    }
    return scale2n((double)(x + y) / ONE, n);
}

// The hybrid: the table hands over (cos, sin) at the grid point below the phase, and the rotation
// only has the residual to cover. No quadrant folding, since the table spans the whole turn.
double hybrid_sin_turns(double t) {
    double p = (t - std::floor(t)) * 4096.0; int i = (int)p;
    int64_t z = (int64_t)std::llround((p - i) * (6.283185307179586 / 4096.0) * ONE), x = g_cosq[i], y = g_sinq[i];
    for (int k = HC0; k <= HEND; k++) {
        int64_t m = z >> 63, xs = x >> k, ys = y >> k;
        x -= (ys ^ m) - m; y += (xs ^ m) - m; z -= (ATAN[k] ^ m) - m;
    }
    return (double)y / ONE;
}

double hybrid_exp2(double v) {
    if (v < -1000.0) return 0.0;
    if (v > 1000.0) v = 1000.0;
    int n = (int)std::floor(v);
    double f = (v - n) * 1024.0; int i = (int)f;
    int64_t z = (int64_t)std::llround((f - i) * (0.6931471805599453 / 1024.0) * ONE), x = g_exp2q[i], y = 0;
    for (int s = 0; s < NHSEQ; s++) {
        int k = HSEQ[s]; int64_t m = z >> 63, xs = x >> k, ys = y >> k;
        x += (ys ^ m) - m; y += (xs ^ m) - m; z -= (ATANH[k] ^ m) - m;
    }
    return scale2n((double)(x + y) / ONE, n);
}

const char* backend_name() {
    return BACKEND == RAW ? "raw libm" : BACKEND == LUT ? "LUT + interpolation" : BACKEND == CORDIC ? "CORDIC" : "LUT-seeded CORDIC";
}

// The compiled backend against libm over a sweep of each function's working range. The budgets are
// the loosest backend's: the LUT sine's 4096-step lerp (3e-7) and the dB table's 1/16 dB lerp (7e-6
// relative). CORDIC and the hybrid land three to four decades inside them, and the print says where.
int selfcheck() {
    init();
    int fails = 0;
    auto sweep = [&](const char* what, double lo, double hi, int n, double tol, bool rel, double (*ours)(double), double (*ref)(double)) {
        double worst = 0, at = 0;
        for (int i = 0; i <= n; i++) {
            double x = lo + (hi - lo) * i / n, a = ours(x), b = ref(x);
            double e = std::fabs(a - b) / (rel ? std::fabs(b) : 1.0);
            if (e > worst) { worst = e; at = x; }
        }
        bool ok = worst <= tol;
        printf("  %s %-6s vs libm: max %s error %.1e at %.4f (budget %.0e)\n", ok ? "ok  " : "FAIL", what, rel ? "relative" : "absolute", worst, at, tol);
        if (!ok) fails++;
    };
    printf("  fastmath backend: %s\n", backend_name());
    sweep("sin",    -3.0,    3.0, 300001, 5e-7, false, [](double t) { return sin_turns(t); }, [](double t) { return std::sin(6.283185307179586 * t); });
    sweep("exp2",  -60.0,   20.0, 200001, 2e-7, true,  [](double x) { return exp2(x); },      [](double x) { return std::exp2(x); });
    sweep("db2lin", -128.0, 16.0, 200001, 2e-5, true,  [](double d) { return db2lin(d); },    [](double d) { return std::pow(10.0, d / 20.0); });
    sweep("tanh",  -12.0,   12.0, 200001, 2e-6, false, [](double x) { return tanh(x); },      [](double x) { return std::tanh(x); });
    return fails;
}

}  // namespace fm
