/* Offline x86 test of fsvr_vst.cpp with FSVR's engine linked in-process (test.sh builds it with
 * ASan/UBSan): factory banks parsed, performances load and sound, notes stop, macros, RANDOM,
 * program change; vowels measured (F1/F2 peaks against the table), STAB/PAD envelopes, an Fseq
 * moving the spectrum, random vowels, MUTATE/UNDO, SAVE/USER LOAD, mono/glide, chunk restore into
 * a second instance with the vowel live, NaN/denormal-free output.
 * With "bench" as the second argument it only measures the real-time factor (build -O2 for that).
 * Prints PASSED/FAILED; exit code follows. */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <dlfcn.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

struct AEffect;
typedef intptr_t (*audioMasterCallback)(AEffect *, int32_t, int32_t, intptr_t, void *, float);
struct AEffect {
    int32_t magic;
    intptr_t (*dispatcher)(AEffect *, int32_t, int32_t, intptr_t, void *, float);
    void (*process)(AEffect *, float **, float **, int32_t);
    void (*setParameter)(AEffect *, int32_t, float);
    float (*getParameter)(AEffect *, int32_t);
    int32_t numPrograms, numParams, numInputs, numOutputs, flags;
    intptr_t resvd1, resvd2;
    int32_t initialDelay, realQualities, offQualities;
    float ioRatio;
    void *object, *user;
    int32_t uniqueID, version;
    void (*processReplacing)(AEffect *, float **, float **, int32_t);
    void (*processDoubleReplacing)(AEffect *, double **, double **, int32_t);
    char future[56];
};
typedef struct {
    int32_t type, byteSize, deltaFrames, flags, noteLength, noteOffset;
    unsigned char midiData[4];
    char detune, noteOffVelocity, reserved1, reserved2;
} VstMidiEvent;
typedef struct { int32_t numEvents; intptr_t reserved; void *events[64]; } VstEvents;

typedef struct {
    double samplePos, sampleRate, nanoSeconds, ppqPos, tempo, barStartPos, cycleStartPos, cycleEndPos;
    int32_t timeSigNumerator, timeSigDenominator, smpteOffset, smpteFrameRate, samplesToNextClock, flags;
} VstTimeInfo;
static long g_samples = 0;     /* the test's transport: 120 BPM, 4/4, always playing */
static VstTimeInfo g_ti;
static int automated = 0;
static intptr_t master(AEffect *, int32_t op, int32_t, intptr_t, void *, float) {
    if (op == 0) automated++;
    if (op == 7) {
        std::memset(&g_ti, 0, sizeof g_ti);
        g_ti.sampleRate = 44100; g_ti.samplePos = (double)g_samples;
        g_ti.ppqPos = g_samples / 44100.0 * 2.0; g_ti.tempo = 120;
        g_ti.timeSigNumerator = 4; g_ti.timeSigDenominator = 4;
        g_ti.flags = (1 << 1) | (1 << 9) | (1 << 10) | (1 << 13);
        return (intptr_t)&g_ti;
    }
    return 0;
}
static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

static const int BS = 256;
static const float SR = 44100.0f;
static void midi(AEffect *e, std::vector<std::vector<uint8_t>> msgs, int delta = 0);

static void midi(AEffect *e, std::vector<std::vector<uint8_t>> msgs, int delta) {
    static VstMidiEvent ev[64];
    static VstEvents list;
    list.numEvents = 0;
    for (auto &m : msgs) {
        VstMidiEvent &v = ev[list.numEvents];
        std::memset(&v, 0, sizeof v);
        v.type = 1; v.byteSize = sizeof v; v.deltaFrames = delta;
        for (size_t k = 0; k < m.size() && k < 3; k++) v.midiData[k] = m[k];
        list.events[list.numEvents++] = &v;
    }
    e->dispatcher(e, 25, 0, 0, &list, 0);
}

/* renders seconds of audio; returns RMS, flags NaN/Inf/denormals */
static double render(AEffect *e, double seconds, bool *bad = nullptr, std::vector<float> *keep = nullptr) {
    std::vector<float> l(BS), r(BS);
    float *out[2] = {l.data(), r.data()};
    double sum = 0; long n = 0;
    int blocks = (int)(seconds * SR / BS);
    for (int b = 0; b < blocks; b++) {
        e->processReplacing(e, nullptr, out, BS);
        g_samples += BS;
        if (keep) keep->insert(keep->end(), l.begin(), l.end());
        for (int i = 0; i < BS; i++) {
            for (float s : {l[i], r[i]}) {
                if (!std::isfinite(s) || (s != 0 && std::fabs(s) < 1e-37f)) { if (bad) *bad = true; }
                sum += (double)s * s; n++;
            }
        }
    }
    return n ? std::sqrt(sum / n) : 0;
}


/* renders seconds after sending msgs; returns the left channel */
static std::vector<float> capture(AEffect *e, std::vector<std::vector<uint8_t>> msgs, double seconds) {
    midi(e, msgs);
    std::vector<float> l(BS), r(BS), all;
    float *out[2] = {l.data(), r.data()};
    int blocks = (int)(seconds * SR / BS);
    for (int b = 0; b < blocks; b++) {
        e->processReplacing(e, nullptr, out, BS);
        g_samples += BS;
        for (int i = 0; i < BS; i++) all.push_back(0.5f * (l[i] + r[i]));
    }
    return all;
}
static double rms(const std::vector<float> &a, double t0 = 0, double t1 = 1e9) {
    size_t i0 = (size_t)(t0 * SR), i1 = std::min(a.size(), (size_t)(t1 * SR));
    double s = 0;
    for (size_t i = i0; i < i1; i++) s += (double)a[i] * a[i];
    return i1 > i0 ? std::sqrt(s / (i1 - i0)) : 0;
}
/* amplitude of harmonics 1..n-1 of f0 (Goertzel over 0.2-0.7 s); h[0] = 0 */
static std::vector<double> harmonics(const std::vector<float> &a, double f0, int n, double t0 = 0.2, double t1 = 0.7) {
    std::vector<double> h((size_t)n, 0.0);
    size_t i0 = (size_t)(t0 * SR), i1 = std::min(a.size(), (size_t)(t1 * SR));
    for (int k = 1; k < n; k++) {
        double w = 2 * M_PI * f0 * k / SR, c = std::cos(w), s1 = 0, s2 = 0;
        for (size_t i = i0; i < i1; i++) { double s0 = a[i] + 2 * c * s1 - s2; s2 = s1; s1 = s0; }
        h[(size_t)k] = std::sqrt(std::max(0.0, s1 * s1 + s2 * s2 - 2 * c * s1 * s2)) / (i1 - i0);
    }
    return h;
}
/* how far the spectral centroid of 110 Hz harmonics wanders in 100 ms windows (std dev, Hz) */
static double movement(const std::vector<float> &a) {
    std::vector<double> c;
    for (double t = 0.1; t + 0.1 < a.size() / (double)SR; t += 0.1) {
        std::vector<double> h = harmonics(a, 110.0, 40, t, t + 0.1);
        double num = 0, den = 0;
        for (int k = 1; k < 40; k++) { num += 110.0 * k * h[k]; den += h[k]; }
        if (den > 1e-7) c.push_back(num / den);
    }
    if (c.size() < 2) return 0;
    double m = 0, v = 0;
    for (double x : c) m += x;
    m /= c.size();
    for (double x : c) v += (x - m) * (x - m);
    return std::sqrt(v / c.size());
}
/* a test WAV: tones around 300 Hz (0-0.5 s), silence, tones around 3 kHz (1.0-1.5 s), silence */
static void write_wav(const std::string &path, int bits, int channels) {
    const int sr = 44100, n = 2 * sr;
    std::vector<uint8_t> d;
    for (int i = 0; i < n; i++) {
        const double t = (double)i / sr;
        double v = 0;
        if (t < 0.5) for (int k = 0; k < 5; k++) v += std::sin(2 * M_PI * (250 + 40 * k) * t + k);
        else if (t >= 1.0 && t < 1.5) for (int k = 0; k < 5; k++) v += std::sin(2 * M_PI * (2600 + 200 * k) * t + k);
        v *= 0.12;
        for (int c = 0; c < channels; c++) {
            const int32_t x = (int32_t)std::lround(v * (bits == 16 ? 32767.0 : 8388607.0));
            for (int b = 0; b < bits / 8; b++) d.push_back((uint8_t)(x >> (8 * b)));
        }
    }
    auto le = [](std::vector<uint8_t> &o, uint32_t v, int bytes) { for (int b = 0; b < bytes; b++) o.push_back((uint8_t)(v >> (8 * b))); };
    std::vector<uint8_t> f = {'R', 'I', 'F', 'F'};
    le(f, (uint32_t)(36 + d.size()), 4);
    for (char c : std::string("WAVEfmt ")) f.push_back((uint8_t)c);
    le(f, 16, 4); le(f, 1, 2); le(f, (uint32_t)channels, 2); le(f, sr, 4);
    le(f, (uint32_t)(sr * channels * bits / 8), 4); le(f, (uint32_t)(channels * bits / 8), 2); le(f, (uint32_t)bits, 2);
    for (char c : std::string("data")) f.push_back((uint8_t)c);
    le(f, (uint32_t)d.size(), 4);
    f.insert(f.end(), d.begin(), d.end());
    FILE *o = std::fopen(path.c_str(), "wb");
    std::fwrite(f.data(), 1, f.size(), o);
    std::fclose(o);
}
/* spectral centroid of 110 Hz harmonics in [t0, t1] */
static double centroid(const std::vector<float> &a, double t0, double t1) {
    std::vector<double> h = harmonics(a, 110.0, 60, t0, t1);
    double num = 0, den = 0;
    for (int k = 1; k < 60; k++) { num += 110.0 * k * h[k]; den += h[k]; }
    return den > 0 ? num / den : 0;
}
/* the engine state inside the chunk (after the settings text) */
static std::vector<uint8_t> state(AEffect *e) {
    void *chunk = nullptr;
    intptr_t len = e->dispatcher(e, 23, 0, 0, &chunk, 0);
    const uint8_t *b = (const uint8_t *)chunk;
    size_t tl = (size_t)(b[5] << 8 | b[6]);
    return std::vector<uint8_t>(b + 7 + tl, b + len);
}

static int param(AEffect *e, const char *name) {
    char buf[64];
    for (int i = 0; i < e->numParams; i++) {
        buf[0] = 0;
        e->dispatcher(e, 8, i, 0, buf, 0);
        if (!std::strcmp(buf, name)) return i;
    }
    return -1;
}
static std::string display(AEffect *e, int i) {
    char buf[64] = {0};
    e->dispatcher(e, 7, i, 0, buf, 0);
    return buf;
}
static void press(AEffect *e, int i) { e->setParameter(e, i, 1.0f); }
static void choose(AEffect *e, int i, int v, int lo, int hi) { e->setParameter(e, i, (float)(v - lo) / (hi - lo)); }


/* Amplitude of the harmonics of f0 (Goertzel) - the spectral envelope a formant voice draws. */
static std::vector<double> harmonics(const std::vector<float> &x, double f0, double fmax) {
    std::vector<double> a;
    for (int h = 1; h * f0 < fmax; h++) {
        double w = 2 * M_PI * h * f0 / SR, c = 2 * std::cos(w), s1 = 0, s2 = 0;
        for (float v : x) { double s0 = v + c * s1 - s2; s2 = s1; s1 = s0; }
        a.push_back(std::sqrt(std::max(0.0, s1 * s1 + s2 * s2 - c * s1 * s2)) / x.size());
    }
    return a;
}
static double band(const std::vector<double> &a, double f0, double lo, double hi) {
    double e = 0;
    for (size_t h = 0; h < a.size(); h++) { double f = (h + 1) * f0; if (f >= lo && f < hi) e += a[h] * a[h]; }
    return e;
}
static double peak_hz(const std::vector<double> &a, double f0, double lo, double hi) {
    double best = 0, at = 0;
    for (size_t h = 0; h < a.size(); h++) { double f = (h + 1) * f0; if (f >= lo && f < hi && a[h] > best) { best = a[h]; at = f; } }
    return at;
}
static std::vector<uint8_t> chunk_of(AEffect *e) {
    void *c = nullptr;
    intptr_t n = e->dispatcher(e, 23, 0, 0, &c, 0);
    return std::vector<uint8_t>((uint8_t *)c, (uint8_t *)c + n);
}
static void opt(AEffect *e, int i, int v, int nopts) { e->setParameter(e, i, (float)v / (nopts - 1)); }

static const std::vector<std::vector<uint8_t>> CHORD_ON = {{0x90, 48, 100}, {0x90, 55, 100}, {0x90, 60, 100}, {0x90, 64, 100}};
static const std::vector<std::vector<uint8_t>> CHORD_OFF = {{0x80, 48, 0}, {0x80, 55, 0}, {0x80, 60, 0}, {0x80, 64, 0}};

int main(int argc, char **argv) {
    const char *so = argc > 1 ? argv[1] : "./fsvr.so";
    const bool bench = argc > 2 && !std::strcmp(argv[2], "bench");
    void *h = dlopen(so, RTLD_NOW | RTLD_LOCAL);
    if (!h) { std::printf("FAILED: dlopen %s\n", dlerror()); return 1; }
    auto mainf = (AEffect * (*)(audioMasterCallback)) dlsym(h, "VSTPluginMain");
    if (!mainf) { std::printf("FAILED: no VSTPluginMain\n"); return 1; }

    AEffect *e = mainf(master);
    CHECK(e && e->magic == 0x56737450, "magic");
    e->dispatcher(e, 0, 0, 0, nullptr, 0);
    e->dispatcher(e, 10, 0, 0, nullptr, SR);
    e->dispatcher(e, 11, 0, BS, nullptr, 0);
    e->dispatcher(e, 12, 0, 1, nullptr, 0);
    const int pBuf = param(e, "Buffer");
    CHECK(pBuf >= 0, "no Buffer parameter");
    e->setParameter(e, pBuf, 0.0f);   /* offline: render in this thread; section 14 tests the buffer */

    const int pPerf = param(e, "Performance"), pLoad = param(e, "Load"), pRand = param(e, "Random"),
              pCat = param(e, "Category"), pCut = param(e, "Cutoff"), pFm = param(e, "FM"),
              pPanic = param(e, "Panic");
    CHECK(pPerf >= 0 && pLoad >= 0 && pRand >= 0 && pCat >= 0 && pCut >= 0 && pFm >= 0 && pPanic >= 0,
          "parameter names");

    if (bench) {   /* real-time factor of every factory performance, 8 notes held, 2 s each */
        struct R { double load; std::string name; };
        std::vector<R> all;
        for (int pf = 0; pf < 384; pf++) {
            choose(e, pPerf, pf, 0, 383);
            press(e, pLoad);
            midi(e, {{0x90, 36, 110}, {0x90, 43, 100}, {0x90, 48, 100}, {0x90, 55, 100},
                     {0x90, 60, 100}, {0x90, 64, 100}, {0x90, 67, 100}, {0x90, 72, 100}});
            auto t0 = std::chrono::steady_clock::now();
            render(e, 2.0);
            double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            all.push_back({100.0 * s / 2.0, display(e, pPerf)});
            e->setParameter(e, pPanic, 1.0f);
        }
        double sum = 0;
        for (auto &r : all) sum += r.load;
        std::sort(all.begin(), all.end(), [](const R &a, const R &b) { return a.load > b.load; });
        std::printf("bench (this CPU, one core, 8 notes): average %.1f %%, heaviest:\n", sum / all.size());
        for (int k = 0; k < 8; k++) std::printf("  %5.1f %%  %s\n", all[k].load, all[k].name.c_str());
        /* the vowel voice: one note, a 4-note chord, one note with an Fseq */
        const int pMake = param(e, "Make"), pShape = param(e, "Shape"), pFseq = param(e, "Fseq");
        choose(e, pShape, 3, 0, 3);
        press(e, pMake);
        for (int pass = 0; pass < 3; pass++) {
            if (pass == 2) choose(e, pFseq, 1, 0, 90);
            std::vector<std::vector<uint8_t>> on = {{0x90, 48, 100}};
            if (pass == 1) { on.push_back({0x90, 55, 100}); on.push_back({0x90, 60, 100}); on.push_back({0x90, 63, 100}); }
            midi(e, on);
            auto t0 = std::chrono::steady_clock::now();
            render(e, 2.0);
            double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            std::printf("  %5.1f %%  vowel voice, %s\n", 100.0 * s / 2.0, pass == 0 ? "1 note" : pass == 1 ? "4 notes" : "1 note + Fseq");
            e->setParameter(e, pPanic, 1.0f);
        }
        /* + the vocoder, 20 bands, on a WAV */
        std::string dir = so;
        dir = dir.find('/') == std::string::npos ? "." : dir.substr(0, dir.rfind('/'));
        ::mkdir((dir + "/fsvr_vox").c_str(), 0755);
        write_wav(dir + "/fsvr_vox/a_test.wav", 16, 1);
        choose(e, pFseq, 0, 0, 90);
        e->setParameter(e, param(e, "Vocoder"), 1.0f);
        e->setParameter(e, param(e, "Bands"), 1.0f);
        e->setParameter(e, param(e, "Vox Mode"), 0.5f);   /* LOOP */
        midi(e, {{0x90, 48, 100}});
        auto t0 = std::chrono::steady_clock::now();
        render(e, 2.0);
        double sv = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("  %5.1f %%  vowel voice, 1 note + vocoder (20 bands)\n", 100.0 * sv / 2.0);
        e->dispatcher(e, 1, 0, 0, nullptr, 0);
        return 0;
    }

    /* 1. the performance that opened plays */
    std::printf("opened with %s\n", display(e, pPerf).c_str());
    bool bad = false;
    double silent = render(e, 0.5, &bad);
    midi(e, CHORD_ON);
    double on = render(e, 1.0, &bad);
    midi(e, CHORD_OFF);
    render(e, 4.0, &bad);
    double after = render(e, 0.5, &bad);
    std::printf("  rms before %.5f, held %.5f, 4 s after release %.5f\n", silent, on, after);
    CHECK(on > 0.0005, "no sound from the first performance");
    CHECK(after < on * 0.5, "notes do not stop");

    /* 2. a spread of factory performances: load, play, no NaN */
    int loud = 0, tried = 0;
    for (int pf : {1, 17, 64, 127, 128, 200, 255, 256, 300, 383}) {
        choose(e, pPerf, pf, 0, 383);
        press(e, pLoad);
        midi(e, CHORD_ON);
        double r = render(e, 0.6, &bad);
        midi(e, CHORD_OFF);
        render(e, 0.2, &bad);
        e->setParameter(e, pPanic, 1.0f);
        tried++;
        if (r > 0.0005) loud++;
        std::printf("  %-22s rms %.5f\n", display(e, pPerf).c_str(), r);
    }
    CHECK(loud >= tried - 2, "%d of %d performances silent", tried - loud, tried);
    CHECK(!bad, "NaN, Inf or denormals in the output");

    /* 3. RANDOM changes the performance, within a category */
    std::string before = display(e, pPerf);
    press(e, pRand);
    render(e, 0.05);
    CHECK(display(e, pPerf) != before, "RANDOM kept %s", before.c_str());
    choose(e, pCat, 13, 0, 22);   /* SYN FX */
    std::printf("  random (%s): ", display(e, pCat).c_str());
    for (int k = 0; k < 4; k++) { press(e, pRand); std::printf("%s | ", display(e, pPerf).c_str()); }
    std::printf("\n");

    /* 4. macros: move, read back on the next load */
    choose(e, pPerf, 0, 0, 383);
    press(e, pLoad);
    float cut0 = e->getParameter(e, pCut);
    choose(e, pCut, -40, -64, 63);
    press(e, pLoad);
    CHECK(std::fabs(e->getParameter(e, pCut) - cut0) < 1e-4, "cutoff not read back on load");

    /* 5. program change: bank LSB 1, program 5 -> B006 */
    midi(e, {{0xB0, 32, 1}, {0xC0, 5, 0}});
    render(e, 0.05);
    CHECK(display(e, pPerf).rfind("B006", 0) == 0, "program change gave %s", display(e, pPerf).c_str());

    /* 6. vowels: the formant peaks land where the table puts them */
    const int pVowel = param(e, "Vowel"), pVtype = param(e, "Voice"), pShape = param(e, "Shape"),
              pBreath = param(e, "Breath"), pBody = param(e, "Body"), pMake = param(e, "Make"),
              pRndV = param(e, "Rnd Vowel"), pFseq = param(e, "Fseq"), pFseqR = param(e, "Rnd Fseq"),
              pMut = param(e, "Mutate"), pAmt = param(e, "Amount"), pUndo = param(e, "Undo"),
              pSave = param(e, "Save"), pUser = param(e, "User"), pULoad = param(e, "User Load"),
              pMono = param(e, "Mono"), pGlide = param(e, "Glide");
    CHECK(pVowel >= 0 && pVtype >= 0 && pShape >= 0 && pBreath >= 0 && pBody >= 0 && pMake >= 0 && pRndV >= 0 &&
          pFseq >= 0 && pFseqR >= 0 && pMut >= 0 && pAmt >= 0 && pUndo >= 0 && pSave >= 0 && pUser >= 0 &&
          pULoad >= 0 && pMono >= 0 && pGlide >= 0, "v0.2 parameter names");
    choose(e, pShape, 2, 0, 3);      /* LEAD: sustained, for the spectrum */
    choose(e, pBreath, 0, 0, 99);
    choose(e, pBody, 0, 0, 99);
    choose(e, pVtype, 0, 0, 3);      /* MALE */
    const char *vn[8] = {"A", "E", "I", "O", "U", "AE", "ER", "UH"};
    const double F12[8][2] = {{730, 1090}, {530, 1840}, {270, 2290}, {570, 840}, {300, 870}, {660, 1720}, {490, 1350}, {640, 1190}};
    double cent[8];
    for (int vw = 0; vw < 8; vw++) {
        choose(e, pVowel, vw, 0, 7);
        press(e, pMake);
        std::vector<float> a = capture(e, {{0x90, 45, 100}}, 0.8);     /* A2, 110 Hz: dense harmonics */
        midi(e, {{0x80, 45, 0}});
        render(e, 0.4);
        std::vector<double> h = harmonics(a, 110.0, 50);
        auto db = [&](int k) { return 20 * std::log10(h[(size_t)k] + 1e-12); };
        auto peak = [&](double lo, double hi) { int b = 0; for (int k = 1; k < 50; k++) if (110.0 * k >= lo && 110.0 * k <= hi && (!b || h[k] > h[b])) b = k; return b; };
        const int p1 = peak(F12[vw][0] * 0.7 - 60, F12[vw][0] * 1.3 + 60);
        const int p2 = peak(F12[vw][1] * 0.75, F12[vw][1] * 1.35);
        int valley = 0;   /* the weakest harmonic between the two formants */
        for (int k = p1 + 1; k < p2; k++) if (!valley || h[k] < h[valley]) valley = k;
        double num = 0, den = 0;
        for (int k = 1; k < 50; k++) { num += 110.0 * k * h[k]; den += h[k]; }
        cent[vw] = den > 0 ? num / den : 0;
        std::printf("  vowel %-2s  F1 %4.0f Hz %5.1f dB, F2 %4.0f Hz %5.1f dB (table %4.0f/%4.0f), valley %5.1f dB, rms %.4f\n",
                    vn[vw], 110.0 * p1, db(p1), 110.0 * p2, db(p2), F12[vw][0], F12[vw][1], valley ? db(valley) : 0.0, rms(a));
        CHECK(rms(a) > 0.002, "vowel %s silent", vn[vw]);
        if (F12[vw][1] / F12[vw][0] > 1.6)   /* O's F1 and F2 merge at 110 Hz harmonics */
            CHECK(p1 > 0 && p2 > 0 && valley > 0 && db(p2) > db(valley) + 6, "vowel %s: no F2 peak above the valley", vn[vw]);
        CHECK(db(p1) > db(p2) - 3, "vowel %s: F2 louder than F1", vn[vw]);
    }
    CHECK(cent[2] > cent[4] && cent[1] > cent[3], "I not brighter than U, or E not brighter than O");

    /* 7. shapes: STAB dies, PAD holds */
    choose(e, pVowel, 0, 0, 7);
    for (int sh : {0, 3}) {
        choose(e, pShape, sh, 0, 3);    /* the vowel is live: changing the shape rebuilds it */
        std::vector<float> a = capture(e, {{0x90, 57, 110}}, 1.2);
        midi(e, {{0x80, 57, 0}});
        render(e, 1.0);
        double early = rms(a, 0.02, 0.2), late = rms(a, 0.9, 1.2);
        std::printf("  shape %s: rms 0.02-0.2 s %.4f, 0.9-1.2 s %.4f\n", display(e, pShape).c_str(), early, late);
        if (sh == 0) CHECK(late < early * 0.25, "STAB does not decay");
        else CHECK(late > early * 0.5, "PAD does not hold");
    }

    /* 8. Fseq: the spectrum moves while a note holds */
    choose(e, pShape, 2, 0, 3);
    press(e, pMake);
    std::vector<float> st = capture(e, {{0x90, 45, 100}}, 1.5);
    midi(e, {{0x80, 45, 0}});
    render(e, 0.5);
    choose(e, pFseq, 1, 0, 90);
    std::printf("  Fseq %s\n", display(e, pFseq).c_str());
    std::vector<float> fs = capture(e, {{0x90, 45, 100}}, 1.5);
    midi(e, {{0x80, 45, 0}});
    render(e, 0.5);
    double ms = movement(st), mf = movement(fs);
    std::printf("  spectral movement: static %.1f Hz, with Fseq %.1f Hz (rms %.4f)\n", ms, mf, rms(fs));
    CHECK(mf > ms * 3 + 20, "the Fseq does not move the formants");
    CHECK(rms(fs) > 0.001, "silent with an Fseq");
    press(e, pFseqR);
    CHECK(display(e, pFseq) != "OFF", "RND FSEQ left it off");
    choose(e, pFseq, 0, 0, 90);
    CHECK(display(e, pFseq) == "OFF", "Fseq not off");

    /* 9. random vowels, mutate, undo */
    for (int k = 0; k < 6; k++) {
        press(e, pRndV);
        std::vector<float> a = capture(e, {{0x90, 50, 100}}, 0.4);
        midi(e, {{0x80, 50, 0}});
        render(e, 0.3, &bad);
        std::printf("  rnd vowel %-2s rms %.4f\n", display(e, pVowel).c_str(), rms(a));
        CHECK(rms(a) > 0.001, "random vowel silent");
    }
    std::vector<uint8_t> s0 = state(e);
    choose(e, pAmt, 60, 0, 100);
    press(e, pMut);
    std::vector<uint8_t> s1 = state(e);
    CHECK(s1 != s0, "mutate changed nothing");
    {
        std::vector<float> a = capture(e, {{0x90, 50, 100}}, 0.4);
        midi(e, {{0x80, 50, 0}});
        render(e, 0.3, &bad);
        CHECK(rms(a) > 0.0005, "mutant silent");
    }
    press(e, pUndo);
    CHECK(state(e) == s0, "undo did not restore the voice");

    /* 10. save / user load */
    press(e, pSave);
    std::string slot = display(e, pUser);
    std::printf("  saved as %s\n", slot.c_str());
    CHECK(slot.find("--") == std::string::npos, "save: %s", slot.c_str());
    std::vector<uint8_t> saved = state(e);
    press(e, pRndV);
    CHECK(state(e) != saved, "random vowel same as saved");
    press(e, pULoad);
    CHECK(state(e) == saved, "user load did not restore the saved sound");

    /* 11. mono and glide */
    choose(e, pMono, 1, 0, 1);
    choose(e, pGlide, 40, 0, 127);
    choose(e, pPerf, 10, 0, 383);
    press(e, pLoad);
    press(e, pUndo);                 /* back to the mono/glide state */
    CHECK(display(e, pMono) == "MONO" && display(e, pGlide) == "40", "mono %s glide %s", display(e, pMono).c_str(), display(e, pGlide).c_str());

    /* 12. vocoder: the WAV's envelope and spectrum on the FS1R carrier */
    std::string dir = so;
    dir = dir.find('/') == std::string::npos ? "." : dir.substr(0, dir.rfind('/'));
    ::mkdir((dir + "/fsvr_vox").c_str(), 0755);
    write_wav(dir + "/fsvr_vox/a_test.wav", 16, 1);
    write_wav(dir + "/fsvr_vox/b_stereo24.wav", 24, 2);
    const int pVoc = param(e, "Vocoder"), pVFile = param(e, "Vox File"), pVMode = param(e, "Vox Mode"),
              pVMix = param(e, "Voc Mix"), pVHf = param(e, "Sibilance"), pBands = param(e, "Bands"), pVShift = param(e, "Band Shift");
    CHECK(pVoc >= 0 && pVFile >= 0 && pVMode >= 0 && pVMix >= 0 && pVHf >= 0 && pBands >= 0 && pVShift >= 0, "vocoder parameter names");
    choose(e, pMono, 0, 0, 1);
    choose(e, pGlide, 0, 0, 127);
    choose(e, pShape, 2, 0, 3);        /* LEAD vowel with body and breath: a broad carrier */
    choose(e, pBody, 70, 0, 99);
    choose(e, pBreath, 50, 0, 99);
    choose(e, pVowel, 0, 0, 7);
    press(e, pMake);
    choose(e, pVHf, 0, 0, 100);
    choose(e, pVoc, 1, 0, 1);          /* ON reads the folder again */
    choose(e, pVFile, 1, 1, 99);
    std::printf("  vocoder file %s, bands %s\n", display(e, pVFile).c_str(), display(e, pBands).c_str());
    CHECK(display(e, pVFile) == "01 a_test", "vox file shows %s", display(e, pVFile).c_str());
    for (int mode : {0, 1}) {          /* NOTE: once; LOOP: again after 2 s */
        choose(e, pVMode, mode, 0, 2);
        std::vector<float> a = capture(e, {{0x90, 45, 100}}, 2.5);
        midi(e, {{0x80, 45, 0}});
        render(e, 0.6, &bad);
        const double lo = rms(a, 0.05, 0.45), gap = rms(a, 0.6, 0.95), hi = rms(a, 1.05, 1.45), tail = rms(a, 2.05, 2.45);
        const double cl = centroid(a, 0.05, 0.45), ch = centroid(a, 1.05, 1.45);
        std::printf("  %s: rms low %.4f, gap %.4f, high %.4f, after 2 s %.4f; centroid low %.0f Hz, high %.0f Hz\n",
                    display(e, pVMode).c_str(), lo, gap, hi, tail, cl, ch);
        CHECK(lo > 4 * gap && hi > 4 * gap, "%s: the WAV's gaps do not gate the carrier", display(e, pVMode).c_str());
        CHECK(ch > 2.5 * cl, "%s: the WAV's spectrum does not reach the carrier", display(e, pVMode).c_str());
        if (mode == 0) CHECK(tail < 2 * gap + 1e-4, "NOTE: the WAV plays more than once");
        else CHECK(tail > 4 * gap, "LOOP: the WAV does not loop");
    }
    {   /* BAR: no note-on restarts; every bar (2 s at 120 BPM) does */
        choose(e, pVMode, 2, 0, 2);
        midi(e, {{0x90, 45, 100}});
        render(e, 0.1);
        const double t0 = g_samples / (double)SR, bar = std::ceil(t0 / 2.0) * 2.0 - t0;   /* next bar, s from now */
        std::vector<float> a = capture(e, {}, bar + 1.2);
        midi(e, {{0x80, 45, 0}});
        render(e, 0.6, &bad);
        const double lo = rms(a, bar + 0.05, bar + 0.45), gap = rms(a, bar + 0.6, bar + 0.95);
        std::printf("  BAR: next bar after %.2f s, rms low %.4f, gap %.4f\n", bar, lo, gap);
        CHECK(lo > 4 * gap, "BAR: the bar does not restart the WAV");
    }
    {   /* shift and band count keep it working; MIX 0 is the dry FS1R */
        choose(e, pVMode, 0, 0, 2);
        choose(e, pBands, 3, 0, 3);
        choose(e, pVShift, 7, -12, 12);
        std::vector<float> a = capture(e, {{0x90, 45, 100}}, 1.0);
        midi(e, {{0x80, 45, 0}});
        render(e, 0.6, &bad);
        CHECK(rms(a, 0.05, 0.45) > 4 * rms(a, 0.6, 0.95), "20 bands, +7: not gated");
        choose(e, pVMix, 0, 0, 100);
        a = capture(e, {{0x90, 45, 100}}, 1.0);
        midi(e, {{0x80, 45, 0}});
        render(e, 0.6, &bad);
        CHECK(rms(a, 0.6, 0.95) > 0.3 * rms(a, 0.05, 0.45), "MIX 0 still gated");
        choose(e, pVMix, 100, 0, 100);
        choose(e, pVFile, 2, 1, 99);   /* 24-bit stereo reads too */
        a = capture(e, {{0x90, 45, 100}}, 1.0);
        midi(e, {{0x80, 45, 0}});
        render(e, 0.6, &bad);
        std::printf("  %s: rms low %.4f, gap %.4f\n", display(e, pVFile).c_str(), rms(a, 0.05, 0.45), rms(a, 0.6, 0.95));
        CHECK(rms(a, 0.05, 0.45) > 4 * rms(a, 0.6, 0.95), "24-bit stereo WAV not used");
    }

    /* 13. SETUP / CPU: output gain, sleep, voice limit, part limit, effects off */
    {
        const int pOut = param(e, "Output"), pVoices = param(e, "Voices"), pParts = param(e, "Parts"), pFx = param(e, "Effects");
        CHECK(pOut >= 0 && pVoices >= 0 && pParts >= 0 && pFx >= 0, "SETUP parameter names");
        choose(e, pVoc, 0, 0, 1);
        choose(e, pPerf, 5, 0, 383);     /* A006 Hollywood: four parts layered */
        press(e, pLoad);
        std::vector<uint8_t> loaded = state(e);
        std::printf("  output default %s\n", display(e, pOut).c_str());
        double lv[2];
        for (int k = 0; k < 2; k++) {
            choose(e, pOut, k ? 12 : 0, -6, 24);
            std::vector<float> a = capture(e, {{0x90, 57, 100}}, 0.8);
            midi(e, {{0x80, 57, 0}});
            render(e, 0.3, &bad);
            lv[k] = rms(a, 0.2, 0.8);
        }
        std::printf("  output 0 dB rms %.4f, +12 dB rms %.4f\n", lv[0], lv[1]);
        CHECK(lv[1] > 3.0 * lv[0], "OUTPUT +12 dB is not louder");
        /* sleep: silence costs (almost) nothing, a note wakes it at once */
        render(e, 6.0, &bad);
        auto t0 = std::chrono::steady_clock::now();
        double sl = render(e, 3.0, &bad);
        double asleep = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        midi(e, {{0x90, 57, 100}});
        t0 = std::chrono::steady_clock::now();
        double awake_rms = render(e, 1.0, &bad);
        double awake = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        midi(e, {{0x80, 57, 0}});
        render(e, 0.5, &bad);
        std::printf("  sleep: 3 s asleep %.3f s cpu (rms %.6f), 1 s playing %.3f s cpu (rms %.4f)\n", asleep, sl, awake, awake_rms);
        CHECK(sl == 0.0 && asleep < 0.3 * awake * 3, "the plugin does not sleep");
        CHECK(awake_rms > 0.001, "a note does not wake it");
        /* parts: 1 part costs less; ALL brings the performance back exactly */
        double cost[2];
        for (int k = 0; k < 2; k++) {
            choose(e, pParts, k ? 1 : 0, 0, 3);
            midi(e, {{0x90, 48, 100}, {0x90, 55, 100}, {0x90, 60, 100}, {0x90, 63, 100}});
            t0 = std::chrono::steady_clock::now();
            double r = render(e, 2.0, &bad);
            cost[k] = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            midi(e, {{0x80, 48, 0}, {0x80, 55, 0}, {0x80, 60, 0}, {0x80, 63, 0}});
            render(e, 0.5, &bad);
            CHECK(r > 0.001, "silent with PARTS %d", k);
        }
        std::printf("  parts: ALL %.3f s, 1 %.3f s cpu for 2 s of a 4-note chord (%s)\n", cost[0], cost[1], display(e, pParts).c_str());
        CHECK(cost[1] < 0.9 * cost[0], "PARTS 1 does not save CPU");
        choose(e, pParts, 0, 0, 3);
        choose(e, pOut, 12, -6, 24);
        {   /* the state differs from the freshly loaded one only by OUTPUT, which is not engine state */
            std::vector<uint8_t> now = state(e);
            CHECK(now == loaded, "PARTS ALL does not restore the performance");
        }
        /* effects off: still sound, less CPU */
        for (int k = 0; k < 2; k++) {
            choose(e, pFx, k, 0, 1);
            midi(e, {{0x90, 48, 100}, {0x90, 55, 100}});
            t0 = std::chrono::steady_clock::now();
            double r = render(e, 2.0, &bad);
            cost[k] = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            midi(e, {{0x80, 48, 0}, {0x80, 55, 0}});
            render(e, 0.5, &bad);
            CHECK(r > 0.001, "silent with effects %s", k ? "off" : "on");
        }
        std::printf("  effects: on %.3f s, off %.3f s cpu\n", cost[0], cost[1]);
        CHECK(cost[1] < cost[0], "EFFECTS OFF does not save CPU");
        choose(e, pFx, 0, 0, 1);
        /* voices 1: two held notes, only the second sounds */
        choose(e, pVoices, 1, 0, 5);
        choose(e, pShape, 2, 0, 3);
        press(e, pMake);
        std::vector<float> a = capture(e, {{0x90, 45, 100}, {0x90, 46, 100}}, 0.8);
        midi(e, {{0x80, 45, 0}, {0x80, 46, 0}});
        render(e, 0.5, &bad);
        auto series = [&](double f0) { double s = 0; for (int k = 4; k <= 20; k++) s += harmonics(a, f0 * k, 2, 0.3, 0.8)[1]; return s; };
        const double f1 = series(110.0), f2 = series(116.54);
        std::printf("  voices 1: harmonics of A2 %.5f, of A#2 %.5f (rms %.4f)\n", f1, f2, rms(a));
        CHECK(f2 > 3 * f1, "VOICES 1 kept the first note");
        choose(e, pVoices, 0, 0, 5);
        choose(e, pVoc, 1, 0, 1);
        choose(e, pVFile, 2, 1, 99);
    }

    /* 14. BUFFER: the worker thread renders ahead; paced in real time, the sound arrives L+32 later
     * and complete (no dropouts) */
    {
        choose(e, pVoc, 0, 0, 1);
        choose(e, pShape, 2, 0, 3);
        press(e, pMake);
        render(e, 2.0);
        auto paced = [&](double sec, bool note) {
            std::vector<float> l(BS), r(BS), all;
            float *o[2] = {l.data(), r.data()};
            if (note) midi(e, {{0x90, 57, 100}});
            const int blocks = (int)(sec * SR / BS);
            auto next = std::chrono::steady_clock::now();
            for (int b = 0; b < blocks; b++) {
                next += std::chrono::microseconds((long)(1e6 * BS / SR));
                std::this_thread::sleep_until(next);
                e->processReplacing(e, nullptr, o, BS);
                g_samples += BS;
                for (int i = 0; i < BS; i++) all.push_back(l[i]);
            }
            if (note) midi(e, {{0x80, 57, 0}});
            return all;
        };
        auto onset = [](const std::vector<float> &a) { for (size_t i = 0; i < a.size(); i++) if (std::fabs(a[i]) > 1e-3f) return (long)i; return -1L; };
        std::vector<float> d = capture(e, {{0x90, 57, 100}}, 0.6);   /* direct */
        midi(e, {{0x80, 57, 0}});
        render(e, 2.0);
        e->setParameter(e, pBuf, 2.0f / 3.0f);                        /* 12 ms */
        paced(0.3, false);                                            /* let the worker fill */
        std::vector<float> b = paced(0.6, true);
        paced(1.0, false);
        long od = onset(d), ob = onset(b);
        long holes = 0;                                               /* zero runs after the onset */
        for (long i = ob + 1; ob >= 0 && i < (long)b.size() - 64; i++) {
            bool z = true;
            for (int k = 0; k < 32; k++) if (b[(size_t)(i + k)] != 0.0f) { z = false; break; }
            if (z) { holes++; i += 32; }
        }
        std::printf("  buffer: onset direct %ld, buffered %ld samples (want +%d), dropouts %ld, rms %.4f\n",
                    od, ob, 512 + 32, holes, rms(b, 0.1, 0.6));
        /* 544 when the worker keeps up; less while it is still filling the ring (never more) */
        CHECK(ob >= 0 && od >= 0 && ob - od >= 256 && ob - od <= 608, "BUFFER: latency %ld instead of up to 544", ob - od);
        CHECK(holes == 0, "BUFFER: %ld dropouts while paced in real time", holes);
        e->setParameter(e, pBuf, 0.0f);
        render(e, 0.5);
        choose(e, pVoc, 1, 0, 1);
    }

    /* 15. chunk into a second instance, vowel live, vocoder on */
    choose(e, pMono, 0, 0, 1);
    press(e, pRndV);
    void *chunk = nullptr;
    intptr_t len = e->dispatcher(e, 23, 0, 0, &chunk, 0);
    std::vector<uint8_t> copy((uint8_t *)chunk, (uint8_t *)chunk + len);
    AEffect *e2 = mainf(master);
    e2->dispatcher(e2, 10, 0, 0, nullptr, SR);
    e2->dispatcher(e2, 24, 0, (intptr_t)copy.size(), copy.data(), 0);
    CHECK(state(e2) == state(e), "chunk: engine state differs");
    CHECK(display(e2, pVFile) == display(e, pVFile) && display(e2, pVoc) == "ON", "chunk: vocoder %s %s",
          display(e2, pVFile).c_str(), display(e2, pVoc).c_str());
    CHECK(display(e2, pVowel) == display(e, pVowel), "chunk: vowel %s vs %s", display(e2, pVowel).c_str(), display(e, pVowel).c_str());
    choose(e, pBreath, 50, 0, 99);   /* live tweak on both: same result */
    choose(e2, pBreath, 50, 0, 99);
    CHECK(state(e2) == state(e), "chunk: vowel spec not restored");
    {   /* the restored instance sounds like the original */
        midi(e, CHORD_ON); midi(e2, CHORD_ON);
        const double r1 = render(e, 0.5, &bad), r2 = render(e2, 0.5, &bad);
        midi(e, CHORD_OFF);
        std::printf("  restored: rms %.5f, original %.5f\n", r2, r1);
        CHECK(r2 > 0.3 * r1 && r1 > 0, "restored instance silent");
    }
    std::printf("  chunk %ld bytes\n", (long)len);
    e2->dispatcher(e2, 1, 0, 0, nullptr, 0);
    CHECK(!bad, "NaN, Inf or denormals in the output");

    CHECK(automated > 0, "the host was never told about changed values");
    e->dispatcher(e, 1, 0, 0, nullptr, 0);
    dlclose(h);
    std::printf(fails ? "FAILED (%d)\n" : "PASSED\n", fails);
    return fails ? 1 : 0;
}
