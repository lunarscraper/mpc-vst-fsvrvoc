/* =============================================================================
 * fsvr_vst.cpp - FSVR (Yamaha FS1R reconstruction, github.com/musicastudio/FSVR) as a
 * VST2 instrument for the MPC OS plugin host (Force, MPC Live/One/X/Key), engine IN-PROCESS.
 *
 * Only FSVR's engine is used (../src: fs1rLib, the public header fs1r.h). Its JUCE/Hollow
 * plug-in layer, X11 and the GUI are not: the MPC host never shows a plug-in editor, the screen
 * comes from the skin in build/skin (gen_vst.py, layout.conf).
 *
 *   PERFORM  the 384 factory performances, RANDOM by category, program change
 *   VOWEL    builds a formant voice from a vowel table (A E I O U AE ER UH, male/female/child/
 *            giant), with an EG shape (stab, pluck, lead, pad), breath noise and a sine body;
 *            RND VOWEL varies formants, widths and levels around the vowel
 *   FSEQ     the 90 preset formant sequences on part 1, at the key's pitch, restarted per note
 *   MUTATE   small random moves on part 1's voice, UNDO (16 steps), SAVE/USER LOAD to
 *            <plugin dir>/fsvr_user/FSVR_nnn.syx (FS1R bulk dumps, FSVR desktop opens them too)
 *   VOCODER  16-20 band vocoder: the modulator a WAV from <plugin dir>/fsvr_vox (restarted per note,
 *            looped, or restarted every bar), the carrier the FS1R; band shift, mix, sibilance
 *   SOUND, MOD/FX  the part's offsets as CC/NRPN, mono and glide
 *   SETUP    OUTPUT gain (with a soft knee), VOICES (held notes, oldest released first), PARTS
 *            (play only the first 1-3 parts of a performance), EFFECTS off - the CPU savers; and
 *            the plugin sleeps (renders nothing) after 1.5 s of silence with no note held
 *
 * Every instance owns its own fs1r::Device (the engine has no globals). All MIDI from the track
 * is moved to channel 1 and the device is forced to listen there, so every part plays.
 *
 * Load is measured: every ~5 s the average and peak DSP load go to /tmp/fsvr_vst.log
 * (ssh root@force 'cat /tmp/fsvr_vst.log'). Over ~70 % average means the Force can't keep up.
 *
 * GPL-3.0-or-later, as FSVR itself (see ../LICENSE, ../src/NOTICE.md).
 * ========================================================================== */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <deque>
#include <dirent.h>
#include <memory>
#include <dlfcn.h>
#include <random>
#include <sys/stat.h>
#include <ctime>
#include <mutex>
#include <random>
#include <string>
#include <vector>
#include <dlfcn.h>
#include <sys/stat.h>

#include "fs1r.h"
#include "params.h"
#include "popup.h"    /* mpc-vst-plugins wrapper/popup.h, copied into build/ by build.sh */

/* ---- VST2 ABI (hand-written; no Steinberg SDK) ---------------------------- */
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
typedef struct { int32_t type, byteSize, deltaFrames, flags; char data[16]; } VstEvent;
typedef struct {
    int32_t type, byteSize, deltaFrames, flags, noteLength, noteOffset;
    unsigned char midiData[4];
    char detune, noteOffVelocity, reserved1, reserved2;
} VstMidiEvent;
typedef struct { int32_t numEvents; intptr_t reserved; VstEvent *events[2]; } VstEvents;

enum {
    effOpen = 0, effClose = 1, effGetParamLabel = 6, effGetParamDisplay = 7, effGetParamName = 8,
    effSetSampleRate = 10, effSetBlockSize = 11, effMainsChanged = 12, effGetChunk = 23,
    effSetChunk = 24, effProcessEvents = 25, effCanBeAutomated = 26, effGetPlugCategory = 35,
    effGetEffectName = 45, effGetVendorString = 47, effGetProductString = 48,
    effGetVendorVersion = 49, effCanDo = 51, effGetVstVersion = 58,
};
enum { audioMasterAutomate = 0, audioMasterGetTime = 7, audioMasterUpdateDisplay = 42 };
enum { kVstTransportPlaying = 1 << 1, kVstPpqPosValid = 1 << 9, kVstTempoValid = 1 << 10, kVstTimeSigValid = 1 << 13 };
typedef struct {
    double samplePos, sampleRate, nanoSeconds, ppqPos, tempo, barStartPos, cycleStartPos, cycleEndPos;
    int32_t timeSigNumerator, timeSigDenominator, smpteOffset, smpteFrameRate, samplesToNextClock, flags;
} VstTimeInfo;
enum { effFlagsCanReplacing = 1 << 4, effFlagsProgramChunks = 1 << 5, effFlagsIsSynth = 1 << 8 };

static FILE *g_log;
static std::mutex g_logMutex;
#define LOG(...) do { std::lock_guard<std::mutex> lk_(g_logMutex); \
    if (g_log) { std::fprintf(g_log, __VA_ARGS__); std::fflush(g_log); } } while (0)

/* ---- the factory banks, embedded by banks.S ------------------------------- */
extern "C" const uint8_t fsvr_voices_begin[], fsvr_voices_end[];
extern "C" const uint8_t fsvr_perfs_begin[], fsvr_perfs_end[];
extern "C" const uint8_t fsvr_fseqs_begin[], fsvr_fseqs_end[];

struct Item {
    std::string name;
    int category = 0;
    std::vector<uint8_t> syx;   /* the message(s) that load it; a DX voice: ACED + VCED */
    bool native = true;         /* false: a DX7 voice (VCED), which takes the part it is given */
};
struct Factory { std::vector<Item> perfs, voices, fseqs; };

static std::string text(const uint8_t *p, int n) {
    std::string s;
    for (int i = 0; i < n; i++) s += (p[i] >= 32 && p[i] < 127) ? (char)p[i] : ' ';
    while (!s.empty() && s.back() == ' ') s.pop_back();
    return s;
}

/* A minimal version of FSVR's plugin/library.cpp parseSyx(): native FS1R bulks and DX7 ACED+VCED. */
static void parse_bank(const uint8_t *b, size_t n, Factory &f) {
    std::vector<uint8_t> aced;
    for (size_t i = 0; i < n;) {
        if (b[i] != 0xF0) { i++; continue; }
        size_t j = i + 1;
        while (j < n && b[j] != 0xF7) j++;
        if (j >= n) break;
        const uint8_t *m = b + i;
        const size_t len = j - i + 1;
        i = j + 1;
        if (len < 8 || m[1] != 0x43 || (m[2] & 0xF0) != 0) continue;
        if (m[3] == 0x5E && len >= 11) {
            const int ah = m[6];
            const uint8_t *d = m + 9;
            const size_t dl = len - 11;
            Item it;
            it.syx.assign(m, m + len);
            if ((ah == 0x10 || ah == 0x11) && dl >= 400) {
                it.name = text(d, 12); it.category = d[14]; f.perfs.push_back(it);
            } else if (((ah >= 0x40 && ah <= 0x43) || ah == 0x51) && dl >= 608) {
                it.name = text(d, 10); it.category = d[0x0E]; f.voices.push_back(it);
            } else if ((ah == 0x60 || ah == 0x61 || ah == 0x70) && dl >= 32) {
                it.name = text(d, 8); f.fseqs.push_back(it);
            }
        } else if (m[3] == 0x05 && m[4] == 0x00 && m[5] == 0x31) {
            aced.assign(m, m + len);
        } else if (m[3] == 0x00 && m[4] == 0x01 && m[5] == 0x1B && len >= 6 + 155 + 2) {
            Item it;
            it.native = false;
            it.name = text(m + 6 + 145, 10);
            it.syx = aced;
            it.syx.insert(it.syx.end(), m, m + len);
            f.voices.push_back(it);
            aced.clear();
        }
    }
}

static const Factory &factory() {
    static Factory f;
    static std::once_flag once;
    std::call_once(once, [] {
        parse_bank(fsvr_voices_begin, (size_t)(fsvr_voices_end - fsvr_voices_begin), f);
        parse_bank(fsvr_perfs_begin, (size_t)(fsvr_perfs_end - fsvr_perfs_begin), f);
        parse_bank(fsvr_fseqs_begin, (size_t)(fsvr_fseqs_end - fsvr_fseqs_begin), f);
        LOG("[fsvr_vst] factory: %zu performances, %zu voices, %zu Fseqs\n",
            f.perfs.size(), f.voices.size(), f.fseqs.size());
    });
    return f;
}

/* The factory voice number (EPROM order: PrA, PrB native, PrC..PrK DX) of a part's bank and
 * program bytes, -1 for off and Int. Same as FSVR plugin.cpp factoryVoice(). */
static int factory_voice(int bank, int program) {
    program = program < 0 ? 0 : program > 127 ? 127 : program;
    if (bank == 2 || bank == 3) return (bank - 2) * 128 + program;
    if (bank >= 4 && bank <= 12) return 256 + (bank - 4) * 128 + program;
    return -1;
}

/* A native bulk readdressed (a voice bound for part p), checksum redone. */
static std::vector<uint8_t> readdress(const std::vector<uint8_t> &syx, int ah, int am, int al) {
    std::vector<uint8_t> m = syx;
    if (m.size() < 11) return m;
    m[6] = (uint8_t)ah; m[7] = (uint8_t)am; m[8] = (uint8_t)al;
    int sum = 0;
    for (size_t k = 4; k + 2 < m.size(); k++) sum += m[k];
    m[m.size() - 2] = (uint8_t)(-sum & 0x7F);
    return m;
}

/* ---- sysex helpers ---------------------------------------------------------------- */
/* One FS1R native bulk: F0 43 00 5E bc bc ah am al <data> cs F7. */
static std::vector<uint8_t> bulk(int ah, int am, int al, const uint8_t *data, size_t size) {
    std::vector<uint8_t> m = {0xF0, 0x43, 0x00, 0x5E, (uint8_t)(size >> 7 & 0x7F), (uint8_t)(size & 0x7F),
                              (uint8_t)ah, (uint8_t)am, (uint8_t)al};
    m.insert(m.end(), data, data + size);
    int sum = 0;
    for (size_t k = 4; k < m.size(); k++) sum += m[k];
    m.push_back((uint8_t)(-sum & 0x7F));
    m.push_back(0xF7);
    return m;
}
/* ---- the engine's own init voice (src/fs1r/firmware/patch.cpp, compiled into this .so) ---- */
void init_blank_voice(uint8_t *b);

/* ---- the macros: where each lives in a part (perf bulk 192 + 52*part + offset) and how it
 * is sent. Offsets are the part parameter addresses the engine's own CC/NRPN handlers write
 * (src/fs1r/firmware/midi.cpp control_change()/data_entry()). ------------------------------ */
struct Macro { const char *key; int part_off; int cc; int nrpn; bool centred; };
static const Macro MACROS[] = {
    {"formant",   0x1D, 80, -1,   true },   /* Formant knob, system default CC 80 */
    {"fm",        0x1E, 81, -1,   true },   /* FM knob, system default CC 81 */
    {"cutoff",    0x18, 74, -1,   true },
    {"reso",      0x19, 71, -1,   true },
    {"attack",    0x1A, 73, -1,   true },
    {"decay",     0x1B, -1, 0x64, true },
    {"release",   0x1C, 72, -1,   true },
    {"lfo_speed", 0x15, -1, 0x08, true },
    {"lfo_pitch", 0x16, -1, 0x09, true },
    {"rev_send",  0x13, 91, -1,   false},
    {"var_send",  0x12, 93, -1,   false},
    {"volume",    0x0B,  7, -1,   false},
};
static const int NMACROS = (int)(sizeof MACROS / sizeof MACROS[0]);

static int param_index(const char *key) {
    for (int i = 0; i < NPARAMS; i++) if (!std::strcmp(PARAMS[i].key, key)) return i;
    return -1;
}

/* ---- vowels: F1..F5 in Hz (Peterson & Barney averages for F1-F3; F4/F5 typical values) --- */
static const char *const VOWEL_NAMES[8] = {"A", "E", "I", "O", "U", "AE", "ER", "UH"};
static const double VOWELS[3][8][5] = {
    { /* male */
        {730, 1090, 2440, 3400, 4100}, {530, 1840, 2480, 3500, 4200}, {270, 2290, 3010, 3500, 4300},
        {570, 840, 2410, 3300, 4000},  {300, 870, 2240, 3300, 4000},  {660, 1720, 2410, 3400, 4100},
        {490, 1350, 1690, 3300, 4000}, {640, 1190, 2390, 3300, 4000} },
    { /* female */
        {850, 1220, 2810, 3800, 4600}, {610, 2330, 2990, 3900, 4700}, {310, 2790, 3310, 3900, 4800},
        {590, 920, 2710, 3700, 4500},  {370, 950, 2670, 3700, 4500},  {860, 2050, 2850, 3800, 4600},
        {500, 1640, 1960, 3700, 4500}, {760, 1400, 2780, 3700, 4500} },
    { /* child */
        {1030, 1370, 3170, 4300, 5000}, {690, 2610, 3570, 4400, 5200}, {370, 3200, 3730, 4500, 5300},
        {680, 1060, 3180, 4200, 5000},  {430, 1170, 3260, 4200, 5000}, {1010, 2320, 3320, 4300, 5100},
        {560, 1820, 2160, 4200, 5000},  {850, 1590, 3360, 4200, 5000} },
};
static const int F_LEVEL[5] = {96, 99, 99, 94, 88};
static const int F_BW[5] = {44, 48, 52, 54, 56};
/* EG shapes, FS1R times (0 = fastest) and levels: STAB, PLUCK, LEAD, PAD */
static const int SHAPE_L[4][4] = {{99, 55, 0, 0}, {99, 35, 0, 0}, {99, 95, 90, 0}, {90, 99, 97, 0}};
static const int SHAPE_T[4][4] = {{0, 48, 58, 38}, {0, 38, 48, 32}, {4, 50, 70, 42}, {55, 75, 80, 60}};

struct VowelSpec {
    int shift[5] = {0, 0, 0, 0, 0};   /* random offsets: 1/128 octave steps */
    int bwd[5] = {0, 0, 0, 0, 0};
    int lvd[5] = {0, 0, 0, 0, 0};
    int skirt[5] = {0, 0, 0, 0, 0};
};

/* Hz -> the voiced op's coarse/fine (form "formant"/fixed: word = 8*(coarse*128+fine) + 0x28ED,
 * src/fs1r/firmware/notes.cpp) and the unvoiced op's (word = (coarse*256 + fine*2)*4 + 0x28ED). */
static void voiced_hz(double hz, uint8_t *p) {
    int x = (int)std::lround((fs1r::Device::fseqWord(hz) - 0x28ED) / 8.0);
    x = x < 0 ? 0 : x > 31 * 128 + 127 ? 31 * 128 + 127 : x;
    p[1] = (uint8_t)(x >> 7); p[2] = (uint8_t)(x & 127);
}
static double voiced_to_hz(const uint8_t *p) {
    int w = 8 * ((p[1] & 31) * 128 + p[2]) + 0x28ED;
    return 440.0 * std::pow(2.0, (w - 26861) / 1024.0);
}
static void unvoiced_hz(double hz, uint8_t *q, int mode) {
    int x = (int)std::lround((fs1r::Device::fseqWord(hz) - 0x28ED) / 4.0);
    x = x < 0 ? 0 : x > 31 * 256 + 254 ? 31 * 256 + 254 : x;
    q[1] = (uint8_t)(mode << 5 | (x >> 8)); q[2] = (uint8_t)((x & 255) >> 1);
}
static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

/* A complete FS1R voice: algorithm 1 (all eight operators carriers), ops 1-5 voiced formants on
 * the vowel's F1..F5, op 6 a sine "body" on the fundamental, ops 1-2 unvoiced breath noise.
 * The Formant knob moves F1-F4, the FM knob widens them, and ops 1-5 follow an Fseq. */
static void build_vowel(uint8_t *v, int vowel, int type, int shape, int breath, int body, const VowelSpec &sp) {
    init_blank_voice(v);
    char name[11];
    std::snprintf(name, sizeof name, "Vowel %-4s", VOWEL_NAMES[vowel]);
    std::memcpy(v, name, 10);
    v[0x0E] = 19;                                   /* category: vocal */
    v[0x2C] = 0;                                    /* algorithm 1 */
    v[0x3D] = 0;                                    /* no feedback */
    v[0x28] = 0; v[0x29] = 0x1F;                    /* Fseq drives voiced ops 1-5 */
    v[0x2A] = 0; v[0x2B] = breath ? 0x03 : 0;       /* and the breath noise on ops 1-2 */
    for (int i = 0; i < 4; i++) {                   /* Formant knob -> F1..F4 frequency */
        v[0x40 + i] = (uint8_t)(2 << 4 | 0 << 3 | i); v[0x45 + i] = 72;
        v[0x4A + i] = (uint8_t)(3 << 4 | 0 << 3 | i); v[0x4F + i] = 80;   /* FM knob -> width */
    }
    v[0x44] = 0; v[0x49] = 64; v[0x4E] = 0; v[0x53] = 64;
    const int t = type == 3 ? 0 : type;
    const double scale = type == 3 ? 0.78 : 1.0;     /* GIANT: male formants a third lower */
    const int *L = SHAPE_L[shape], *T = SHAPE_T[shape];
    for (int k = 0; k < 8; k++) {
        uint8_t *p = v + 112 + 62 * k;
        for (int i = 0; i < 4; i++) { p[12 + i] = (uint8_t)L[i]; p[16 + i] = (uint8_t)T[i]; }
        uint8_t *q = p + 35;
        for (int i = 0; i < 4; i++) { q[13 + i] = (uint8_t)L[i]; q[17 + i] = (uint8_t)T[i]; }
        if (k < 5) {                                 /* voiced formant F(k+1) */
            double hz = VOWELS[t][vowel][k] * scale * std::pow(2.0, sp.shift[k] / 128.0);
            p[0] = 24;
            voiced_hz(hz, p);
            p[3] = 0;                                /* no key tracking: the formant stays put */
            p[4] = (uint8_t)(7 << 3 | 7);            /* bw bias 7 (centre), spectral form "formant" */
            p[5] = (uint8_t)(1 << 6 | clampi(sp.skirt[k], 0, 7) << 3 | k);   /* fixed, skirt, Fseq track k */
            p[6] = (uint8_t)clampi(F_BW[k] + sp.bwd[k], 1, 99);
            p[22] = (uint8_t)clampi(F_LEVEL[k] + sp.lvd[k], 0, 99);
        } else if (k == 5) {                         /* body: a sine at the played pitch */
            p[0] = 24; p[1] = 1; p[2] = 0; p[4] = 7 << 3; p[5] = 5;
            p[22] = (uint8_t)clampi(body, 0, 99);
        }
        if (k < 2 && breath) {                       /* breath: unvoiced noise around F3 and F4+ */
            double hz = VOWELS[t][vowel][k ? 4 : 2] * scale * 1.1;
            q[0] = 24;
            unvoiced_hz(hz, q, 0);
            q[4] = (uint8_t)(k ? 34 : 24);           /* width: 38.6 Hz a step */
            q[11] = (uint8_t)clampi(breath - k * 8, 0, 99);
        }
    }
}

/* ---- vocoder: the modulator is a WAV file from <plugin dir>/fsvr_vox, the carrier the FS1R ---- */
struct Biquad {
    float b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    inline float run(float x) { float y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
    void bandpass(double f, double q, double sr) {   /* RBJ, 0 dB peak; keeps the state */
        const double w = 2 * M_PI * std::min(f, sr * 0.45) / sr, al = std::sin(w) / (2 * q), a0 = 1 + al;
        b0 = (float)(al / a0); b1 = 0; b2 = (float)(-al / a0); a1 = (float)(-2 * std::cos(w) / a0); a2 = (float)((1 - al) / a0);
    }
    void highpass(double f, double q, double sr) {
        const double w = 2 * M_PI * f / sr, al = std::sin(w) / (2 * q), c = std::cos(w), a0 = 1 + al;
        b0 = (float)((1 + c) / 2 / a0); b1 = (float)(-(1 + c) / a0); b2 = b0; a1 = (float)(-2 * c / a0); a2 = (float)((1 - al) / a0);
    }
};
struct Band { Biquad m1, m2, c1, c2; float env = 0; };
struct Wave { std::vector<float> s; double rate = 44100; std::string name; };

/* A RIFF/WAVE file as mono float: PCM 8/16/24/32 bit, float 32, WAVE_FORMAT_EXTENSIBLE; at most 60 s. */
static bool load_wav(const std::string &path, Wave &out) {
    std::vector<uint8_t> b;
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    uint8_t buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0 && b.size() < 64u * 1024 * 1024) b.insert(b.end(), buf, buf + n);
    std::fclose(f);
    if (b.size() < 44 || std::memcmp(&b[0], "RIFF", 4) || std::memcmp(&b[8], "WAVE", 4)) return false;
    auto u16 = [&](size_t o) { return (int)(b[o] | b[o + 1] << 8); };
    auto u32 = [&](size_t o) { return (uint32_t)(b[o] | b[o + 1] << 8 | b[o + 2] << 16 | (uint32_t)b[o + 3] << 24); };
    int fmt = 0, ch = 0, bits = 0; uint32_t rate = 0; size_t data = 0, dlen = 0;
    for (size_t o = 12; o + 8 <= b.size();) {
        const uint32_t len = u32(o + 4);
        if (!std::memcmp(&b[o], "fmt ", 4) && o + 24 <= b.size()) {
            fmt = u16(o + 8); ch = u16(o + 10); rate = u32(o + 12); bits = u16(o + 22);
            if (fmt == 0xFFFE && o + 34 <= b.size()) fmt = u16(o + 32);   /* extensible: the sub-format */
        } else if (!std::memcmp(&b[o], "data", 4)) {
            data = o + 8; dlen = std::min((size_t)len, b.size() - data);
            break;
        }
        o += 8 + len + (len & 1);
    }
    if (!data || ch < 1 || rate < 4000 || !(fmt == 1 || fmt == 3)) return false;
    const int bps = bits / 8;
    if (bps < 1 || bps > 4 || (fmt == 3 && bits != 32)) return false;
    size_t frames = dlen / (size_t)(bps * ch);
    frames = std::min(frames, (size_t)rate * 60);
    out.s.assign(frames, 0.0f);
    out.rate = rate;
    for (size_t i = 0; i < frames; i++) {
        float sum = 0;
        for (int c = 0; c < ch; c++) {
            const uint8_t *p = &b[data + (i * ch + c) * bps];
            float v;
            if (fmt == 3) { float x; std::memcpy(&x, p, 4); v = x; }
            else if (bps == 1) v = (p[0] - 128) / 128.0f;
            else if (bps == 2) v = (int16_t)(p[0] | p[1] << 8) / 32768.0f;
            else if (bps == 3) v = (int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24) / 2147483648.0f;
            else v = (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24) / 2147483648.0f;
            sum += v;
        }
        out.s[i] = sum / ch;
    }
    return !out.s.empty();
}

static const int VOC_BANDS[4] = {8, 12, 16, 20};

struct Vocoder {
    std::vector<Band> bands;
    Biquad hp1, hp2;                   /* the modulator's sibilance (> 5 kHz), passed straight through */
    std::shared_ptr<const Wave> wave;
    double pos = -1;                   /* playback position in source frames, -1 stopped */
    double sr = 44100;
    float att = 0, rel = 0;
    /* settings, written by the UI thread, read per block */
    std::atomic<int> nbands{16}, shift{0}, mode{0}, start{0}, speed{100}, mix{100}, hf{30};
    std::atomic<bool> on{false}, redesign{true};

    void design() {
        const int nb = nbands.load();
        bands.resize((size_t)nb);
        const double lo = 120.0, hi = 7000.0, q = nb * 0.42;   /* log-spaced, overlapping a little */
        const double sh = std::pow(2.0, shift.load() / 12.0);
        for (int k = 0; k < nb; k++) {
            const double f = lo * std::pow(hi / lo, (k + 0.5) / nb);
            bands[(size_t)k].m1.bandpass(f, q, sr); bands[(size_t)k].m2.bandpass(f, q, sr);
            bands[(size_t)k].c1.bandpass(f * sh, q, sr); bands[(size_t)k].c2.bandpass(f * sh, q, sr);
        }
        hp1.highpass(5000, 0.7, sr); hp2.highpass(5000, 0.7, sr);
        att = (float)(1 - std::exp(-1.0 / (0.003 * sr)));
        rel = (float)(1 - std::exp(-1.0 / (0.040 * sr)));
    }
    void trigger() {
        const Wave *w = wave.get();
        pos = w ? w->s.size() * (start.load() / 100.0) : -1;
    }
    /* in place: out = dry * (1 - mix) + vocoded * mix */
    void process(float *L, float *R, int n) {
        if (redesign.exchange(false)) design();
        const Wave *w = wave.get();
        const int m = mode.load();
        const float wet = mix.load() / 100.0f, dry = 1 - wet, h = hf.load() / 100.0f;
        const double step = w ? w->rate / sr * speed.load() / 100.0 : 0;
        const size_t len = w ? w->s.size() : 0;
        const int nb = (int)bands.size();
        const float makeup = 6.0f * std::sqrt((float)nb);
        for (int i = 0; i < n; i++) {
            float mod = 0;
            if (w && pos >= 0) {
                const size_t i0 = (size_t)pos;
                const float fr = (float)(pos - (double)i0);
                mod = w->s[i0] + ((i0 + 1 < len ? w->s[i0 + 1] : 0.0f) - w->s[i0]) * fr;
                pos += step;
                if (pos >= (double)len) pos = m == 0 ? -1 : len * (start.load() / 100.0);   /* NOTE: once; LOOP/BAR: wrap */
                if (pos >= (double)len) pos = 0;
            }
            const float car = 0.5f * (L[i] + R[i]);
            float v = 0;
            for (int k = 0; k < nb; k++) {
                Band &b = bands[(size_t)k];
                const float e = std::fabs(b.m2.run(b.m1.run(mod)));
                b.env += (e > b.env ? att : rel) * (e - b.env);
                v += b.c2.run(b.c1.run(car)) * b.env;
            }
            v = v * makeup + h * hp2.run(hp1.run(mod));
            L[i] = L[i] * dry + v * wet;
            R[i] = R[i] * dry + v * wet;
        }
    }
};

static std::string vox_dir();
/* The WAV files in fsvr_vox, sorted by name. */
static std::vector<std::string> scan_vox() {
    std::vector<std::string> names;
    if (DIR *d = opendir(vox_dir().c_str())) {
        while (dirent *e = readdir(d)) {
            std::string nm = e->d_name;
            if (nm.size() > 4) {
                std::string ext = nm.substr(nm.size() - 4);
                for (auto &c : ext) c = (char)std::tolower((unsigned char)c);
                if (ext == ".wav" && nm[0] != '.') names.push_back(nm);
            }
        }
        closedir(d);
    }
    std::sort(names.begin(), names.end());
    if (names.size() > 99) names.resize(99);
    return names;
}

/* ---- per-instance state ----------------------------------------------------- */
struct Ev { int32_t delta; uint8_t b[3]; uint8_t len; };

struct Plugin {
    AEffect fx;
    audioMasterCallback master = nullptr;
    fs1r::Device dev;
    std::atomic<float> cache[NPARAMS];     /* what the host sees */
    std::atomic<int> notify[NPARAMS];      /* 1: tell the host the new cache value; 2: momentary release */
    float open[NPARAMS] = {0};             /* popup "open" flags (popup.h) */
    volatile int release[NPARAMS] = {0};   /* popup.h: a popup closed, report 0 to the host */
    std::atomic<bool> update_display{false};
    std::atomic<int> perf{0};              /* the last loaded factory performance */
    int pc_bank = 0;                       /* bank select LSB 0..2 for program change -> performance */
    float sr = 44100.0f;
    Ev evq[256];
    int ev_n = 0;
    std::mutex evMutex;
    std::mutex uiMutex;                    /* undo stack, vowel spec, user list */
    std::vector<uint8_t> chunk;
    std::deque<std::vector<uint8_t>> undo; /* engine states before each destructive action */
    bool vowel_active = false;             /* the voice in part 1 is a vowel this plugin built */
    VowelSpec spec;
    std::mt19937 rng;
    std::vector<std::string> user_names;   /* index = user slot - 1, "" = empty */
    double load_acc = 0, load_peak = 0, load_audio = 0;
    int idx_macro[16];
    Vocoder voc;
    std::mutex vocMutex;                   /* swapping voc.wave (UI) against processing it (audio) */
    std::vector<std::string> vox_names;    /* index = VOX FILE - 1 */
    std::string vox_loaded;                /* the file voc.wave came from */
    double next_bar = -1;                  /* BAR mode: the next bar start, in quarter notes */
    /* CPU savers */
    bool sleeping = false;                 /* silent and no note: dev.process is skipped */
    long quiet = 0;                        /* samples of silence so far */
    std::vector<int> held;                 /* held notes, oldest first (VOICES) */
    int banks[4] = {1, 0, 0, 0};           /* each part's voice bank as loaded (PARTS turns them off) */
    std::atomic<float> out_gain{4.0f};
};

static int P_(const char *key) {           /* parameter index by key, looked up once */
    return param_index(key);
}
static int IDX_OUTPUT, IDX_VOICES, IDX_PARTS, IDX_FX;
static int IDX_VOC_ON, IDX_VOC_FILE, IDX_VOC_MODE, IDX_VOC_START, IDX_VOC_SPEED, IDX_VOC_BANDS, IDX_VOC_SHIFT,
    IDX_VOC_MIX, IDX_VOC_HF;
static int IDX_PERF, IDX_CAT, IDX_VOWEL, IDX_VTYPE, IDX_SHAPE, IDX_BREATH, IDX_BODY, IDX_FSEQ,
    IDX_FSEQ_SPEED, IDX_MUT, IDX_USER, IDX_MONO, IDX_GLIDE;

static float clamp01(float v) { return v < 0 ? 0 : v > 1 ? 1 : v; }
static void copy_str(void *dst, const char *s, size_t max) {
    std::strncpy((char *)dst, s, max - 1);
    ((char *)dst)[max - 1] = 0;
}
static int norm_to_ui(const param_t *p, float n) {
    if (p->nopts) return (int)std::lround(clamp01(n) * (p->nopts - 1));
    return (int)std::lround(p->min + (p->max - p->min) * clamp01(n));
}
static float ui_to_norm(const param_t *p, double v) {
    if (p->nopts) return p->nopts > 1 ? clamp01((float)(v / (p->nopts - 1))) : 0.0f;
    return p->max > p->min ? clamp01((float)((v - p->min) / (p->max - p->min))) : 0.0f;
}
static int ui(Plugin *w, int i) { return i < 0 ? 0 : norm_to_ui(&PARAMS[i], w->cache[i].load()); }
static void set_cache(Plugin *w, int i, float n) {
    if (i < 0) return;
    w->cache[i].store(clamp01(n));
    w->notify[i].store(1);
}
static void set_ui(Plugin *w, int i, int v) { if (i >= 0) set_cache(w, i, ui_to_norm(&PARAMS[i], v)); }

/* ---- MIDI and sysex into the engine (channel forced to 1) -------------------------- */
static void send3(Plugin *w, uint8_t s, uint8_t d1, uint8_t d2) {
    const uint8_t m[3] = {s, d1, d2};
    w->dev.sendMidi(m, 3);
}
static void send_macro(Plugin *w, const Macro &m, int byte) {
    byte = clampi(byte, 0, 127);
    if (m.cc >= 0) { send3(w, 0xB0, (uint8_t)m.cc, (uint8_t)byte); return; }
    send3(w, 0xB0, 99, 0x01);              /* NRPN 01 xx, data entry */
    send3(w, 0xB0, 98, (uint8_t)m.nrpn);
    send3(w, 0xB0, 6, (uint8_t)byte);
}
/* An FS1R parameter change, device number 0. Wide (14-bit) parameters take the whole value. */
static void send_param(Plugin *w, int ah, int am, int al, int val) {
    const uint8_t m[10] = {0xF0, 0x43, 0x10, 0x5E, (uint8_t)ah, (uint8_t)am, (uint8_t)al,
                           (uint8_t)(val >> 7 & 0x7F), (uint8_t)(val & 0x7F), 0xF7};
    w->dev.sendMidi(m, 10);
}
static void send_voice(Plugin *w, int part, const uint8_t *v) {   /* one 608-byte voice bulk */
    const std::vector<uint8_t> m = bulk(0x40 + part, 0, 0, v, 608);
    w->dev.loadSyx(m.data(), m.size(), 0, part);
}

/* The engine's current performance (400 bytes) or a part's voice (608) from getState. */
static bool state_block(Plugin *w, int ah, size_t size, std::vector<uint8_t> &out) {
    std::vector<uint8_t> st;
    w->dev.getState(st);
    for (size_t i = 0; i + 11 < st.size(); i++) {
        if (st[i] != 0xF0 || st[i + 1] != 0x43 || st[i + 3] != 0x5E || st[i + 6] != ah) continue;
        if (i + 9 + size > st.size()) return false;
        out.assign(st.begin() + (long)i + 9, st.begin() + (long)(i + 9 + size));
        return true;
    }
    return false;
}

/* Every value the plugin shows that lives in the engine: macros, Fseq, mono, glide. */
static void readback(Plugin *w) {
    std::vector<uint8_t> d;
    if (!state_block(w, 0x10, 400, d)) return;
    for (int k = 0; k < NMACROS; k++) {
        int i = w->idx_macro[k];
        if (i < 0) continue;
        int b = d[192 + MACROS[k].part_off];   /* part 1 shows the performance's value */
        set_ui(w, i, MACROS[k].centred ? b - 64 : b);
    }
    const bool fs = (d[0x15] & 7) != 0 && (d[0x16] & 1);
    set_ui(w, IDX_FSEQ, fs ? d[0x17] + 1 : 0);
    const int ratio = d[0x18] << 7 | d[0x19];
    if (ratio >= 100) set_ui(w, IDX_FSEQ_SPEED, ratio / 10);
    set_ui(w, IDX_MONO, d[192 + 5] == 0 ? 1 : 0);
    set_ui(w, IDX_GLIDE, (d[192 + 0x24] & 1) ? d[192 + 0x25] : 0);
    w->update_display.store(true);
}

static void apply_parts(Plugin *w);
static void capture_banks(Plugin *w);
static void apply_mono(Plugin *w);
static void apply_glide(Plugin *w);

/* ---- undo --------------------------------------------------------------------------- */
static void push_undo(Plugin *w) {
    std::vector<uint8_t> st;
    w->dev.getState(st);
    std::lock_guard<std::mutex> lk(w->uiMutex);
    w->undo.push_back(std::move(st));
    while (w->undo.size() > 16) w->undo.pop_front();
}
static void do_undo(Plugin *w) {
    std::vector<uint8_t> st;
    {
        std::lock_guard<std::mutex> lk(w->uiMutex);
        if (w->undo.empty()) return;
        st = std::move(w->undo.back());
        w->undo.pop_back();
    }
    w->dev.allNotesOff();
    w->dev.setState(st.data(), st.size());
    w->vowel_active = false;
    capture_banks(w);
    apply_parts(w);
    readback(w);
    LOG("[fsvr_vst] undo\n");
}

/* ---- factory performances ------------------------------------------------------------ */
/* A factory performance with its four voices and its Fseq, as FSVR's loadFactoryPerf(). */
static void load_perf(Plugin *w, int index, bool undoable = true) {
    const Factory &f = factory();
    if (f.perfs.empty()) return;
    if (undoable) push_undo(w);
    index = clampi(index, 0, (int)f.perfs.size() - 1);
    const Item &it = f.perfs[(size_t)index];
    w->dev.allNotesOff();
    w->dev.loadSyx(it.syx.data(), it.syx.size(), 0, 0);
    const uint8_t *d = it.syx.data() + 9;
    for (int p = 0; p < 4; p++) {
        const int fv = factory_voice(d[192 + 52 * p + 1], d[192 + 52 * p + 2]);
        if (fv < 0 || fv >= (int)f.voices.size()) continue;
        const Item &v = f.voices[(size_t)fv];
        const std::vector<uint8_t> m = v.native ? readdress(v.syx, 0x40 + p, 0, 0) : v.syx;
        w->dev.loadSyx(m.data(), m.size(), 0, p);
    }
    if ((d[0x15] & 7) != 0 && (d[0x16] & 1)) {          /* a preset Fseq */
        const int num = d[0x17];
        if (num < (int)f.fseqs.size())
            w->dev.loadSyx(f.fseqs[(size_t)num].syx.data(), f.fseqs[(size_t)num].syx.size(), 0, 0);
    }
    w->perf.store(index);
    w->vowel_active = false;
    for (int p = 0; p < 4; p++) w->banks[p] = d[192 + 52 * p + 1];
    apply_parts(w);
    set_ui(w, IDX_PERF, index);
    readback(w);
    LOG("[fsvr_vst] performance %d \"%s\"\n", index, it.name.c_str());
}

/* RANDOM: a factory performance of the chosen category ("ALL" = any), never the current one. */
static void load_random(Plugin *w) {
    const Factory &f = factory();
    const int cat = ui(w, IDX_CAT);
    std::vector<int> pool;
    for (int i = 0; i < (int)f.perfs.size(); i++)
        if ((cat == 0 || f.perfs[(size_t)i].category == cat) && i != w->perf.load()) pool.push_back(i);
    if (pool.empty())
        for (int i = 0; i < (int)f.perfs.size(); i++) if (i != w->perf.load()) pool.push_back(i);
    if (pool.empty()) return;
    load_perf(w, pool[std::uniform_int_distribution<int>(0, (int)pool.size() - 1)(w->rng)]);
}

/* ---- Fseq --------------------------------------------------------------------------- */
/* Gives part 1's voice Fseq tracks if it has none: its formant operators, lowest first. */
static void fseq_tracks(Plugin *w) {
    std::vector<uint8_t> v;
    if (!state_block(w, 0x40, 608, v)) return;
    if ((v[0x28] & 1) || v[0x29]) return;               /* the voice has its own */
    std::vector<std::pair<double, int>> ops;
    for (int k = 0; k < 8; k++) {
        const uint8_t *p = v.data() + 112 + 62 * k;
        if ((p[4] & 7) == 7 && p[22] > 0) ops.push_back({voiced_to_hz(p), k});
    }
    if (ops.empty()) return;
    std::sort(ops.begin(), ops.end());
    int mask = 0;
    for (size_t r = 0; r < ops.size() && r < 8; r++) {
        uint8_t *p = v.data() + 112 + 62 * ops[r].second;
        p[5] = (uint8_t)((p[5] & 0x78) | (int)r);
        mask |= 1 << ops[r].second;
    }
    v[0x28] = (uint8_t)(mask >> 7 & 1); v[0x29] = (uint8_t)(mask & 0x7F);
    send_voice(w, 0, v.data());
}

static void set_fseq_speed(Plugin *w) { send_param(w, 0x10, 0, 0x18, clampi(ui(w, IDX_FSEQ_SPEED), 10, 500) * 10); }

/* n = 1..90 a preset Fseq on part 1, played at the key's pitch and restarted by every note;
 * 0 = no Fseq. */
static void choose_fseq(Plugin *w, int n, bool undoable = true) {
    const Factory &f = factory();
    if (undoable) push_undo(w);
    if (n <= 0 || n > (int)f.fseqs.size()) {
        send_param(w, 0x10, 0, 0x15, 0);
        set_ui(w, IDX_FSEQ, 0);
        return;
    }
    const Item &it = f.fseqs[(size_t)n - 1];
    w->dev.loadSyx(it.syx.data(), it.syx.size(), 0, 0);
    const uint8_t *h = it.syx.data() + 9;
    send_param(w, 0x10, 0, 0x16, 1);                     /* preset bank */
    send_param(w, 0x10, 0, 0x17, n - 1);
    send_param(w, 0x10, 0, 0x15, 1);                     /* part 1 */
    send_param(w, 0x10, 0, 0x1A, 0);                     /* start step */
    send_param(w, 0x10, 0, 0x1C, h[0x10] << 7 | h[0x11]);   /* the Fseq's own loop points */
    send_param(w, 0x10, 0, 0x1E, h[0x12] << 7 | h[0x13]);
    send_param(w, 0x10, 0, 0x20, 0);                     /* one-way loop */
    send_param(w, 0x10, 0, 0x21, 2);                     /* play mode: normal */
    send_param(w, 0x10, 0, 0x23, 1);                     /* pitch: the key's, not the Fseq's */
    send_param(w, 0x10, 0, 0x24, 1);                     /* every note restarts it */
    set_fseq_speed(w);
    fseq_tracks(w);
    set_ui(w, IDX_FSEQ, n);
    LOG("[fsvr_vst] Fseq %d \"%s\"\n", n, it.name.c_str());
}

/* ---- vowels ------------------------------------------------------------------------- */
/* Part 1 plays the built voice, parts 2-4 go quiet; the performance's effects stay. */
static void apply_vowel(Plugin *w) {
    uint8_t v[608];
    VowelSpec sp;
    { std::lock_guard<std::mutex> lk(w->uiMutex); sp = w->spec; }
    build_vowel(v, ui(w, IDX_VOWEL), ui(w, IDX_VTYPE), ui(w, IDX_SHAPE), ui(w, IDX_BREATH), ui(w, IDX_BODY), sp);
    send_voice(w, 0, v);
    std::vector<uint8_t> d;
    if (state_block(w, 0x10, 400, d) && d[192 + 1] == 0) send_param(w, 0x30, 0, 1, 1);   /* part 1 on */
    send_param(w, 0x30, 0, 0x0B, 120);                                                    /* part 1 volume */
    for (int p = 1; p < 4; p++) send_param(w, 0x30 + p, 0, 1, 0);                        /* 2-4 off */
    w->banks[0] = d.empty() || d[192 + 1] == 0 ? 1 : d[192 + 1];
    w->banks[1] = w->banks[2] = w->banks[3] = 0;
    if (ui(w, IDX_FSEQ) > 0) fseq_tracks(w);
}
static void make_vowel(Plugin *w, bool random) {
    push_undo(w);
    w->dev.allNotesOff();
    {
        std::lock_guard<std::mutex> lk(w->uiMutex);
        w->spec = VowelSpec();
        if (random) {
            std::normal_distribution<double> g(0.0, 1.0);
            std::uniform_int_distribution<int> sk(0, 2);
            for (int k = 0; k < 5; k++) {
                w->spec.shift[k] = (int)std::lround(g(w->rng) * 10.0);   /* about +-6 % */
                w->spec.bwd[k] = (int)std::lround(g(w->rng) * 6.0);
                w->spec.lvd[k] = k ? (int)std::lround(g(w->rng) * 7.0) : 0;
                w->spec.skirt[k] = sk(w->rng) == 2 ? sk(w->rng) : 0;
            }
        }
    }
    if (random) set_ui(w, IDX_VOWEL, std::uniform_int_distribution<int>(0, 7)(w->rng));
    w->vowel_active = true;
    /* part 1 neutral, whatever the performance had made of it: all 32 notes, the whole keyboard and
     * velocity range, no shift or detune, its offsets centred (the sends and the insertion stay) */
    static const uint8_t NEUTRAL[][2] = {{0x00, 32}, {0x08, 24}, {0x09, 64}, {0x0A, 64}, {0x0C, 64}, {0x0D, 64},
        {0x0F, 0}, {0x10, 127}, {0x15, 64}, {0x16, 64}, {0x17, 64}, {0x18, 64}, {0x19, 64}, {0x1A, 64}, {0x1B, 64},
        {0x1C, 64}, {0x1D, 64}, {0x1E, 64}, {0x1F, 64}, {0x20, 64}, {0x21, 64}, {0x22, 64}, {0x23, 64},
        {0x2A, 1}, {0x2B, 127}, {0x2C, 0}};
    for (auto &n : NEUTRAL) send_param(w, 0x30, 0, n[0], n[1]);
    for (int p = 1; p < 4; p++) send_param(w, 0x30 + p, 0, 0, 0);   /* their note reserve to part 1 */
    apply_vowel(w);
    apply_mono(w);
    apply_glide(w);
    readback(w);
    LOG("[fsvr_vst] vowel %s (%s)\n", VOWEL_NAMES[ui(w, IDX_VOWEL)], random ? "random" : "made");
}

/* ---- mutate: small random moves on part 1's voice, scaled by Amount ------------------- */
static void mutate(Plugin *w) {
    const double a = ui(w, IDX_MUT) / 100.0;
    if (a <= 0) return;
    std::vector<uint8_t> v;
    if (!state_block(w, 0x40, 608, v)) return;
    push_undo(w);
    std::normal_distribution<double> g(0.0, 1.0);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    auto nudge = [&](uint8_t &b, double sd, int lo, int hi) { b = (uint8_t)clampi(b + (int)std::lround(g(w->rng) * sd * a), lo, hi); };
    for (int k = 0; k < 8; k++) {
        uint8_t *p = v.data() + 112 + 62 * k;
        uint8_t *q = p + 35;
        if (p[22] > 0) {
            if ((p[4] & 7) == 7 || (p[5] >> 6 & 1)) {   /* a formant: move it, up to ~a third of an octave */
                int x = clampi((p[1] & 31) * 128 + p[2] + (int)std::lround(g(w->rng) * 30.0 * a), 0, 31 * 128 + 127);
                p[1] = (uint8_t)(x >> 7); p[2] = (uint8_t)(x & 127);
                nudge(p[6], 12, 1, 99);                       /* bandwidth */
                if (u01(w->rng) < 0.25 * a) p[5] = (uint8_t)((p[5] & 0x47) | clampi((p[5] >> 3 & 7) + (u01(w->rng) < 0.5 ? -1 : 1), 0, 7) << 3);
            }
            nudge(p[22], 10, 1, 99);                          /* level */
            for (int i = 0; i < 4; i++) nudge(p[16 + i], 12, 0, 99);   /* EG times */
        }
        if (q[11] > 0) {                                      /* an unvoiced formant */
            int x = clampi((q[1] & 31) * 256 + q[2] * 2 + (int)std::lround(g(w->rng) * 60.0 * a), 0, 31 * 256 + 254);
            q[1] = (uint8_t)((q[1] & 0x60) | x >> 8); q[2] = (uint8_t)((x & 255) >> 1);
            nudge(q[4], 8, 1, 99);
            nudge(q[11], 10, 0, 99);
        }
    }
    std::memcpy(v.data(), "Mutant    ", 10);
    send_voice(w, 0, v.data());
    w->vowel_active = false;
    LOG("[fsvr_vst] mutate %.0f %%\n", 100 * a);
}

/* ---- user library: <plugin dir>/fsvr_user/FSVR_nnn.syx, the engine state as bulk dumps --- */
static std::string plugin_dir() {
    Dl_info info;
    if (dladdr((void *)&plugin_dir, &info) && info.dli_fname) {
        std::string p = info.dli_fname;
        size_t s = p.rfind('/');
        if (s != std::string::npos) return p.substr(0, s);
    }
    return "/tmp";
}
static std::string user_dir() { return plugin_dir() + "/fsvr_user"; }
static std::string user_path(int n) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "/FSVR_%03d.syx", n);
    return user_dir() + buf;
}
static bool read_file(const std::string &path, std::vector<uint8_t> &out) {
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    out.clear();
    uint8_t buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) out.insert(out.end(), buf, buf + n);
    std::fclose(f);
    return !out.empty();
}
static std::string state_name(const std::vector<uint8_t> &st) {   /* the voice name of part 1 */
    for (size_t i = 0; i + 20 < st.size(); i++)
        if (st[i] == 0xF0 && st[i + 1] == 0x43 && st[i + 3] == 0x5E && st[i + 6] == 0x40) return text(&st[i + 9], 10);
    return "?";
}
static void scan_user(Plugin *w) {
    std::vector<std::string> names(199);
    for (int n = 1; n <= 199; n++) {
        std::vector<uint8_t> st;
        if (read_file(user_path(n), st)) names[(size_t)n - 1] = state_name(st);
    }
    std::lock_guard<std::mutex> lk(w->uiMutex);
    w->user_names = names;
}
static void save_user(Plugin *w) {
    ::mkdir(user_dir().c_str(), 0755);
    int n = 1;
    { std::lock_guard<std::mutex> lk(w->uiMutex);
      while (n <= 199 && !w->user_names[(size_t)n - 1].empty()) n++; }
    if (n > 199) { LOG("[fsvr_vst] user library full\n"); return; }
    std::vector<uint8_t> st;
    w->dev.getState(st);
    FILE *f = std::fopen(user_path(n).c_str(), "wb");
    if (!f) { LOG("[fsvr_vst] cannot write %s\n", user_path(n).c_str()); return; }
    std::fwrite(st.data(), 1, st.size(), f);
    std::fclose(f);
    { std::lock_guard<std::mutex> lk(w->uiMutex); w->user_names[(size_t)n - 1] = state_name(st); }
    set_ui(w, IDX_USER, n);
    LOG("[fsvr_vst] saved %s\n", user_path(n).c_str());
}
static void load_user(Plugin *w) {
    std::vector<uint8_t> st;
    if (!read_file(user_path(ui(w, IDX_USER)), st)) return;
    push_undo(w);
    w->dev.allNotesOff();
    if (!w->dev.setState(st.data(), st.size())) return;
    w->vowel_active = false;
    capture_banks(w);
    apply_parts(w);
    readback(w);
}

static std::string vox_dir() { return plugin_dir() + "/fsvr_vox"; }

/* VOX FILE n (1-based) into the vocoder; the list is read again each time. */
static void load_vox(Plugin *w, int n) {
    std::vector<std::string> names = scan_vox();
    std::shared_ptr<Wave> wv;
    if (n >= 1 && n <= (int)names.size()) {
        wv = std::make_shared<Wave>();
        if (!load_wav(vox_dir() + "/" + names[(size_t)n - 1], *wv)) {
            LOG("[fsvr_vst] cannot read %s (WAV, PCM or float)\n", names[(size_t)n - 1].c_str());
            wv.reset();
        } else {
            wv->name = names[(size_t)n - 1];
            LOG("[fsvr_vst] vocoder: %s, %.1f s at %.0f Hz\n", wv->name.c_str(), wv->s.size() / wv->rate, wv->rate);
        }
    }
    {
        std::lock_guard<std::mutex> lk(w->vocMutex);
        w->voc.wave = wv;
        w->voc.pos = -1;
    }
    std::lock_guard<std::mutex> lk(w->uiMutex);
    w->vox_names = names;
    w->vox_loaded = wv ? wv->name : "";
}
/* The settings the vocoder reads from the parameters. */
static void voc_settings(Plugin *w) {
    Vocoder &v = w->voc;
    const int nb = VOC_BANDS[clampi(ui(w, IDX_VOC_BANDS), 0, 3)], sh = ui(w, IDX_VOC_SHIFT);
    if (nb != v.nbands.load() || sh != v.shift.load()) { v.nbands.store(nb); v.shift.store(sh); v.redesign.store(true); }
    v.on.store(ui(w, IDX_VOC_ON) != 0);
    v.mode.store(ui(w, IDX_VOC_MODE));
    v.start.store(ui(w, IDX_VOC_START));
    v.speed.store(ui(w, IDX_VOC_SPEED));
    v.mix.store(ui(w, IDX_VOC_MIX));
    v.hf.store(ui(w, IDX_VOC_HF));
}

/* ---- PARTS: only the first n parts play (bank byte 0 = part off) ---------------------- */
static const int VOICE_LIMITS[6] = {0, 1, 2, 3, 4, 6};
static void apply_parts(Plugin *w) {
    const int lim = clampi(ui(w, IDX_PARTS), 0, 3);   /* 0 = all */
    std::vector<uint8_t> d;
    if (!state_block(w, 0x10, 400, d)) return;
    for (int p = 0; p < 4; p++) {
        const int want = (lim == 0 || p < lim) ? w->banks[p] : 0;
        if (d[192 + 52 * p + 1] != want) send_param(w, 0x30 + p, 0, 1, want);
    }
    if (state_block(w, 0x10, 400, d)) LOG("[fsvr_vst] parts %d: banks %d %d %d %d (kept %d %d %d %d)\n", lim, d[193], d[193 + 52], d[193 + 104], d[193 + 156], w->banks[0], w->banks[1], w->banks[2], w->banks[3]);
}
/* The parts' banks after a load. A part PARTS keeps off reads 0: its last known bank stays. */
static void capture_banks(Plugin *w) {
    std::vector<uint8_t> d;
    if (!state_block(w, 0x10, 400, d)) return;
    const int lim = clampi(ui(w, IDX_PARTS), 0, 3);
    for (int p = 0; p < 4; p++) {
        const int b = d[192 + 52 * p + 1];
        if (b || lim == 0 || p < lim) w->banks[p] = b;
    }
}
static void apply_output(Plugin *w) { w->out_gain.store(std::pow(10.0f, ui(w, IDX_OUTPUT) / 20.0f)); }

/* ---- mono and glide on all four parts ----------------------------------------------- */
static void apply_mono(Plugin *w) {
    for (int p = 0; p < 4; p++) send_param(w, 0x30 + p, 0, 5, ui(w, IDX_MONO) ? 0 : 1);
}
static void apply_glide(Plugin *w) {
    const int g = ui(w, IDX_GLIDE);
    for (int p = 0; p < 4; p++) {
        send_param(w, 0x30 + p, 0, 0x24, g > 0 ? 1 : 0);   /* switch on, fingered */
        send_param(w, 0x30 + p, 0, 0x25, g);
    }
}

/* ---- audio ---------------------------------------------------------------------- */
struct NoDenormals {   /* FSVR plugin.cpp: a heavy performance costs half as much again without it */
#if defined(__arm__) && defined(__ARM_FP)
    uint32_t old = 0;
    NoDenormals() { asm volatile("vmrs %0, fpscr" : "=r"(old)); asm volatile("vmsr fpscr, %0" : : "r"(old | (1u << 24))); }
    ~NoDenormals() { asm volatile("vmsr fpscr, %0" : : "r"(old)); }
#elif defined(__x86_64__) || defined(__i386__)
    unsigned old = __builtin_ia32_stmxcsr();
    NoDenormals() { __builtin_ia32_ldmxcsr(old | 0x8040); }
    ~NoDenormals() { __builtin_ia32_ldmxcsr(old); }
#endif
};

static void handle_event(Plugin *w, const Ev &e) {
    uint8_t s = e.b[0];
    if (e.len == 0) { w->voc.trigger(); return; }       /* BAR mode: a bar starts here */
    if (s < 0x80 || s >= 0xF0) return;
    const uint8_t t = s & 0xF0;
    if (t == 0xC0) {                                     /* program change -> factory performance */
        load_perf(w, w->pc_bank * 128 + e.b[1], false);
        return;
    }
    if (t == 0xB0 && e.b[1] == 32) { w->pc_bank = e.b[2] > 2 ? 2 : e.b[2]; return; }
    if (t == 0xB0 && e.b[1] == 0) return;                /* bank MSB: ours, not the engine's */
    if (t == 0x90 && e.b[2] > 0 && w->voc.mode.load() != 2) w->voc.trigger();   /* NOTE/LOOP: restart */
    if (t == 0x90 && e.b[2] > 0) {
        w->sleeping = false; w->quiet = 0;
        const int lim = VOICE_LIMITS[clampi(ui(w, IDX_VOICES), 0, 5)];
        w->held.erase(std::remove(w->held.begin(), w->held.end(), (int)e.b[1]), w->held.end());
        while (lim > 0 && (int)w->held.size() >= lim) {      /* VOICES: the oldest note goes */
            const uint8_t off[3] = {0x80, (uint8_t)w->held.front(), 0};
            w->dev.sendMidi(off, 3);
            w->held.erase(w->held.begin());
        }
        w->held.push_back(e.b[1]);
    } else if (t == 0x80 || t == 0x90) {
        w->held.erase(std::remove(w->held.begin(), w->held.end(), (int)e.b[1]), w->held.end());
    } else if (t == 0xB0 && (e.b[1] == 120 || e.b[1] == 123)) {
        w->held.clear();
    }
    const uint8_t m[3] = {t, e.b[1], e.b[2]};            /* everything onto channel 1 */
    w->dev.sendMidi(m, (t == 0xC0 || t == 0xD0) ? 2 : 3);
}

static void processReplacing(AEffect *e, float **in, float **out, int32_t n) {
    (void)in;
    Plugin *w = (Plugin *)e->object;
    const auto t0 = std::chrono::steady_clock::now();
    {
        NoDenormals nd;
        Ev q[256];
        int qn;
        {
            std::lock_guard<std::mutex> lk(w->evMutex);
            qn = w->ev_n;
            std::memcpy(q, w->evq, sizeof(Ev) * (size_t)qn);
            w->ev_n = 0;
        }
        if (w->voc.on.load() && w->voc.mode.load() == 2 && qn < 250) {   /* BAR: restart on every bar */
            VstTimeInfo *ti = (VstTimeInfo *)w->master(&w->fx, audioMasterGetTime, 0,
                                                        kVstPpqPosValid | kVstTempoValid | kVstTimeSigValid, 0, 0);
            if (ti && (ti->flags & kVstTransportPlaying) && (ti->flags & kVstPpqPosValid) && ti->tempo > 0) {
                const double beats = (ti->flags & kVstTimeSigValid) && ti->timeSigNumerator > 0
                                         ? ti->timeSigNumerator * 4.0 / std::max(1, ti->timeSigDenominator) : 4.0;
                const double ppq0 = ti->ppqPos, ppq1 = ppq0 + n * (ti->tempo / 60.0) / w->sr;
                for (double b = std::ceil(ppq0 / beats - 1e-9) * beats; b < ppq1 - 1e-9 && qn < 255; b += beats) {
                    Ev &m = q[qn++];
                    m.delta = (int32_t)((b - ppq0) / (ti->tempo / 60.0) * w->sr);
                    m.len = 0;
                }
            }
        }
        std::stable_sort(q, q + qn, [](const Ev &a, const Ev &b) { return a.delta < b.delta; });
        std::unique_lock<std::mutex> vl(w->vocMutex, std::defer_lock);
        const bool voc = w->voc.on.load() && vl.try_lock();     /* the UI swaps the WAV under this lock */
        auto render = [&](int32_t from, int32_t len) {
            if (w->sleeping) {                                   /* asleep: costs nothing */
                std::memset(out[0] + from, 0, sizeof(float) * (size_t)len);
                std::memset(out[1] + from, 0, sizeof(float) * (size_t)len);
                return;
            }
            w->dev.process(out[0] + from, out[1] + from, len);
            if (voc) w->voc.process(out[0] + from, out[1] + from, len);
        };
        int32_t pos = 0;
        for (int k = 0; k < qn; k++) {                   /* sample-accurate: render up to each event */
            int32_t at = q[k].delta < 0 ? 0 : q[k].delta > n ? n : q[k].delta;
            if (at > pos) { render(pos, at - pos); pos = at; }
            handle_event(w, q[k]);
        }
        if (pos < n) render(pos, n - pos);
        /* sleep after 1.5 s below -80 dBFS with no note held */
        if (!w->sleeping) {
            float pk = 0;
            for (int c = 0; c < 2; c++) for (int i = 0; i < n; i++) pk = std::max(pk, std::fabs(out[c][i]));
            if (pk < 1e-4f && w->dev.activeNotes() == 0) {
                w->quiet += n;
                if (w->quiet > (long)(1.5f * w->sr)) { w->sleeping = true; LOG("[fsvr_vst] sleeping\n"); }
            } else w->quiet = 0;
        }
        /* OUTPUT, then a soft knee above -3 dBFS */
        const float g = w->out_gain.load();
        for (int c = 0; c < 2; c++)
            for (int i = 0; i < n; i++) {
                float x = out[c][i] * g;
                const float a = std::fabs(x);
                if (a > 0.7f) {
                    const float t = std::min((a - 0.7f) / 0.3f, 3.0f), t2 = t * t;
                    x = std::copysign(0.7f + 0.3f * t * (27 + t2) / (27 + 9 * t2), x);
                }
                out[c][i] = x;
            }
    }
    /* load meter: DSP time against the block's real time */
    const double used = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const double real = n / (double)w->sr;
    w->load_acc += used;
    w->load_audio += real;
    if (real > 0 && used / real > w->load_peak) w->load_peak = used / real;
    if (w->load_audio >= 5.0) {
        LOG("[fsvr_vst] load avg %.0f %%, peak %.0f %% (perf %d, %d notes)\n",
            100.0 * w->load_acc / w->load_audio, 100.0 * w->load_peak, w->perf.load(), w->dev.activeNotes());
        w->load_acc = w->load_audio = w->load_peak = 0;
    }
    /* tell the host what changed */
    bool any = false;
    for (int i = 0; i < NPARAMS; i++) {
        if (w->release[i]) { w->release[i] = 0; any = true; w->master(&w->fx, audioMasterAutomate, i, 0, 0, 0.0f); }
        int f = w->notify[i].exchange(0);
        if (!f) continue;
        any = true;
        w->master(&w->fx, audioMasterAutomate, i, 0, 0, f == 2 ? 0.0f : w->cache[i].load());
    }
    if (w->update_display.exchange(false) || any) w->master(&w->fx, audioMasterUpdateDisplay, 0, 0, 0, 0.0f);
}

/* ---- parameters ------------------------------------------------------------------ */
static void press(Plugin *w, const char *key) {
    if (!std::strcmp(key, "perf_load")) load_perf(w, ui(w, IDX_PERF));
    else if (!std::strcmp(key, "rand_go")) load_random(w);
    else if (!std::strcmp(key, "panic")) w->dev.allNotesOff();
    else if (!std::strcmp(key, "make")) make_vowel(w, false);
    else if (!std::strcmp(key, "rand_vowel")) make_vowel(w, true);
    else if (!std::strcmp(key, "fseq_rand")) {
        int cur = ui(w, IDX_FSEQ), n;
        do n = std::uniform_int_distribution<int>(1, (int)factory().fseqs.size())(w->rng); while (n == cur && factory().fseqs.size() > 1);
        choose_fseq(w, n);
    }
    else if (!std::strcmp(key, "mutate")) mutate(w);
    else if (!std::strcmp(key, "undo")) do_undo(w);
    else if (!std::strcmp(key, "save")) save_user(w);
    else if (!std::strcmp(key, "user_load")) load_user(w);
}

static void setParameter(AEffect *e, int32_t i, float n) {
    Plugin *w = (Plugin *)e->object;
    if (i < 0 || i >= NPARAMS) return;
    const param_t *p = &PARAMS[i];
    if (popup_set(w->open, i, n)) return;
    if (p->momentary) {
        if (n <= 0.5f) return;
        press(w, p->key);
        w->notify[i].store(2);
        return;
    }
    bool nudge = false;
    if (p->nopts > 1) {   /* a Q-Link nudge lands between options -> step one option */
        float pos = clamp01(n) * (p->nopts - 1);
        if (std::fabs(pos - std::round(pos)) > 0.001f) {
            float cur = w->cache[i].load() * (p->nopts - 1);
            int idx = (int)std::lround(cur) + (pos > cur ? 1 : -1);
            idx = clampi(idx, 0, p->nopts - 1);
            n = (float)idx / (p->nopts - 1);
            nudge = true;
        }
    }
    const int before = ui(w, i);
    w->cache[i].store(clamp01(n));
    const int now = ui(w, i);
    for (int k = 0; k < NMACROS; k++)
        if (w->idx_macro[k] == i) send_macro(w, MACROS[k], MACROS[k].centred ? now + 64 : now);
    if (now != before) {
        if (i == IDX_VOWEL || i == IDX_VTYPE || i == IDX_SHAPE || i == IDX_BREATH || i == IDX_BODY) {
            if (w->vowel_active) apply_vowel(w);       /* tweak the vowel live */
        } else if (i == IDX_FSEQ) choose_fseq(w, now);
        else if (i == IDX_FSEQ_SPEED) set_fseq_speed(w);
        else if (i == IDX_MONO) apply_mono(w);
        else if (i == IDX_GLIDE) apply_glide(w);
        else if (i == IDX_VOC_FILE) load_vox(w, now);
        else if (i == IDX_OUTPUT) apply_output(w);
        else if (i == IDX_PARTS) apply_parts(w);
        else if (i == IDX_FX) w->dev.setEffects(now == 0);
        if (i == IDX_VOC_ON || i == IDX_VOC_MODE || i == IDX_VOC_START || i == IDX_VOC_SPEED || i == IDX_VOC_BANDS ||
            i == IDX_VOC_SHIFT || i == IDX_VOC_MIX || i == IDX_VOC_HF) {
            voc_settings(w);
            if (i == IDX_VOC_ON && now) load_vox(w, ui(w, IDX_VOC_FILE));   /* new files on the card */
        }
    }
    if (!nudge) popup_picked(w->open, w->release, i);
}

static float getParameter(AEffect *e, int32_t i) {
    Plugin *w = (Plugin *)e->object;
    if (i < 0 || i >= NPARAMS) return 0.0f;
    if (popup_is(i)) return w->open[i];
    return PARAMS[i].momentary ? 0.0f : w->cache[i].load();
}

/* ---- project chunk: "FSVR" 2, settings as text, then the engine's whole state ---------- */
static const char *const CHUNK_KEYS[] = {"perf", "rand_cat", "vowel", "vtype", "shape", "breath", "body",
                                         "fseq_speed", "mut_amt", "user", "voc_on", "voc_file", "voc_mode",
                                         "voc_start", "voc_speed", "voc_bands", "voc_shift", "voc_mix", "voc_hf",
                                         "output", "voices", "parts", "fx"};
static intptr_t get_chunk(Plugin *w, void **ptr) {
    std::string t;
    char buf[64];
    for (const char *k : CHUNK_KEYS) {
        int i = param_index(k);
        if (i < 0) continue;
        std::snprintf(buf, sizeof buf, "%s=%d;", k, ui(w, i));
        t += buf;
    }
    {
        std::lock_guard<std::mutex> lk(w->uiMutex);
        std::snprintf(buf, sizeof buf, "va=%d;", w->vowel_active ? 1 : 0);
        t += buf;
        if (!w->vox_loaded.empty() && w->vox_loaded.find(';') == std::string::npos) t += "vf=" + w->vox_loaded + ";";
        std::snprintf(buf, sizeof buf, "pb=%d,%d,%d,%d;", w->banks[0], w->banks[1], w->banks[2], w->banks[3]);
        t += buf;
        for (int k = 0; k < 5; k++) {
            std::snprintf(buf, sizeof buf, "s%d=%d,%d,%d,%d;", k, w->spec.shift[k], w->spec.bwd[k], w->spec.lvd[k], w->spec.skirt[k]);
            t += buf;
        }
    }
    std::vector<uint8_t> st;
    w->dev.getState(st);
    w->chunk = {'F', 'S', 'V', 'R', 2, (uint8_t)(t.size() >> 8), (uint8_t)(t.size() & 0xFF)};
    w->chunk.insert(w->chunk.end(), t.begin(), t.end());
    w->chunk.insert(w->chunk.end(), st.begin(), st.end());
    *ptr = w->chunk.data();
    return (intptr_t)w->chunk.size();
}
static intptr_t set_chunk(Plugin *w, const void *data, intptr_t len) {
    const uint8_t *b = (const uint8_t *)data;
    if (len < 7 || std::memcmp(b, "FSVR", 4) || b[4] != 2) return 0;
    const size_t tl = (size_t)(b[5] << 8 | b[6]);
    if (7 + tl > (size_t)len) return 0;
    std::string t((const char *)b + 7, tl);
    bool va = false;
    std::string vox_name;
    VowelSpec sp;
    size_t pos = 0;
    while (pos < t.size()) {
        size_t semi = t.find(';', pos);
        if (semi == std::string::npos) break;
        std::string kv = t.substr(pos, semi - pos);
        pos = semi + 1;
        size_t eq = kv.find('=');
        if (eq == std::string::npos) continue;
        std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
        if (k == "va") va = std::atoi(v.c_str()) != 0;
        else if (k == "vf") vox_name = v;
        else if (k == "pb") std::sscanf(v.c_str(), "%d,%d,%d,%d", &w->banks[0], &w->banks[1], &w->banks[2], &w->banks[3]);
        else if (k.size() == 2 && k[0] == 's' && k[1] >= '0' && k[1] <= '4') {
            int j = k[1] - '0';
            std::sscanf(v.c_str(), "%d,%d,%d,%d", &sp.shift[j], &sp.bwd[j], &sp.lvd[j], &sp.skirt[j]);
        } else {
            int i = param_index(k.c_str());
            if (i >= 0) set_ui(w, i, std::atoi(v.c_str()));
        }
    }
    w->perf.store(ui(w, IDX_PERF));
    { std::lock_guard<std::mutex> lk(w->uiMutex); w->spec = sp; w->undo.clear(); }
    if (!vox_name.empty()) {                    /* the file by name, wherever it sits in the list now */
        std::vector<std::string> names = scan_vox();
        for (size_t k = 0; k < names.size(); k++) if (names[k] == vox_name) set_ui(w, IDX_VOC_FILE, (int)k + 1);
    }
    load_vox(w, ui(w, IDX_VOC_FILE));
    voc_settings(w);
    w->dev.allNotesOff();
    const uint8_t *st = b + 7 + tl;
    const size_t sl = (size_t)len - 7 - tl;
    if (sl > 0 && w->dev.setState(st, sl)) {
        w->vowel_active = va;
        readback(w);
    } else {
        load_perf(w, w->perf.load(), false);   /* state unreadable: at least the performance */
    }
    apply_output(w);
    w->dev.setEffects(ui(w, IDX_FX) == 0);
    apply_parts(w);
    w->held.clear();
    w->sleeping = false; w->quiet = 0;
    return 1;
}

/* ---- dispatcher ---------------------------------------------------------------- */
static void display(Plugin *w, int idx, char *out) {
    const param_t *pp = &PARAMS[idx];
    const int u = popup_is(idx) ? norm_to_ui(pp, w->open[idx]) : ui(w, idx);
    char buf[48];
    const Factory &f = factory();
    if (idx == IDX_PERF) {   /* "A001 Zap!" - the unit's own bank letters, 128 per bank */
        std::snprintf(buf, sizeof buf, "%c%03d %s", 'A' + u / 128, u % 128 + 1,
                      u < (int)f.perfs.size() ? f.perfs[(size_t)u].name.c_str() : "");
    } else if (idx == IDX_FSEQ) {
        if (u <= 0 || u > (int)f.fseqs.size()) std::snprintf(buf, sizeof buf, "OFF");
        else std::snprintf(buf, sizeof buf, "%02d %s", u, f.fseqs[(size_t)u - 1].name.c_str());
    } else if (idx == IDX_USER) {
        std::lock_guard<std::mutex> lk(w->uiMutex);
        const std::string nm = u >= 1 && u <= (int)w->user_names.size() ? w->user_names[(size_t)u - 1] : "";
        std::snprintf(buf, sizeof buf, "U%03d %s", u, nm.empty() ? "--" : nm.c_str());
    } else if (idx == IDX_VOC_FILE) {
        std::lock_guard<std::mutex> lk(w->uiMutex);
        if (u >= 1 && u <= (int)w->vox_names.size()) {
            std::string nm = w->vox_names[(size_t)u - 1];
            nm = nm.substr(0, nm.size() - 4);
            std::snprintf(buf, sizeof buf, "%02d %s", u, nm.c_str());
        } else std::snprintf(buf, sizeof buf, "%02d --", u);
    } else if (idx == IDX_OUTPUT) {
        std::snprintf(buf, sizeof buf, "%+d dB", u);
    } else if (idx == IDX_VOC_SHIFT) {
        std::snprintf(buf, sizeof buf, "%+d st", u);
    } else if (idx == IDX_FSEQ_SPEED || idx == IDX_MUT || idx == IDX_VOC_START || idx == IDX_VOC_SPEED ||
               idx == IDX_VOC_MIX || idx == IDX_VOC_HF) {
        std::snprintf(buf, sizeof buf, "%d %%", u);
    } else if (pp->nopts) {
        std::snprintf(buf, sizeof buf, "%s", pp->opts[u]);
    } else {
        std::snprintf(buf, sizeof buf, "%d", u);
    }
    copy_str(out, buf, 24);
}

static intptr_t dispatcher(AEffect *e, int32_t op, int32_t idx, intptr_t v, void *p, float o) {
    Plugin *w = (Plugin *)e->object;
    switch (op) {
    case effOpen: return 1;
    case effClose:
        LOG("[fsvr_vst] closed\n");
        delete w;
        return 1;
    case effGetPlugCategory: return 2;   /* kPlugCategSynth */
    case effGetEffectName:
    case effGetProductString: copy_str(p, PLUG_NAME, 32); return 1;
    case effGetVendorString: copy_str(p, PLUG_VENDOR, 32); return 1;
    case effGetVendorVersion: return PLUG_VERSION;
    case effGetVstVersion: return 2400;
    case effCanBeAutomated: return idx >= 0 && idx < NPARAMS;
    case effGetParamName:
        if (idx >= 0 && idx < NPARAMS) copy_str(p, PARAMS[idx].name, 32);
        return 1;
    case effGetParamLabel:
        if (idx >= 0 && idx < NPARAMS) copy_str(p, PARAMS[idx].unit, 8);
        return 1;
    case effGetParamDisplay:
        if (idx < 0 || idx >= NPARAMS) return 0;
        if (PARAMS[idx].momentary) { copy_str(p, "", 24); return 1; }
        display(w, idx, (char *)p);
        return 1;
    case effSetSampleRate:
        if (o > 0) {
            w->sr = o; w->dev.setSampleRate(o);
            std::lock_guard<std::mutex> lk(w->vocMutex);
            w->voc.sr = o; w->voc.redesign.store(true);
        }
        return 1;
    case effSetBlockSize: return 1;
    case effMainsChanged: if (!v) w->dev.allNotesOff(); return 1;
    case effProcessEvents: {
        VstEvents *ev = (VstEvents *)p;
        std::lock_guard<std::mutex> lk(w->evMutex);
        for (int i = 0; ev && i < ev->numEvents && w->ev_n < 256; i++) {
            if (ev->events[i]->type != 1) continue;   /* kVstMidiType */
            const VstMidiEvent *me = (const VstMidiEvent *)ev->events[i];
            Ev &q = w->evq[w->ev_n++];
            q.delta = me->deltaFrames;
            std::memcpy(q.b, me->midiData, 3);
            q.len = 3;
        }
        return 1;
    }
    case effCanDo: {
        const char *s = (const char *)p;
        return (!std::strcmp(s, "receiveVstEvents") || !std::strcmp(s, "receiveVstMidiEvent")) ? 1 : -1;
    }
    case effGetChunk: return get_chunk(w, (void **)p);
    case effSetChunk: return set_chunk(w, p, v);
    default: return 0;
    }
}

/* Start values that differ from a parameter's minimum (gen_vst.py only knows ranges). */
static void start_values(Plugin *w) {
    set_ui(w, IDX_BREATH, 20);
    set_ui(w, IDX_BODY, 30);
    set_ui(w, IDX_FSEQ_SPEED, 100);
    set_ui(w, IDX_MUT, 30);
    set_ui(w, IDX_USER, 1);
    set_ui(w, IDX_VOC_FILE, 1);
    set_ui(w, IDX_VOC_SPEED, 100);
    set_ui(w, IDX_VOC_BANDS, 2);   /* 16 */
    set_ui(w, IDX_VOC_MIX, 100);
    set_ui(w, IDX_VOC_HF, 30);
    set_ui(w, IDX_OUTPUT, 12);
    set_ui(w, IDX_VOICES, 4);   /* 4 notes: a chord, and a cap on the CPU */
}

extern "C" __attribute__((visibility("default"))) AEffect *VSTPluginMain(audioMasterCallback master) {
    {
        std::lock_guard<std::mutex> lk(g_logMutex);
        if (!g_log) g_log = std::fopen("/tmp/fsvr_vst.log", "a");
    }
    static std::once_flag idx_once;
    std::call_once(idx_once, [] {
        IDX_PERF = P_("perf"); IDX_CAT = P_("rand_cat"); IDX_VOWEL = P_("vowel"); IDX_VTYPE = P_("vtype");
        IDX_SHAPE = P_("shape"); IDX_BREATH = P_("breath"); IDX_BODY = P_("body"); IDX_FSEQ = P_("fseq");
        IDX_FSEQ_SPEED = P_("fseq_speed"); IDX_MUT = P_("mut_amt"); IDX_USER = P_("user");
        IDX_MONO = P_("mono"); IDX_GLIDE = P_("glide");
        IDX_VOC_ON = P_("voc_on"); IDX_VOC_FILE = P_("voc_file"); IDX_VOC_MODE = P_("voc_mode");
        IDX_VOC_START = P_("voc_start"); IDX_VOC_SPEED = P_("voc_speed"); IDX_VOC_BANDS = P_("voc_bands");
        IDX_VOC_SHIFT = P_("voc_shift"); IDX_VOC_MIX = P_("voc_mix"); IDX_VOC_HF = P_("voc_hf");
        IDX_OUTPUT = P_("output"); IDX_VOICES = P_("voices"); IDX_PARTS = P_("parts"); IDX_FX = P_("fx");
    });

    Plugin *w = new Plugin();
    w->master = master;
    w->rng.seed((unsigned)std::time(nullptr) ^ (unsigned)(uintptr_t)w);
    for (int i = 0; i < NPARAMS; i++) { w->cache[i].store(PARAMS[i].def); w->notify[i].store(0); }
    for (int k = 0; k < NMACROS; k++) w->idx_macro[k] = param_index(MACROS[k].key);
    start_values(w);
    w->dev.setSampleRate(w->sr);
    w->dev.forceChannel(0);
    load_perf(w, 0, false);
    scan_user(w);
    w->voc.sr = w->sr;
    load_vox(w, ui(w, IDX_VOC_FILE));
    voc_settings(w);
    apply_output(w);

    AEffect *e = &w->fx;
    std::memset(e, 0, sizeof *e);
    e->magic = 0x56737450; /* 'VstP' */
    e->dispatcher = dispatcher;
    e->setParameter = setParameter;
    e->getParameter = getParameter;
    e->processReplacing = processReplacing;
    e->numParams = NPARAMS;
    e->numInputs = 0;
    e->numOutputs = 2;
    e->flags = effFlagsCanReplacing | effFlagsIsSynth | effFlagsProgramChunks;
    e->uniqueID = PLUG_UID;
    e->version = PLUG_VERSION;
    e->object = w;
    LOG("[fsvr_vst] up, %d params\n", NPARAMS);
    return e;
}
