// fsvr/device.cpp - fs1r::Device: host sample rate, state, the public surface. Ours.
#include "fs1r/internal.h"

// ------------------------------------------------------------------------------------------ fs1r::Device
namespace fs1r {

struct Device::Impl {
    Synth s;
    Rom rom;
    double hostRate = ENGINE_RATE;
    double pos = 0;                          // fractional read position between engine samples
    float h[2][4] = {};                      // four-point history per channel for the interpolator
    float eng[2][256]; int engFill = 0, engRead = 0;
    bool echo = false;
    // ponytail: cubic interpolation between engine samples. Audible aliasing above ~15 kHz when the host
    // runs at 44.1; swap in a polyphase FIR if a spectrum measurement ever shows it matters.
    inline void pull() {
        if (engRead >= engFill) {
            engFill = 32; engRead = 0;   // mpc-vst-fsvr: 32, not 256 - an even load per host block
            s.render(eng[0], eng[1], engFill);
        }
        for (int c = 0; c < 2; c++) { h[c][0] = h[c][1]; h[c][1] = h[c][2]; h[c][2] = h[c][3]; h[c][3] = eng[c][engRead]; }
        engRead++;
    }
    static inline float cubic(const float* v, double t) {
        double a = v[3] - v[2] - v[0] + v[1], b = v[0] - v[1] - a, c = v[2] - v[0];
        return (float)(((a * t + b) * t + c) * t + v[1]);
    }
};

Device::Device() : p(new Impl) { init_tables(); }
Device::~Device() = default;

void Device::setSampleRate(double hostRate) { if (hostRate > 1000) p->hostRate = hostRate; }
double Device::sampleRate() const { return p->hostRate; }
void Device::setGain(double g) { p->s.gain = g; }
void Device::setMaxChannels(int n) { std::lock_guard<std::mutex> lk(p->s.mtx); p->s.maxChan = n; }   // mpc-vst-fsvr
void Device::setEffects(bool on) { std::lock_guard<std::mutex> lk(p->s.mtx); p->s.fxBypass = !on; }   // mpc-vst-fsvr
void Device::setEchoParameters(bool on) { p->echo = on; }
void Device::forceChannel(int channel) { p->s.forceChannel = channel; }

void Device::process(float* outL, float* outR, int n) {
    if (p->hostRate == ENGINE_RATE) {        // the common case: no resampling at all
        p->s.render(outL, outR, n);
        return;
    }
    const double step = ENGINE_RATE / p->hostRate;
    for (int i = 0; i < n; i++) {
        p->pos += step;
        while (p->pos >= 1.0) { p->pos -= 1.0; p->pull(); }
        outL[i] = Impl::cubic(p->h[0], p->pos);
        outR[i] = Impl::cubic(p->h[1], p->pos);
    }
}

void Device::sendMidi(const uint8_t* b, size_t len) {
    if (!len) return;
    if (b[0] == 0xF0) {
        std::lock_guard<std::mutex> lk(p->s.mtx);
        if (load_sysex(p->s, p->rom.ok ? &p->rom : nullptr, b, len, 0, 0)) return;
        if (apply_param_change_locked(p->s, b, len) && p->echo && len >= 10 && (b[2] & 0xF0) == 0x10)
            p->s.push_param(b[4], b[5], b[6], b[7] << 7 | b[8]);
        return;
    }
    std::lock_guard<std::mutex> lk(p->s.mtx);
    p->s.midi_in(b[0], len > 1 ? b[1] & 0x7F : 0, len > 2 ? b[2] & 0x7F : 0);
}

bool Device::nextMidiOut(std::vector<uint8_t>& out) {
    std::lock_guard<std::mutex> lk(p->s.mtx);
    if (p->s.outQ.empty()) return false;
    out.swap(p->s.outQ.front());
    p->s.outQ.erase(p->s.outQ.begin());
    return true;
}

void Device::allNotesOff() { std::lock_guard<std::mutex> lk(p->s.mtx); p->s.all_off(); }

void Device::getState(std::vector<uint8_t>& sysex) const {
    std::lock_guard<std::mutex> lk(p->s.mtx);
    Synth& S = p->s;
    S.outQ.clear();
    S.push_bulk(0x00, 0, 0, S.sys, 76);
    uint8_t perfBytes[400]; S.current_perf_bytes(perfBytes);
    S.push_bulk(0x10, 0, 0, perfBytes, 400);
    for (int i = 0; i < 4; i++) S.push_bulk(0x40 + i, 0, 0, S.perf.part[i].voice.raw, 608);
    if (S.fseq.valid) { std::vector<uint8_t> f; S.fseq_bytes(f); S.push_bulk(0x60, 0, 0, f.data(), (int)f.size()); }   // the current Fseq, Data List 3.2.1
    sysex.clear();
    for (auto& m : S.outQ) sysex.insert(sysex.end(), m.begin(), m.end());
    S.outQ.clear();
}

bool Device::setState(const uint8_t* d, size_t len) {
    bool any = false;
    for (size_t i = 0; i < len; ) {
        if (d[i] != 0xF0) { i++; continue; }
        size_t j = i + 1;
        while (j < len && d[j] != 0xF7) j++;
        if (j >= len) break;
        size_t n = j - i + 1;
        {
            std::lock_guard<std::mutex> lk(p->s.mtx);
            // The system block getState wrote: the firmware's loader takes no system bulk, and this is the
            // device's own state rather than a dump off the MIDI input, so it goes back byte for byte.
            if (n == 76 + 11 && d[i + 3] == 0x5E && d[i + 6] == 0x00) { memcpy(p->s.sys, d + i + 9, 76); any = true; }
            else if (load_sysex(p->s, p->rom.ok ? &p->rom : nullptr, d + i, n, 0, 0)) any = true;
            else if (apply_param_change_locked(p->s, d + i, n)) any = true;
        }
        i = j + 1;
    }
    // The Fseq bulk's loader gives a performance with no Fseq part the first one, as a load from the Fseq
    // page wants; restoring a saved state is not that, and the performance's own Fseq part stands.
    std::lock_guard<std::mutex> lk(p->s.mtx);
    p->s.fseqPart = (p->s.perf.c[0x15] & 7) ? (p->s.perf.c[0x15] & 7) - 1 : -1;
    return any;
}

bool Device::loadRom(const char* path) {
    if (!load_rom(p->rom, path)) return false;
    p->s.rom = &p->rom;
    return true;
}
bool Device::romLoaded() const { return p->rom.ok; }
bool Device::loadRomPerformance(int idx) { if (!p->rom.ok) return false; return rom_perf(p->s, p->rom, idx); }
bool Device::loadRomVoice(int part, int idx) {
    if (!p->rom.ok || part < 0 || part > 3) return false;
    std::lock_guard<std::mutex> lk(p->s.mtx);
    rom_voice(p->rom, idx, p->s.perf.part[part].voice);
    return true;
}
bool Device::loadRomFseq(int n) { if (!p->rom.ok) return false; return rom_fseq(p->s, p->rom, n); }
bool Device::loadSyx(const uint8_t* d, size_t len, int pick, int part) {
    return load_sysex(p->s, p->rom.ok ? &p->rom : nullptr, d, len, pick, part);
}

const char* Device::performanceName() const { return p->s.perf.name; }
const char* Device::voiceName(int part) const { auto& pt = p->s.perf.part[clampi(part, 0, 3)]; return pt.p[1] == 0 ? "off" : pt.voice.name; }   // the panel shows "off" for a part with no bank
int Device::algorithm(int part) const { return p->s.perf.part[clampi(part, 0, 3)].voice.alg; }
bool Device::partActive(int part) const { return p->s.perf.part[clampi(part, 0, 3)].rcv() != 0x7F; }
const char* Device::fseqName() const { return p->s.fseq.valid ? p->s.fseq.name : ""; }
int Device::fseqFrames() const { return p->s.fseq.valid ? p->s.fseq.nframes : 0; }
bool Device::fseqFrame(int step, uint8_t out[50]) const {
    if (!p->s.fseq.valid || step < 0 || step >= p->s.fseq.nframes) return false;
    memcpy(out, p->s.fseq.frame[step], 50);
    return true;
}
int Device::fseqPosition() const { return p->s.fseq.valid ? p->s.fseqStep : 0; }
int Device::fseqPart() const { return p->s.fseqPart; }
double Device::fseqFrameSeconds(int s) { return (VELW[clampi(s, 0, 127)] * 84.0 + 2884.0) * 32.0 / CPU_HZ; }   // fseq_start at ratio 1000
// word_hz inverted, and a frame level: notes.cpp refresh_regs doubles the byte into the level register, and
// the chip attenuates LEVEL_DB per register step (ymp706.cpp).
int Device::fseqWord(double hz) { return hz > 0 ? clampi((int)lround(26861 + 1024 * log2(hz / 440.0)), 0, 0x7FFE) : 0; }
int Device::fseqLevel(double gain) { return gain > 0 ? clampi((int)lround(-20 * log10(gain) / (2 * LEVEL_DB)), 0, 127) : 127; }
int Device::activeNotes() const {
    std::lock_guard<std::mutex> lk(p->s.mtx);
    int n = 0;
    for (auto& c : p->s.ch) n += c.active && (c.held || c.sustained);
    return n;
}

int Device::selfTest() {
    init_tables();
    Synth s;
    int fails = selftest(s);
    // A saved state with an Fseq loaded but no Fseq part comes back with none: the plug-in's session
    // restore once played the last Fseq on part 1 under a performance that has no Fseq.
    std::vector<uint8_t> f(9 + 32 + 128 * 50 + 2, 0), state;
    f[0] = 0xF0; f[1] = 0x43; f[3] = 0x5E; f[6] = 0x60; f.back() = 0xF7;
    Device a, b;
    a.sendMidi(f.data(), f.size());
    const uint8_t off[10] = {0xF0, 0x43, 0x10, 0x5E, 0x10, 0x00, 0x15, 0x00, 0x00, 0xF7};
    a.sendMidi(off, sizeof off);
    a.getState(state);
    b.setState(state.data(), state.size());
    if (a.fseqPart() != -1 || b.fseqPart() != -1) { printf("  FAIL state restore enables the Fseq (part %d)\n", b.fseqPart()); fails++; }
    return fails;
}

}  // namespace fs1r

