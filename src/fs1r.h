// fs1r.h - the FS1R engine as a device: audio out, MIDI in, MIDI out, state as sysex.
//
// Nothing here knows about Windows, a host, or a GUI. The plugin layer never reads engine internals: it
// moves parameter values in and out as sysex exactly as a hardware editor would, and learns the state
// back from the engine's MIDI output.
//
// The engine always runs at the hardware's 48 kHz; setSampleRate() only sets the rate of the resampler
// on the way out, so patch timing is identical at every host rate.
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace fs1r {

static const double ENGINE_RATE = 48000.0;   // the FS1R's DAC clock, measured on rgwan's digital-out board

class Device {
public:
    Device();
    ~Device();
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    // ---- audio. Real-time safe: no allocation, no locks taken that the MIDI side holds for long.
    void setSampleRate(double hostRate);
    double sampleRate() const;
    void process(float* outL, float* outR, int numSamples);
    void setGain(double g);
    // mpc-vst-fsvr: false skips the insertion, variation and reverb blocks (parts go out dry) to save CPU.
    void setEffects(bool on);
    // mpc-vst-fsvr: at most n of the 32 channels are used; a note beyond takes one over (damped, as the
    // unit does when all 32 are busy), so release tails cannot pile up. 32 = the unit.
    void setMaxChannels(int n);

    // ---- MIDI. One channel message or one complete sysex message per call.
    void sendMidi(const uint8_t* bytes, size_t len);
    // Drains one message the engine wants to send (dump and parameter replies, parameter echoes).
    // Returns false when the queue is empty.
    bool nextMidiOut(std::vector<uint8_t>& out);
    // Echo every parameter change back on the MIDI output, so an editor or the plugin GUI tracks the
    // engine without reading its internals. Off by default; the console does not need it.
    void setEchoParameters(bool on);
    void allNotesOff();

    // ---- state: the whole device as a run of FS1R bulk dumps (system, performance, four voices, Fseq).
    void getState(std::vector<uint8_t>& sysex) const;
    bool setState(const uint8_t* data, size_t len);

    // ---- patch sources
    bool loadRom(const char* path);            // the 2 MB v1.20 EPROM image
    bool romLoaded() const;
    bool loadRomPerformance(int index);        // 0..383, the EPROM's PrA, PrB and PrC banks
    bool loadRomVoice(int part, int index);    // 0..1407
    bool loadRomFseq(int number);              // 1..90
    // FS1R voice / performance / Fseq bulk dumps and DX7 VCED files; pick selects the n-th match.
    bool loadSyx(const uint8_t* data, size_t len, int pick, int part);

    // ---- what the console and a patch browser print
    const char* performanceName() const;
    const char* voiceName(int part) const;
    int algorithm(int part) const;             // 0-based
    bool partActive(int part) const;
    const char* fseqName() const;
    int fseqFrames() const;                    // 0 when no Fseq is loaded
    // One Fseq frame as its 50 raw bytes, and where playback currently is. For a display only.
    bool fseqFrame(int step, uint8_t out[50]) const;
    int fseqPosition() const;
    int fseqPart() const;                      // -1 when no part is assigned
    int activeNotes() const;                   // notes held or sustained, for a polyphony readout
    // How long one Fseq frame lasts at 100 % speed for a header speed adjust of 0..127 (fseq_start).
    static double fseqFrameSeconds(int speedAdjust);
    // An Fseq frame's bytes on the engine's scales: a frequency as the word its hi and lo bytes make
    // (hi * 256 + lo * 2), and a gain 0..1 as its level byte (an attenuation, 0 loudest).
    static int fseqWord(double hz);
    static int fseqLevel(double gain);
    void forceChannel(int channel);            // -1 = parts use their own receive channels

    // Engine self check: every sysex path, RPN/NRPN, bank select, a note. Prints failures, returns the
    // number of them.
    static int selfTest();

    struct Impl;
private:
    std::unique_ptr<Impl> p;
};

}  // namespace fs1r
