// fsvr/selftest.cpp - the engine self check. Ours, not the hardware's.
#include "fs1r/internal.h"
#include "display.h"
#include <cstring>

// ------------------------------------------------------------------------------------------ self test
// fsvr_console -selftest: every sysex path the plugin will use. Parameter change in, parameter request out,
// bulk dump out and straight back in, and the whole 400/608 byte state surviving the round trip.
static int g_fails = 0;
static void ck(const char* what, bool ok) { if (!ok) { printf("  FAIL %s\n", what); g_fails++; } }
int selftest(Synth& S) {
    ck("fastmath backend within its error budget of libm", fm::selfcheck() == 0);
    // 1. a parameter change reaches the engine at every address class
    struct { int ah, am, al, v; const char* name; } pc[] = {
        {0x00, 0, 0x0E, 3, "system velocity curve"},
        {0x10, 0, 0x11, 100, "performance pan"},
        {0x10, 0, 0x58, 5, "reverb type"},
        {0x10, 1, 0x2F, 9, "insertion type"},
        {0x31, 0, 0x0E, 20, "part 2 pan"},
        {0x42, 0, 0x54, 2, "part 3 voice filter type"},
        {0x63, 5, 0x16, 77, "part 4 op 6 level"},
        {0x60, 1, 0x05, 5 << 3 | 1, "part 1 op 2 skirt"},   // what captures/sweep.py skirt sweeps
    };
    for (auto& t : pc) {
        uint8_t m[10] = {0xF0, 0x43, 0x10, 0x5E, (uint8_t)t.ah, (uint8_t)t.am, (uint8_t)t.al, 0, (uint8_t)t.v, 0xF7};
        ck(t.name, apply_param_change_locked(S, m, 10));
        ck(t.name, S.read_param(t.ah, t.am, t.al) == t.v);
    }
    ck("voice decode followed the change", S.perf.part[2].voice.fltType == 2);
    ck("op level decode followed the change", S.perf.part[3].voice.v[5].level == 77);
    // The ladder's zero-delay solve and its (1 + k) normalisation: a DC input has to come back out at
    // unity from every tap, at every feedback the resonance table can reach. The mean over the second
    // half of the run averages the ring away, and a loop that has gone unstable blows this up.
    for (int raw : {0, 32, 64, 116}) {
        for (int tap = 2; tap <= 4; tap++) {
            Ladder L; L.set(1000.0, reso_fb(raw - 16), 0.0);
            double acc = 0; int n = 0;
            for (int i = 0; i < 96000; i++) { double y = L.run(1.0, tap); if (i >= 48000) { acc += y; n++; } }
            ck("ladder unity passband", fabs(acc / n - 1.0) < 0.05);
        }
    }

    // The chip's EG key rate scaling, against the 24 rates 02_envelope_3 measured. Key codes 85, 93
    // and 101 are notes 36, 60 and 84; the rows are the rate offset at time scaling 0 to 7, and the
    // truncation toward zero is what separates the middle row from a flooring divide. docs/aeg.md.
    {
        const int kc[3] = {85, 93, 101};
        const int want[3][8] = {{0, -1, -2, -3, -5, -6, -7, -8},
                                {0, 0, 0, 0, -1, -1, -1, -1},
                                {0, 0, 1, 1, 2, 2, 3, 3}};
        for (int n = 0; n < 3; n++)
            for (int t = 0; t < 8; t++) ck("EG rate scaling", eg_ratescale(t, kc[n]) == want[n][t]);
        // and 10_envelope2's ten key codes, one note per octave from 12 to 120, at time scaling 7 and 3
        const int kc2[10] = {77, 81, 85, 89, 93, 97, 101, 105, 109, 113};
        const int w7[10] = {-11, -11, -8, -4, -1, 1, 3, 7, 10, 11};
        const int w3[10] = {-4, -4, -3, -1, 0, 0, 1, 3, 4, 4};
        for (int n = 0; n < 10; n++) {
            ck("EG rate scaling, tscale 7", eg_ratescale(7, kc2[n]) == w7[n]);
            ck("EG rate scaling, tscale 3", eg_ratescale(3, kc2[n]) == w3[n]);
        }
        ck("EG rate scaling saturates low", eg_ratescale(7, 60) == -11);
        ck("key code 87, the demo drum's note 41, reads -8 (19_drums)", eg_keyoff(87) == -8);
        // 21_keycode, every semitone: saturated below 82 and above 109, and the hold does not scale
        ck("key code table ends (21_keycode)", eg_keyoff(70) == -13 && eg_keyoff(81) == -13 && eg_keyoff(82) == -12 && eg_keyoff(109) == 12 && eg_keyoff(110) == 13 && eg_keyoff(127) == 13);
        { EG lo, hi; int L[4] = {0, 0, 0, 63}, R[4] = {0, 34, 0, 0}; lo.start(L, R, 40, -11); hi.start(L, R, 40, 11);
          ck("hold length ignores the key code (21_keycode)", lo.holdLeft == hi.holdLeft); }
        ck("EG rate scaling saturates high", eg_ratescale(7, 127) == 11);
    }
    // The formant window against the bandwidth byte, 04_formant_1's own staircase: flat to 40, opening
    // from 44, halving every eight steps after that.
    {
        // The window is a time, so its length in periods scales with the fundamental and its width in Hz
        // does not. Both are checked here because getting the units wrong is what cost two fitted knees.
        auto hz = [](double bw) { return cal::FRMT_BW_HZ0 * pow(2.0, bw / cal::FRMT_BW_DB); };
        auto wl = [&](double bw, double f0) { return std::min(cal::FRMT_WL_MAX, f0 / hz(bw)); };
        ck("formant bandwidth doubles every eight", fabs(hz(56) / hz(48) - 2.0) < 1e-9);
        ck("formant bandwidth is the same in Hz at any note", fabs(hz(64) - hz(64)) < 1e-12);
        ck("the window is four times as many periods an octave down",
           fabs(wl(72, 65.41) * 4.0 - wl(72, 261.64)) < 1e-6);
        ck("the window clamps at two periods", wl(0, 261.64) == cal::FRMT_WL_MAX);
    }
    // The skirt multiplies the window exponent, it does not step it. A sin^(2n) window one grain period
    // long puts its lines on the binomial row C(2n, n-k), which is what res2 gives the unit: 6.02 dB down
    // either side at skirt 0, 3.52 and 15.56 at skirt 1. Checking the table against those rows pins the
    // law and the doubling at once, and the sine table's own interpolation with them.
    {
        auto line = [&](int s, int k) {          // the k-th Fourier coefficient of g_win[0][s]
            double re = 0;
            for (int i = 0; i < 1024; i++) re += g_win[0][s][i] * cos(2 * PI * k * i / 1024.0);
            return re;
        };
        auto row_db = [&](int s, int k) { return 20 * log10(fabs(line(s, k) / line(s, 0))); };
        ck("skirt 0 is sin^2: one period either side 6.02 dB down", fabs(row_db(0, 1) + 6.0206) < 0.02);
        ck("skirt 0 is sin^2: nothing two periods out", row_db(0, 2) < -60);
        ck("skirt 1 is sin^4, not sin^4 by a different route", fabs(row_db(1, 1) + 3.5218) < 0.02
                                                            && fabs(row_db(1, 2) + 15.563) < 0.05);
        ck("skirt 2 is sin^8", fabs(row_db(2, 1) + 1.9382) < 0.02 && fabs(row_db(2, 3) + 18.837) < 0.1);
        ck("the exponent doubles rather than stepping", fabs(row_db(3, 1) + 1.0216) < 0.02);
        // The asymmetric family: the same rise, a sin^2 fall whatever the skirt, and no mean in the
        // stored all1/all2 waveform, which is the window with g_winDC taken out.
        ck("the '1' family shares the rise", g_win[1][5][256] == g_win[0][5][256]);
        ck("the '1' family falls as sin^2", fabs(g_win[1][5][768] - g_win[0][0][768]) < 1e-6);
        ck("the '1' family is asymmetric", g_win[1][5][768] > 4 * g_win[0][5][768]);
        double dc = 0; for (int i = 0; i < 1024; i++) dc += g_win[0][0][i] - g_winDC[0][0];
        ck("all2 at skirt 0 carries no DC", fabs(dc) < 1e-3);
    }
    // Register 0xC0 against the note. docs/ymp706_registers.md used to gloss it as "note/3 + 10", which is
    // out by 63 and is what made the old rate scaling look like dead code when it was merely wrong.
    {
        const int note[5] = {0, 36, 60, 84, 127};
        const int want[5] = {73, 85, 93, 101, 116};
        for (int i = 0; i < 5; i++) ck("key code register", (NOTETAB[note[i]] >> 8) + 10 == want[i]);
    }
    // Pan key scaling at its extreme byte, the ten notes 10_envelope2 measured: hard right at 12, centre
    // at 60, hard left from 108 up.
    {
        const int note[10] = {12, 24, 36, 48, 60, 72, 84, 96, 108, 120};
        const int want[10] = {128, 112, 96, 80, 64, 48, 32, 16, 0, -16};
        for (int i = 0; i < 10; i++) ck("pan key scaling", pan_index(64, 0, note[i]) == want[i]);
        ck("pan key scaling centred", pan_index(64, 50, 12) == 64 && pan_index(64, 50, 120) == 64);
    }
    // A rise from silence starts at the attack floor and gets to its target, and the hold is half a
    // traverse plus the lag. Both are measured; an EG that crawls up from -200 dB is the old bug.
    {
        int L[4] = {0, 0, 0, 63}, R[4] = {32, 0, 0, 0};        // L1 full, L4 silent, attack at rate 32
        EG e; e.start(L, R, 0, 0);
        ck("no hold at register 0x3F", e.holdLeft == 0 && e.stage == 1);
        ck("attack starts at the floor", fabs(e.cur - cal::EG_ATTACK_FLOOR) < 1e-9);
        for (int i = 0; i < (int)(rate_secs(32) * SR); i++) e.tick();
        ck("attack reaches its target", e.cur > -0.5);
        // A rise to a target part way up takes the same route: the chip aims at the top of the scale and
        // stops at the target, so it gets there in a fraction of the time an approach aimed at the target
        // itself would need. Level 70 is 21 dB down; the unit is there inside 80 ms at rate 32.
        int M[4] = {LEVTAB[70] >> 1, 0, 0, 63};
        EG m; m.start(M, R, 0, 0);
        int n = 0;
        while (m.stage == 1 && n < (int)(0.5 * SR)) { m.tick(); n++; }
        ck("a rise to a mid target aims at the top", n > (int)(0.04 * SR) && n < (int)(0.10 * SR));
        int H[4] = {63, 63, 63, 63};
        EG h; h.start(L, H, 40, 0);                             // hold 40 -> register 41
        double want = (rate_secs(41) * cal::EG_HOLD_FRAC + cal::EG_HOLD_LAG) * SR;
        ck("hold is half a traverse plus the lag", fabs(h.holdLeft - want) < 1.0);
    }

    // 2. a parameter request comes back as a parameter change with the same value
    { uint8_t m[8] = {0xF0, 0x43, 0x30, 0x5E, 0x10, 0, 0x11, 0xF7};
      S.outQ.clear(); apply_param_change_locked(S, m, 8);
      ck("parameter request replied", S.outQ.size() == 1);
      if (S.outQ.size() == 1) { auto& r = S.outQ[0];
          ck("reply is a parameter change", r.size() == 10 && r[2] == 0x10 && r[4] == 0x10 && r[6] == 0x11);
          ck("reply carries the value", (r[7] << 7 | r[8]) == 100); } }

    // 3. bulk dumps: request, check the checksum, feed it back, state unchanged
    uint8_t before[400]; S.current_perf_bytes(before);
    { uint8_t m[8] = {0xF0, 0x43, 0x20, 0x5E, 0x10, 0, 0, 0xF7};
      S.outQ.clear(); apply_param_change_locked(S, m, 8);
      ck("performance dump replied", S.outQ.size() == 1);
      if (S.outQ.size() == 1) {
          auto r = S.outQ[0];
          int n = r[4] << 7 | r[5];
          ck("byte count 400", n == 400);
          int sum = 0; for (size_t i = 4; i + 1 < r.size(); i++) sum += r[i];
          ck("checksum", (sum & 0x7F) == 0);
          for (int i = 0; i < 4; i++) S.perf.part[i].p[0x0E] = 1;          // scribble, then reload
          ck("bulk reloaded", load_sysex(S, nullptr, r.data(), r.size(), 0, 0));
          uint8_t after[400]; S.current_perf_bytes(after);
          ck("performance survived the round trip", memcmp(before, after, 400) == 0);
      } }
    { uint8_t m[8] = {0xF0, 0x43, 0x20, 0x5E, 0x40, 0, 0, 0xF7};
      S.outQ.clear(); apply_param_change_locked(S, m, 8);
      ck("voice dump replied", S.outQ.size() == 1 && (S.outQ[0][4] << 7 | S.outQ[0][5]) == 608); }
    { uint8_t m[8] = {0xF0, 0x43, 0x20, 0x5E, 0x00, 0, 0, 0xF7};
      S.outQ.clear(); apply_param_change_locked(S, m, 8);
      ck("system dump replied", S.outQ.size() == 1 && (S.outQ[0][4] << 7 | S.outQ[0][5]) == 76); }

    // 4. RPN / NRPN and bank select
    S.perf.part[0].p[4] = 0; S.forceChannel = 0;
    S.midi_in(0xB0, 101, 0); S.midi_in(0xB0, 100, 0); S.midi_in(0xB0, 6, 12);
    ck("RPN bend range", S.perf.part[0].p[0x26] == 0x4C && S.perf.part[0].p[0x27] == 0x34);
    S.midi_in(0xB0, 100, 2); S.midi_in(0xB0, 6, 0x28 + 12);
    ck("RPN note shift", S.perf.part[0].p[8] == 12);
    S.midi_in(0xB0, 99, 1); S.midi_in(0xB0, 98, 0x20); S.midi_in(0xB0, 6, 90);
    ck("NRPN filter cutoff", S.perf.part[0].p[0x18] == 90);
    S.midi_in(0xB0, 0, 0x3F); S.midi_in(0xB0, 32, 3);
    ck("bank select", S.perf.part[0].p[1] == 4);
    S.midi_in(0xB0, 126, 0); ck("mono mode", S.perf.part[0].p[5] == 0);
    S.midi_in(0xB0, 127, 0); ck("poly mode", S.perf.part[0].p[5] == 1);

    // 5. the controller matrix (FUN_00016F9C / F58 / 17032 and the source order in FUN_000191C0)
    ck("scaler f9c centre", Synth::scale_f9c(0, 0) == 0);
    ck("scaler f9c full", Synth::scale_f9c(127, 63) == 127);          // (128 * 64 * 2) >> 6 saturates
    ck("scaler f9c half", Synth::scale_f9c(64, 16) == 34);            // (65 * 17 * 2) >> 6
    ck("scaler f9c negative", Synth::scale_f9c(-128, 63) == -128);
    ck("scaler f9c inverted", Synth::scale_f9c(64, -16) == -33);      // a negative depth takes no bias
    ck("scaler f58 adds to the byte", Synth::scale_f58(64, 16, 64) == 64 + 17);
    ck("scaler f58 clamps low", Synth::scale_f58(-128, 63, 10) == 0);
    ck("scaler f58 clamps high", Synth::scale_f58(127, 63, 100) == 127);
    ck("scaler 17032 is f58 doubled less one", Synth::scale_f032(64, 16, 0) == 33);
    {
        Synth& T = S;
        memset(T.perf.c + 0x28, 0, 0x28);
        T.perf.c[0x28] = 0x0F;                                        // set 1 on for every part
        T.perf.c[0x30] = 0; T.perf.c[0x31] = 1 << Synth::SRC_KN1;     // source KN1
        T.perf.c[0x40] = 37;                                          // destination: voiced band width
        T.perf.c[0x48] = 64 + 32;                                     // depth +32
        memset(T.perf.part[0].src, 0, sizeof T.perf.part[0].src);
        ck("no source, no offset", T.ctrl_offset(0, 37) == 0);
        T.set_source(T.perf.part[0], Synth::SRC_KN1, 127);
        ck("KN1 is bipolar", T.perf.part[0].src[Synth::SRC_KN1] == 127);
        T.set_source(T.perf.part[0], Synth::SRC_KN1, 64);
        ck("KN1 centre is zero", T.perf.part[0].src[Synth::SRC_KN1] == 0);
        T.set_source(T.perf.part[0], Synth::SRC_KN1, 0);
        ck("KN1 bottom is -128", T.perf.part[0].src[Synth::SRC_KN1] == -128);
        T.set_source(T.perf.part[0], Synth::SRC_MW, 100);
        ck("the wheel keeps its raw value", T.perf.part[0].src[Synth::SRC_MW] == 100);
        T.set_source(T.perf.part[0], Synth::SRC_KN1, 127);
        ck("offset follows the source", T.ctrl_offset(0, 37) == Synth::scale_f9c(127, 32));
        ck("another destination stays clear", T.ctrl_offset(0, 38) == 0);
        T.perf.c[0x28] = 0x0E;                                        // part 1 switched out of the set
        ck("the part switch gates it", T.ctrl_offset(0, 37) == 0);
        T.perf.c[0x40] = 46;                                          // Fseq speed is performance-wide
        ck("a global destination ignores the part switch", T.ctrl_offset(0, 46) != 0);
        T.perf.c[0x28] = 0x0F; T.perf.c[0x40] = 21;                   // filter cutoff edits the part byte
        ck("a part-byte destination lands on the byte",
           T.ctrl_part(0, 21, 64) == Synth::scale_f58(127, 32, 64));
        // Two sets on one destination do not compose: each handler stores a per-part byte, so the
        // highest-numbered one wins (FUN_00014DDC walks 0 to 7, ctrlDest_37 writes DAT_01029758).
        T.perf.c[0x40] = 37; T.perf.c[0x29] = 0x0F;
        T.perf.c[0x32] = 0; T.perf.c[0x33] = 1 << Synth::SRC_KN1;
        T.perf.c[0x41] = 37; T.perf.c[0x49] = 64 + 16;                // set 2, same destination, depth +16
        ck("the higher set wins, it does not sum", T.ctrl_offset(0, 37) == Synth::scale_f9c(127, 16));
        // Frequency bias reaches formant and fixed operators only: the ratio branch of FUN_0001E838
        // adds nothing, which is what detuned the demo's ratio operators under a moving controller.
        T.perf.c[0x41] = 36; T.perf.c[0x49] = 64 + 16;                // set 2 -> frequency bias
        T.perf.c[0x40] = 0;
        Voice& V = T.perf.part[0].voice;
        V.v[0].form = 7; V.v[0].fbias = 7 + 4; V.v[1].form = 0; V.v[1].fixed = 0; V.v[1].fbias = 7 + 4;
        T.refresh_bias(T.ch[0], T.perf.part[0]);
        int want = (int16_t)(Synth::scale_f9c(127, 16) * 4 * 0x10) >> 2;
        ck("the formant operator takes the frequency bias", T.ch[0].fbW[0] == want && want != 0);
        ck("the ratio operator does not", T.ch[0].fbW[1] == 0);
        memset(T.perf.c + 0x28, 0, 0x28);
        init_default_voice(V);
    }

    // 5b. the detune law: the ROM's own key scaled table, against what 11_detune measured. The band is
    // (pitchNote >> 8) + 10 - 0x50, which advances four per octave, so note 60 is band 13.
    {
        auto detw = [](int byte, int band) {
            int dd = byte < 15 ? (~byte) : byte - 15, idx = (dd & 0x1F) * 32 + band;
            return (idx & 0x200) ? -(int)FRMDET[idx & 0x1FF] : (int)FRMDET[idx & 0x1FF];
        };
        ck("detune centre is none", detw(15, 13) == 0);
        ck("detune +7 at note 60 is 7 word units", detw(22, 13) == 7);      // 8.20 cents, measured 7.85
        ck("detune -7 at note 60 mirrors it", detw(8, 13) == -7);
        ck("detune +15 at note 60 is 22 units", detw(30, 13) == 22);        // 25.78 cents, measured 25.84
        ck("the same detune is smaller three octaves up", detw(30, 25) == 7 && detw(22, 25) == 2);
        ck("and larger three octaves down", detw(30, 1) == 37 && detw(22, 1) == 13);

        // And that it reaches the operator. Earlier sections leave note shifts scribbled, so the band
        // is read back off the channel rather than assumed: what is under test here is that the word
        // reaches the frequency at all, the table itself being checked above.
        Synth& T = S;
        Voice& V = T.perf.part[0].voice;
        init_default_voice(V);
        std::vector<float> a(64), b(64);
        auto play = [&](int det) {
            V.v[0].detune = det;
            T.midi_in(0x90, 60, 100);
            T.render(a.data(), b.data(), 64);
            Chan* c = nullptr;
            for (auto& ch : T.ch) if (ch.active && ch.note == 60) { c = &ch; break; }
            double f = c ? c->op[0].fop : 0.0;
            int bnd = c ? (c->pitchNote >> 8) + 10 : 0;
            bnd = bnd < 0x50 ? 0 : bnd < 0x70 ? ((bnd ^ 0x10) & 0x1F) : 0x1F;
            T.midi_in(0x80, 60, 0); T.midi_in(0xB0, 120, 0);
            return std::make_pair(f, bnd);
        };
        auto centred = play(15), up = play(22), down = play(8);
        ck("a centred operator has a frequency at all", centred.first > 1.0);
        double want = detw(22, up.second) * 1200.0 / 1024.0;
        ck("detune +7 moves the operator by its table entry",
           fabs(1200.0 * log2(up.first / centred.first) - want) < 0.01 && want > 0.0);
        ck("detune -7 moves it the other way by the same",
           fabs(1200.0 * log2(down.first / centred.first) + want) < 0.01);
        init_default_voice(V);
    }

    // 6. notes still sound and stop
    S.midi_in(0x90, 60, 100);
    int live = 0; for (auto& c : S.ch) if (c.active) live++;
    ck("note on allocated a channel", live > 0);
    std::vector<float> bl(4096), br(4096);
    S.render(bl.data(), br.data(), 4096);
    double pk = 0; for (auto v : bl) pk = std::max(pk, (double)fabs(v));
    ck("note produced audio", pk > 0.0001);
    S.midi_in(0x80, 60, 0); S.midi_in(0xB0, 120, 0);
    live = 0; for (auto& c : S.ch) if (c.active) live++;
    ck("all sound off", live == 0);

    // The summary belongs after the last check, not here: it used to print at this point, so every
    // check below it (the filter EG, the formant window, the DX7 conversion, the voice edit) printed its
    // failure and then reported "selftest: ok" above it, and the caller's exit code was the only place
    // the truth showed.
    // The filter EG as the firmware steps it (FUN_0000D050 / FUN_0000CB6C): every segment reaches its end
    // level exactly, a flat segment still ends (asymptote target + 3), the hold is a hold, and the release
    // returns to L4. Time per rate word is the chip's and unmeasured, so only the structure is checked.
    {
        StepEG e; double L[4] = {200, 100, 100, -100}; int R[4] = {20, 20, 50, 20};   // times, not rate words: 0 is the slowest
        e.start(L, R);
        int n = 0; while (e.stage == 0 && n < 100000) { e.tick(); n++; }
        ck("filter EG attack lands on L1", e.stage == 1 && e.cur == 200 && n > 1);
        n = 0; while (e.stage == 1 && n < 100000) { e.tick(); n++; }
        ck("filter EG decay lands on L2", e.stage == 2 && e.cur == 100);
        n = 0; while (e.stage == 2 && n < 100000) { e.tick(); n++; }
        ck("filter EG flat segment holds FEG_FLAT_S", e.stage == 9 && e.cur == 100 && fabs(n / TICK_HZ - cal::FEG_FLAT_S) < 0.01);
        for (int i = 0; i < 1000; i++) e.tick();
        ck("filter EG holds at L3", e.cur == 100);
        e.release(); n = 0; while (e.stage == 3 && n < 100000) { e.tick(); n++; }
        ck("filter EG release lands on L4", e.stage == 9 && e.cur == -100);
        // Named rather than compound literals: a (int[4]){...} temporary is a clang extension that
        // GCC rejects as taking the address of a temporary array and MSVC as C4576.
        const int rateSlow[4] = {60, 0, 0, 0}, rateFast[4] = {20, 0, 0, 0};
        StepEG slow, fast; slow.start(L, rateSlow); fast.start(L, rateFast);
        int ns = 0, nf = 0; while (slow.stage == 0 && ns < 10000000) { slow.tick(); ns++; }
        while (fast.stage == 0 && nf < 10000000) { fast.tick(); nf++; }
        ck("filter EG time 60 vs 20: 2^(40/15.5) = 6x per 15 words... 15x over 40 (flteg 0.81 vs 0.052 s)", ns > 12 * nf && ns < 18 * nf);
    }

    // The part's filter switch is live: a held note picks it up on the tick, as cutoff and resonance
    // already do. Ours, not the unit's, which latches it at note-on. docs/Differences.md.
    {
        Part& pt = S.perf.part[0]; pt.p[7] = 0;
        S.midi_in(0x90, 60, 100);
        std::vector<float> l(1024), r(1024);
        S.render(l.data(), r.data(), 1024);
        Chan* c = nullptr; for (auto& x : S.ch) if (x.active && x.note == 60) { c = &x; break; }
        ck("note on with the filter off leaves it off", c && !c->fltOn);
        pt.p[7] = 1; S.render(l.data(), r.data(), 1024);
        ck("filter switched on mid-note reaches the held note", c && c->fltOn);
        pt.p[7] = 0; S.render(l.data(), r.data(), 1024);
        ck("and switched off again mid-note", c && !c->fltOn);
        S.midi_in(0x80, 60, 0); S.midi_in(0xB0, 120, 0);
    }

    // A part whose voice bank is off receives nothing even with a receive channel (A011 Sho, parts 3 and 4).
    { S.perf.part[1].p[1] = 0; S.perf.part[1].p[4] = 0x10; ck("bank off silences the part", !S.part_listens(1, 0)); S.perf.part[1].p[1] = 2; ck("bank on hears it again", S.part_listens(1, 0)); }

    // DX7 conversion against the firmware's own: demo song 10 sends the unit's conversion of PrC voice 371
    // (DX-Acrd 4), edited afterwards on a few musical bytes. Every byte the demo left alone must match:
    // the common block outside LFO/PEG/category, the two unused operator slots, and on each used slot the
    // placement (byte 5), fixed/coarse/fine where unedited, detune, hold, time scaling and the sense bytes.
    { static const uint8_t demo[608] = {
    0x44, 0x58, 0x2D, 0x41, 0x63, 0x72, 0x64, 0x20, 0x34, 0x20, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x04, 0x13, 0x26, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x13, 0x00, 0x00, 0x00, 0x00, 0x24, 0x32,
    0x2F, 0x32, 0x32, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x32, 0x00,
    0x16, 0x00, 0x00, 0x00, 0x00, 0x4A, 0x40, 0x40, 0x40, 0x40, 0x13, 0x00, 0x00, 0x00, 0x00, 0x47, 0x40, 0x40, 0x40, 0x40, 0x00, 0x24, 0x0A, 0x55, 0x07, 0x00, 0x00, 0x45, 0x3C, 0x0C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x40, 0x32, 0x64, 0x4B, 0x4B, 0x1E, 0x1E, 0x1E, 0x63, 0x00, 0x00, 0x00, 0x18, 0x01, 0x00, 0x00, 0x38, 0x06, 0x14, 0x0F, 0x32, 0x32, 0x14, 0x14, 0x63, 0x63, 0x63, 0x00,
    0x00, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x38, 0x07, 0x07, 0x07, 0x18, 0x0E, 0x00, 0x00, 0x14, 0x07, 0x00, 0x32, 0x32, 0x14, 0x14, 0x00, 0x07,
    0x63, 0x63, 0x63, 0x00, 0x00, 0x14, 0x14, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x18, 0x01, 0x00, 0x00, 0x38, 0x07, 0x14, 0x0F, 0x32, 0x32, 0x14, 0x14, 0x63, 0x63, 0x63, 0x00, 0x00, 0x14,
    0x14, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x38, 0x07, 0x07, 0x07, 0x18, 0x0F, 0x00, 0x00, 0x14, 0x07, 0x00, 0x32, 0x32, 0x14, 0x14, 0x00, 0x07, 0x63, 0x63,
    0x63, 0x00, 0x00, 0x14, 0x14, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x58, 0x04, 0x00, 0x00, 0x38, 0x00, 0x00, 0x08, 0x32, 0x32, 0x00, 0x00, 0x5F, 0x5C, 0x5C, 0x00, 0x07, 0x4A, 0x54, 0x2A,
    0x00, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x07, 0x37, 0x0A, 0x18, 0x0F, 0x4B, 0x00, 0x14, 0x07, 0x00, 0x32, 0x32, 0x14, 0x14, 0x00, 0x07, 0x63, 0x63, 0x63, 0x00,
    0x00, 0x14, 0x14, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x58, 0x00, 0x00, 0x00, 0x38, 0x01, 0x00, 0x0B, 0x32, 0x32, 0x00, 0x00, 0x63, 0x60, 0x54, 0x00, 0x0E, 0x5C, 0x54, 0x34, 0x00, 0x01,
    0x4C, 0x27, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x39, 0x07, 0x07, 0x07, 0x18, 0x10, 0x00, 0x00, 0x14, 0x07, 0x00, 0x32, 0x32, 0x14, 0x14, 0x00, 0x07, 0x63, 0x63, 0x63, 0x00, 0x00, 0x14,
    0x14, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x58, 0x01, 0x00, 0x00, 0x38, 0x02, 0x00, 0x08, 0x32, 0x32, 0x00, 0x00, 0x63, 0x63, 0x63, 0x00, 0x1F, 0x4C, 0x56, 0x24, 0x00, 0x01, 0x63, 0x38,
    0x06, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x07, 0x3D, 0x0A, 0x18, 0x10, 0x2A, 0x00, 0x14, 0x07, 0x00, 0x32, 0x32, 0x14, 0x14, 0x00, 0x07, 0x63, 0x63, 0x63, 0x00, 0x00, 0x14, 0x14, 0x00,
    0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x58, 0x03, 0x00, 0x00, 0x38, 0x03, 0x00, 0x13, 0x32, 0x32, 0x00, 0x00, 0x63, 0x5C, 0x47, 0x00, 0x00, 0x4D, 0x52, 0x17, 0x00, 0x00, 0x44, 0x4B, 0x00, 0x32,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x07, 0x37, 0x0A, 0x18, 0x10, 0x4B, 0x00, 0x14, 0x07, 0x00, 0x32, 0x32, 0x14, 0x14, 0x00, 0x07, 0x63, 0x63, 0x63, 0x00, 0x00, 0x14, 0x14, 0x00, 0x00, 0x00,
    0x07, 0x07, 0x07, 0x07, 0x58, 0x00, 0x01, 0x00, 0x38, 0x04, 0x00, 0x12, 0x32, 0x32, 0x00, 0x00, 0x63, 0x5C, 0x47, 0x00, 0x00, 0x50, 0x55, 0x3B, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x39, 0x07, 0x37, 0x0A, 0x18, 0x10, 0x68, 0x00, 0x14, 0x07, 0x00, 0x32, 0x32, 0x14, 0x14, 0x00, 0x07, 0x63, 0x63, 0x63, 0x00, 0x00, 0x14, 0x14, 0x00, 0x00, 0x00, 0x07, 0x07,
    0x07, 0x07, 0x58, 0x01, 0x00, 0x00, 0x38, 0x05, 0x00, 0x13, 0x32, 0x32, 0x00, 0x00, 0x63, 0x5C, 0x52, 0x00, 0x20, 0x51, 0x56, 0x1F, 0x00, 0x01, 0x63, 0x37, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x39, 0x07, 0x3D, 0x0A, 0x18, 0x11, 0x00, 0x00, 0x14, 0x07, 0x00, 0x32, 0x32, 0x14, 0x14, 0x00, 0x07, 0x63, 0x63, 0x63, 0x00, 0x00, 0x14, 0x14, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07,
      };
      static const uint8_t vced[155] = {   // PrC 371 DX-Acrd 4, VCED order with the name last
    0x46, 0x0F, 0x05, 0x2F, 0x5F, 0x5C, 0x5C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x3E, 0x00, 0x04, 0x00, 0x00, 0x44, 0x00, 0x08, 0x27, 0x63, 0x60, 0x54, 0x00, 0x27, 0x00,
    0x00, 0x01, 0x03, 0x01, 0x00, 0x00, 0x4A, 0x00, 0x00, 0x00, 0x03, 0x3C, 0x14, 0x0A, 0x3C, 0x63, 0x63, 0x63, 0x00, 0x38, 0x06, 0x0A, 0x00, 0x00, 0x01, 0x03, 0x06, 0x63, 0x00, 0x01, 0x00,
    0x00, 0x57, 0x0F, 0x0A, 0x46, 0x63, 0x5C, 0x47, 0x00, 0x4B, 0x00, 0x32, 0x00, 0x00, 0x00, 0x03, 0x00, 0x42, 0x00, 0x03, 0x00, 0x0B, 0x5B, 0x0F, 0x0A, 0x23, 0x63, 0x5C, 0x47, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x4E, 0x00, 0x00, 0x01, 0x0A, 0x3B, 0x0F, 0x0A, 0x41, 0x63, 0x5C, 0x52, 0x00, 0x37, 0x00, 0x0A, 0x00, 0x00, 0x01, 0x03, 0x06, 0x63, 0x00, 0x01,
    0x00, 0x0B, 0x63, 0x62, 0x62, 0x62, 0x2F, 0x32, 0x32, 0x32, 0x03, 0x07, 0x01, 0x13, 0x26, 0x07, 0x00, 0x00, 0x04, 0x01, 0x24, 0x44, 0x58, 0x2D, 0x41, 0x63, 0x72, 0x64, 0x20, 0x34, 0x20,
      };
      uint8_t out[608]; convert_dx7(vced, out); int bad = 0;
      for (int i = 0; i < 608; i++) if (i != 0x0E && out[i] != demo[i]) bad++;   // 0x0E is the category, the demo's one edit
      ck("DX7 conversion matches the unit's own DX-Acrd 4 bulk on every byte but the category", bad == 0); }

    // DX7 conversion: the operator slot is DX7MAP word 25 + j, and every DX carrier lands on an FS1R carrier
    { uint8_t vced[155] = {}; vced[134] = 4;                        // DX algorithm 5: carriers 1, 3, 5
      for (int j = 0; j < 6; j++) vced[j * 21 + 16] = (uint8_t)(10 + j);   // output level tags op6..op1 as 10..15
      uint8_t out[608]; convert_dx7(vced, out); bool ok = true;
      const unsigned char* alg = FS1R_ALG[out[0x2C]];
      for (int j = 0; j < 6; j++) for (int o = 0; o < 8; o++) if (out[112 + o * 62 + 22] == 10 + j) {
          bool carrier = alg[2 * o + 1] & 1, want = (6 - j) == 1 || (6 - j) == 3 || (6 - j) == 5; if (carrier != want) ok = false; }
      ck("DX7 alg 5 carriers land on FS1R carriers (B025 Tremolo)", ok); }

    // The formant window follows the operator's own pitch mod sense, not the channel word at pms 7
    { S.perf.part[0].p[1] = 2; S.perf.part[0].p[4] = 0x10; Voice& V = S.perf.part[0].voice;
      V.v[0].form = 7; V.v[0].fixed = 1; V.v[0].pms = 1; V.pmd = 99; V.lfo1delay = 0;
      S.midi_in(0x90, 60, 100); float L[480], R[480]; double lo = 1e9, hi = 0;
      for (int k = 0; k < 400; k++) { S.render(L, R, 480); if (k < 200) continue; for (auto& c : S.ch) if (c.active && c.part == 0) { lo = std::min(lo, c.op[0].fw); hi = std::max(hi, c.op[0].fw); } }
      ck("formant window pitch mod goes through pms (B011 Human Eh)", hi > lo && 1200 * log2(hi / lo) < 200); S.all_off(); }

    // A voice edit reaches a sounding note: op 1 coarse 1 -> 2 moves its frequency word an octave.
    { S.perf.part[0].p[1] = 2; S.perf.part[0].p[4] = 0x10; S.perf.part[0].voice.v[0].fixed = 0; S.perf.part[0].voice.v[0].form = 0;
      S.midi_in(0x90, 60, 100); int w0 = -1; for (auto& c : S.ch) if (c.active && c.part == 0) w0 = c.freqWord[0];
      uint8_t m[10] = {0xF0, 0x43, 0x10, 0x5E, 0x60, 0x00, 0x01, 0x00, 2, 0xF7}; apply_param_change_locked(S, m, 10);
      int w1 = -1; for (auto& c : S.ch) if (c.active && c.part == 0) w1 = c.freqWord[0];
      ck("voice edit reaches the sounding note", w0 >= 0 && w1 == w0 + 1024); S.all_off(); }

    // The operator frequency EG is a linear ramp at a rate the amplitude EG's own ladder sets, and the
    // slope does not depend on the swing: that is what 07_modulation_2 measured and what separates it
    // from the exponential approach the engine used to have. Two swings at one attack time, the larger
    // taking proportionally longer, is the smallest thing that fails if the shape goes back.
    { init_perf(S.perf); Voice& V = S.perf.part[0].voice;
      S.perf.part[0].p[1] = 2; S.perf.part[0].p[4] = 0x10;
      auto ramp_ticks = [&](int init) {
          // attack level 0, so the ramp runs from feg_semis(init) to zero at the attack time
          V.v[0].fegInit = init; V.v[0].fegAtt = 0; V.v[0].fegAttT = 40; V.v[0].fegDecT = 40;
          S.all_off(); S.midi_in(0x90, 60, 100);
          Chan* C = nullptr; for (auto& c : S.ch) if (c.active && c.part == 0) C = &c;
          if (!C) return -1;
          double prev = C->op[0].feg.cur; int n = 0; double step = 0, worst = 0;
          // tick the EG directly: it runs per sample, and what is under test is its own shape. The last
          // step is short, since it lands exactly on the target rather than overshooting it, so the
          // constant-rate check covers every step but that one.
          for (int k = 0; k < 400000 && C->op[0].feg.stage == 0; k++) {
              double cur = C->op[0].feg.tick();
              double d = fabs(cur - prev);
              if (C->op[0].feg.stage != 0) break;            // the truncated final step
              if (n == 0) step = d; else worst = std::max(worst, fabs(d - step));
              prev = cur; n++;
          }
          ck("frequency EG ramps at a constant rate", n > 100 && worst < step * 1e-6);
          return n;
      };
      int n25 = ramp_ticks(25), n50 = ramp_ticks(50);
      double s25 = FreqEG::feg_semis(25), s50 = FreqEG::feg_semis(50);
      // the two swings are 10.5 and 47.6 semitones, so the sample counts must be in that ratio
      ck("frequency EG time is proportional to the swing",
         n25 > 0 && n50 > 0 && fabs((double)n50 / n25 - s50 / s25) < 0.02 * (s50 / s25));
      // and the rate is FEG_TRAVERSE semitones per traverse of the amplitude EG's rate word
      double want = cal::FEG_TRAVERSE / (rate_secs(egrate(40)) * SR);
      ck("frequency EG slope is FEG_TRAVERSE per traverse",
         n25 > 0 && fabs(s25 / n25 - want) < want * 0.01);
      S.all_off(); init_perf(S.perf); }

    // A note-on that takes a channel from a sounding note damps it rather than zeroing it
    // (FUN_00023000: EG stage word to 4 and register 0xFC/FD, the mask out of the one-hot table at
    // 0x35B260). Fill every channel with a sounding note, strike one more, and the displaced channel's
    // own output must carry on into the next sample instead of dropping to zero.
    { init_perf(S.perf); Voice& V = S.perf.part[0].voice;
      S.perf.part[0].p[1] = 2; S.perf.part[0].p[4] = 0x10;
      V.v[0].form = 0; V.v[0].level = 99; V.v[0].fixed = 0; V.v[0].coarse = 1; V.v[0].keysync = 1;
      for (int i = 0; i < 4; i++) { V.v[0].L[i] = i == 3 ? 0 : 99; V.v[0].T[i] = i == 3 ? 99 : 0; }
      S.all_off();
      float l[64], r[64];
      for (int k = 0; k < NCHAN; k++) { S.midi_in(0x90, 36 + k, 100); S.render(l, r, 8); }
      int live = 0; for (auto& c : S.ch) if (c.active) live++;
      ck("every channel is sounding before the steal", live == NCHAN);
      // the channel the allocator will actually take, found the same way the allocator picks it:
      // strike the note, then see which channel now holds it (age no longer decides this)
      double before[NCHAN];
      for (int i = 0; i < NCHAN; i++) before[i] = fabs(S.ch[i].lastL) + fabs(S.ch[i].lastR);
      S.midi_in(0x90, 100, 100);                       // one more note: the allocator has to steal
      Chan* victim = nullptr;
      for (auto& c : S.ch) if (c.active && c.note == 100) victim = &c;
      ck("the steal produced a channel holding the new note", victim != nullptr);
      if (victim) {
          int vi = int(victim - S.ch);
          ck("the victim channel was making sound", before[vi] > 1e-4);
          double damp = fabs(victim->dampL) + fabs(victim->dampR);
          ck("note-on damps the stolen channel rather than zeroing it", damp > 0.5 * before[vi]);
          double first = damp;
          for (int k = 0; k < 400; k++) S.render(l, r, 1);
          double later = fabs(victim->dampL) + fabs(victim->dampR);
          ck("the damp decays", later < 0.5 * first);
      }
      S.all_off(); init_perf(S.perf); }

    // The allocator, against the unit itself (FS1R.unlock captures/2026-09-25-7, session6 alloc).
    // Filling an empty machine with notes 36..67 on part 1 put them on channels 1..31 then 0, and a
    // 33rd note of 100 arriving on the full part took channel 2 (which held note 37). Both are
    // round-robin behaviour the old "lowest free, else oldest" got wrong.
    { init_perf(S.perf); Voice& V = S.perf.part[0].voice;
      V.v[0].form = 0; V.v[0].level = 99; V.v[0].fixed = 0; V.v[0].coarse = 1; V.v[0].keysync = 1;
      for (int i = 0; i < 4; i++) { V.v[0].L[i] = i == 3 ? 0 : 99; V.v[0].T[i] = i == 3 ? 60 : 0; }
      S.all_off(); S.nextChan = 0; for (int i = 0; i < 4; i++) S.partChan[i] = 0;
      float l[8], r[8];
      int order[NCHAN], n = 0;
      for (int note = 36; note < 36 + NCHAN; note++) {
          S.midi_in(0x90, note, 100); S.render(l, r, 4);
          int got = -1;                                    // the channel this note landed on
          for (int i = 0; i < NCHAN; i++) if (S.ch[i].active && S.ch[i].note == note) got = i;
          order[n++] = got;
      }
      bool rr = true;
      for (int i = 0; i < NCHAN; i++) rr = rr && order[i] == (i + 1) % NCHAN;
      ck("allocation is round robin: 32 notes land on channels 1..31 then 0", rr);
      // the 33rd, above every held note: the unit took channel 2, which held note 37
      int before = -1;
      for (int i = 0; i < NCHAN; i++) if (S.ch[i].note == 37) before = i;
      S.midi_in(0x90, 100, 100); S.render(l, r, 4);
      int took = -1;
      for (int i = 0; i < NCHAN; i++) if (S.ch[i].active && S.ch[i].note == 100) took = i;
      ck("the 33rd note steals the channel the unit stole (ch 2)", took == 2 && before == 2);
      S.all_off(); init_perf(S.perf); }

    // fsvr/display.h, the transcribed display tables, against values the Data List and the manual give.
    { using namespace display;
      ck("fixed frequency: 440.2 at coarse 16, fine 0", !strcmp(FIXED_FREQUENCY[0][15], "440.2"));
      ck("fixed frequency tops out at 28024 Hz", !strcmp(FIXED_FREQUENCY[127][20], "28024"));
      ck("ratio spans 0.500 to 61.69", ratio(0, 0) == 0.5 && fabs(ratio(31, 99) - 61.69) < 1e-9);
      const EffectType& hall = REVERB[1];
      const ValueTable& rt = VALUE_TABLES[hall.slot[0].table];
      ck("Hall1's Reverb Time at 0x50 starts at 2.0 s", hall.slot[0].lsb == 0x50 && !strcmp(rt.text[hall.slot[0].init - rt.sysexMin], "2.0"));
      const EffectType& echo = REVERB[15];
      const ValueTable& fb = VALUE_TABLES[echo.slot[1].table];
      ck("Echo's Lch FB Level starts at +22", !strcmp(fb.text[echo.slot[1].init - fb.sysexMin], "+22"));
      ck("Echo's Lch Delay1 starts at 220.0 ms (14-bit, 0.1 ms)", echo.slot[0].init == 2200 && VALUE_TABLES[echo.slot[0].table].count == 0); }

    printf(g_fails ? "selftest: %d FAILURES\n" : "selftest: ok\n", g_fails);
    return g_fails ? 1 : 0;
}

