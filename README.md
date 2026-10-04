# mpc-vst-fsvr

MPC/Force VST2 port of [FSVR](https://github.com/musicastudio/FSVR), the software
reconstruction of the Yamaha FS1R, for the Akai MPC OS plugin host (Force, MPC Live/One/X/Key).
Built for formant and vowel sounds: stabs, talking leads, choir pads, vocoded phrases.

`vst/` is the plugin port (see `vst/build.sh`), built the same way as `mpc-vst-euclidier`:
FSVR's engine (`src/`, see `src/VENDORED.md`) is compiled in-process, its factory banks (`data/`)
are embedded. No JUCE, no GUI: the screen is an MPC skin (`vst/layout.conf`).

| Tab | What it does |
|---|---|
| PERFORM | the 384 factory performances (knob + LOAD), RANDOM with a category filter (VOCAL, SYN FX, SEQUENCE ...), PANIC. Program change from the track loads a performance too (bank select LSB 0-2 = A/B/C). |
| VOWEL | MAKE builds a formant voice: VOWEL (A E I O U AE ER UH), VOICE (male, female, child, giant), SHAPE (stab, pluck, lead, pad), BREATH (noise), BODY (sine on the fundamental). RND VOWEL picks a vowel and varies formants, widths and levels around it. After MAKE every vowel knob changes the sound live. |
| FSEQ | one of the 90 preset formant sequences on part 1 (0 = off), RND FSEQ, SPEED 10-500 %. Played at the key's pitch, restarted by every note. Works on the vowel voice and on any voice with formant operators. |
| MUTATE | AMOUNT + MUTATE moves formants, widths, levels and EG times of part 1's voice; UNDO (16 steps, covers every load and change); SAVE writes `fsvr_user/FSVR_nnn.syx` next to the plugin, USER + USER LOAD bring it back. The files are FS1R bulk dumps, FSVR desktop opens them too. |
| SOUND | the FS1R's Formant and FM knobs (on the vowel voice: formants up/down, wider/narrower), filter cutoff/resonance, EG attack/decay/release. |
| MOD/FX | LFO speed, vibrato, reverb and variation send, volume, MONO, GLIDE. |
| SETUP | the CPU savers. OUTPUT (-6..+24 dB, default +12, soft knee above -3 dBFS). VOICES (ALL, 1-6; default 4): more held notes release the oldest, and the engine uses at most VOICES x parts channels, so release tails cannot pile up. BUFFER (OFF, 6, 12, 23 ms; default 12): the engine renders ahead on its own thread, on another CPU core; the MPC's audio thread only copies, and a slow block no longer crackles (the delay is reported to the host). PARTS (ALL, 1-3): only the first parts of a performance play - layered factory performances cost a quarter. EFFECTS ON/OFF. The plugin also sleeps after 1.5 s of silence with no note held and costs nothing until the next note. |
| VOCODER | 8-20 band vocoder with the FS1R as carrier. The modulator is a WAV from `fsvr_vox/` next to the plugin (`/sdcard/vst/fsvr_vox/` on the device; up to 99 files, sorted by name, 60 s each; 8/16/24/32-bit PCM or 32-bit float, mono or stereo, any sample rate). MODE: NOTE plays it once from each note, LOOP loops it from each note, BAR restarts it on every bar while the transport runs. START, SPEED 25-400 %, BAND SHIFT +-12 semitones, SIBILANCE (the WAV's highs straight through), MIX. Switching the vocoder ON reads the folder again. |

All MIDI from the track reaches every part (the engine listens on one forced channel), and the
engine's own MIDI implementation applies: CC 80 (Formant), CC 81 (FM), CC 74/71, CC 73/72,
CC 91/93, CC 7, CC 5/65, sustain, bend, aftertouch, RPN, NRPN 01 xx.

**CPU** (x86 benchmark, 4-note chord): A025 Everybody 15.9 % as loaded, 4.4 % with PARTS 1, 2.1 % with
PARTS 1 + EFFECTS OFF + VOICES 2; silent: 0 %. The plugin logs its DSP load every ~5 s to `/tmp/fsvr_vst.log` on the device
(`ssh root@<device> cat /tmp/fsvr_vst.log`). `vst/test.sh` benchmarks every factory performance
and the vowel voice on the CI machine (x86: vowel voice ~2 % of a core for one note, ~4 % for a
4-note chord, the 20-band vocoder another ~0.5 %; the Force's Cortex-A17 is several times slower).

Build/deploy workflow: see `sd88me/mpc-vst-plugins`' `docs/PORTING.md`.
License: GPL-3.0, as FSVR.

## Hinweis

Entwickelt mit Unterstützung von Claude (Anthropic)
