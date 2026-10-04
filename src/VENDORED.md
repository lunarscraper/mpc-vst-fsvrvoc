# Vendored: FSVR engine

Source: [musicastudio/FSVR](https://github.com/musicastudio/FSVR), branch `main`, v0.5.1 era
(downloaded 2026-10-01): `src/fs1r.h`, `src/fs1r/`, `src/fsvr/` (the engine library `fs1rLib`).
`../data/` holds FSVR's `plugin/generated/fs1r_presets.syx`, `fs1r_performances.syx`,
`fs1r_fseqs.syx` and `fs1r_presets.csv`, the factory banks its own plugin embeds.
License: GPL-3.0 (see `../LICENSE`); the tables and factory data are Yamaha's, see `NOTICE.md`.

Not used: FSVR's plug-in layer (`plugin/`, `hollow/`, JUCE, clap-wrapper) and the console.

**One local change**, in `fsvr/fastmath.h` (`FSVR_MATH == 1`, the lookup-table backend), marked
`mpc-vst-fsvr`: `sin_turns()` and `exp2()` read one element past their tables when the input sits
a rounding error below an integer (found by AddressSanitizer in `vst/test.sh`, reached through the
variation effect's `tanh`). Both now clamp to the last table segment. Worth reporting upstream;
drop the change once FSVR has its own fix.

**Two more local changes for the MPC's CPU**, also marked `mpc-vst-fsvr`:
- `fsvr/device.cpp`: at a host rate other than 48 kHz the engine rendered ahead in blocks of 256
  samples, so one host block in two did all the work and the other none - a load spike that
  crackled on the Force. It now renders 32 at a time: the same samples, an even load.
- `fs1r.h`, `fsvr/device.cpp`, `fs1r/internal.h`, `fs1r/chips/ymp706.cpp`: `Device::setEffects(false)`
  skips the insertion, variation and reverb blocks (every part goes out dry, the master EQ stays).

Re-vendor by copying the same files from a newer FSVR commit and re-applying the fix if needed.
