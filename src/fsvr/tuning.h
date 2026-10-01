// fsvr/tuning.h - our own cost knobs. Nothing here is a fact about the FS1R.
//
// These are choices FSVR makes about how to spend CPU and memory. The hardware has no counterpart to
// any of them, so none can be calibrated against a recording and none has a right answer to discover.
//
// The rule that makes this file worth having: **changing a value here must not change the output.**
// If it does, the value is too coarse and the number is wrong, not the hardware's. Tune these when
// the engine is too slow. Never tune them to move a measurement, and never treat a disagreement with
// a real unit as evidence about one of them.
//
// Facts read off the board are in fs1r/hardware.h. Quantities measured off a real unit, which are
// claims about the hardware and are changed only by measuring, are in fs1r/chips/cal.h.
#pragma once

namespace tuning {

// How often the per-operator frequency and level maths is recomputed, in samples. Its inputs only
// move on the 192.3 Hz tick, which is one step per 250 samples at 48 kHz, so anything at or below
// that is inaudible and 16 leaves plenty of margin. This exists because a dozen pow() calls per
// operator per sample was the engine's largest single cost.
static const int CTL_DECIMATION = 16;

// How many sysex messages the engine will queue for the host before dropping them. The FS1R answers
// dump and parameter requests, and a host that never drains the queue must not grow it without
// bound. Deep enough for a full parameter-request burst.
static const int MIDI_OUT_QUEUE = 64;

}  // namespace tuning
