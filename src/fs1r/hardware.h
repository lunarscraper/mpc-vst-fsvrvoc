// fs1r/hardware.h - the FS1R's own numbers, read off the board or out of the firmware.
//
// KNOWN. Every value here is a fact about the hardware with a source behind it, not a choice and not
// a fit. Nothing in this file is a calibration knob: if one of these disagrees with a real unit the
// answer is that we read it wrong, not that it needs tuning. See STATUS.md, the KNOWN list.
//
// Quantities we have measured but not read are in fs1r/chips/cal.h. Quantities that are ours rather
// than the hardware's are in fsvr/tuning.h.
#pragma once

static const int SR = 48000;   // the hardware rate; fs1r::Device resamples to the host
static const int NCHAN = 32;
static const double PI = 3.14159265358979323846;
static const double CPU_HZ = 28000000.0;         // SCI BRR 27 gives exactly 31250 baud at 28 MHz
static const double TICK_HZ = CPU_HZ / 16.0 / 9099.0;   // MTU2 TGRA compare every 0x238B counts at clock/16 -> 192.3 Hz LFO/PEG/portamento tick
