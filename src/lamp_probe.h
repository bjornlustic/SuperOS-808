// lamp_probe.h -- measure the VARIATION lamp line ON THE MACHINE, in BOTH
// firmwares, and compare them.
//
// WHY THIS EXISTS. The player reports that the A lamp flickers while SuperOS
// plays and does not while the D650C emulator plays, on the same panel and the
// same board. I have now given two explanations for that and the machine has
// falsified both: claim-111's "restore the lamp with the data lines" reduced it
// but did not remove it, and claim-128's "loop jitter from host load" was
// disproved directly -- "the led is still flickering with or without usb
// traffic so that idea is false".
//
// Two wrong models is enough. This measures the pin.
//
// WHAT IT MEASURES. PE1 is the VARIATION lamp driver (HIGH = B) and the rim
// shot data line. Showing B, the line rests HIGH and every trigger without a
// rim shot pulls it LOW, which lights A. So the quantity that matters is: how
// long is it LOW, how often, and how much does that vary. Accumulated at the
// WRITE SITES, so it is exact rather than sampled -- a sampler would carry the
// jitter it is supposed to be measuring.
//
// WHY .noinit. Only one firmware runs at a time, and switching between them is
// a WATCHDOG RESET (combined_switch_firmware). .noinit survives that, the way
// g_reset_cause already does, so the emulator's figures are still there to read
// after switching back to SuperOS -- which is the only side that can answer a
// SysEx. That is what makes a headless A/B possible at all.
#pragma once
#include <Arduino.h>

#ifdef SUPEROS_LAMP_PROBE

// Three fields, not five. min_us and a span counter were here and are gone:
// this build had 653 bytes of headroom and the probe has to fit inside it. The
// host times the run with its own clock, and max plus the average is the shape
// that matters -- a short minimum tells you nothing you did not already know.
struct LampAcc {
  uint32_t low_us;    // total microseconds at the NOT-lamp level
  uint32_t dips;      // how many separate excursions
  uint32_t max_us;    // the longest one
};

// One per firmware, so a switch does not blend them.
extern LampAcc g_lamp_sos __attribute__((section(".noinit")));
extern LampAcc g_lamp_emu __attribute__((section(".noinit")));
extern uint32_t g_lamp_magic __attribute__((section(".noinit")));
static const uint32_t LAMP_MAGIC = 0x4C4D5031UL;   // "LMP1"

// DEFINED ONCE, in combined.cpp, and deliberately NOT inline: it is called from
// three sites across two firmwares and inlining it there cost 419 bytes, which
// is more headroom than this build has.
void lamp_probe_reset();
void lamp_probe_edge(LampAcc &a, uint8_t level, uint8_t wanted);


#else
struct LampAcc { uint32_t low_us, dips, max_us; };
#endif
