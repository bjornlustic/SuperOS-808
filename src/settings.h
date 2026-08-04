// SuperOS-808 — global device settings (MIDI channel, clock source, OUT/THRU)
//
// One small struct of user-configurable globals, edited from the web editor over
// SysEx (0x30-0x32, see midi_api.h) and persisted to the FB_SETTINGS block. All
// apply to RAM the moment they change; the store write is deferred to the next
// stopped/idle moment.
//
//   midi_channel  0 = omni (act on every channel), 1..16 = that channel only.
//                 Gates MIDI-IN Note On (drum triggers), Program Change and Bank
//                 Select (pattern selection). The editor SysEx link is
//                 channel-less and always works.
//
//   clock_source  MIDI     = slave to an incoming MIDI clock whenever one is
//                            present (auto, with internal fallback when it
//                            stops); the received clock is forwarded to OUT.
//                 INTERNAL = ignore MIDI clock + transport entirely; always run
//                            from the 808's own TEMPO knob / rear sync jack,
//                            and clock MIDI OUT as the master.
//
//   out_mode      OUT  = MIDI OUT carries the 808's own play (a note per drum
//                        hit, transport, clock when master) plus editor SysEx.
//                 THRU = MIDI OUT mirrors MIDI IN performance data as a soft
//                        thru; the 808's own note/clock/transport output is
//                        muted. Editor SysEx still flows.
#pragma once
#include <Arduino.h>

enum : uint8_t { CLK_SRC_MIDI = 0, CLK_SRC_INTERNAL = 1 };
enum : uint8_t { OUT_MODE_OUT = 0, OUT_MODE_THRU = 1 };

struct Settings {
  uint8_t midi_channel = 0;             // 0 = omni, 1..16
  uint8_t clock_source = CLK_SRC_MIDI;  // CLK_SRC_*
  uint8_t out_mode     = OUT_MODE_OUT;  // OUT_MODE_*

  void sanitize() {
    if (midi_channel > 16)               midi_channel = 0;
    if (clock_source > CLK_SRC_INTERNAL) clock_source = CLK_SRC_MIDI;
    if (out_mode     > OUT_MODE_THRU)    out_mode     = OUT_MODE_OUT;
  }
};

// Single global instance, defined in main.cpp and read from midi.cpp.
extern Settings g_settings;
