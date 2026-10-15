// SuperOS-808 — global device settings (MIDI channel, clock source, OUT/THRU,
// the four MIDI I/O switches)
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
//
//   io_mask       Four independent switches, one bit each, 1 = enabled
//                 (config menu steps 3, 4, 13, 14; a lit key means OFF):
//                 IO_SYSEX_IN   every incoming SysEx is parsed for framing (so
//                               the byte stream stays in sync) and dropped,
//                               including 0x32 set-settings and 0x4A boot: the
//                               ONLY ways back on are the config menu (step 3)
//                               or a settings wipe. Deliberate, same rule as
//                               the 606: "off" that can be talked back on over
//                               the wire is not off.
//                 IO_SYSEX_OUT  no SysEx leaves the machine on either port.
//                               Replies to a still-live IN are swallowed too.
//                 IO_NOTES_IN   MIDI-IN Note On no longer triggers the voices.
//                               Program Change / Bank Select are NOT notes and
//                               keep working, as does clock/transport.
//                 IO_NOTES_OUT  the 808's own drum hits send no Note On/Off.
//                               Clock, transport and SysEx are untouched; this
//                               is narrower than OUT/THRU, which mutes all
//                               three.
//                 Until 2026-09-07 this was a single sysex_enable byte covering
//                 IN and OUT together. That byte is still carried on the wire
//                 and in the flash record as a PROJECTION of the two SysEx bits
//                 (see legacy_sysex), so a 606-era 0x31 decoder, and an older
//                 firmware reading a newer record, both still see a sane value.
#pragma once
#include <Arduino.h>

enum : uint8_t { CLK_SRC_MIDI = 0, CLK_SRC_INTERNAL = 1 };
enum : uint8_t { OUT_MODE_OUT = 0, OUT_MODE_THRU = 1 };

// io_mask bits. The order is the config-menu order (steps 3, 4, 13, 14) and
// the editor's; do not renumber, the mask is persisted as-is.
enum : uint8_t {
  IO_SYSEX_IN  = 0x01,
  IO_SYSEX_OUT = 0x02,
  IO_NOTES_IN  = 0x04,
  IO_NOTES_OUT = 0x08,
  IO_SYSEX     = IO_SYSEX_IN | IO_SYSEX_OUT,
  IO_ALL       = 0x0F,
};

struct Settings {
  uint8_t midi_channel = 0;             // 0 = omni, 1..16
  uint8_t clock_source = CLK_SRC_MIDI;  // CLK_SRC_*
  uint8_t out_mode     = OUT_MODE_OUT;  // OUT_MODE_*
  uint8_t io_mask      = IO_ALL;        // IO_* bits, 1 = enabled

  bool sysex_in()  const { return (io_mask & IO_SYSEX_IN)  != 0; }
  bool sysex_out() const { return (io_mask & IO_SYSEX_OUT) != 0; }
  bool notes_in()  const { return (io_mask & IO_NOTES_IN)  != 0; }
  bool notes_out() const { return (io_mask & IO_NOTES_OUT) != 0; }

  // The pre-2026-09-07 "sysex_enable" value: 1 only when the link is live in
  // BOTH directions, which is what that byte always meant. Anything narrower
  // projects to 0, the safe direction for a reader that only knows on/off.
  uint8_t legacy_sysex() const { return (io_mask & IO_SYSEX) == IO_SYSEX ? 1 : 0; }

  void sanitize() {
    if (midi_channel > 16)               midi_channel = 0;
    if (clock_source > CLK_SRC_INTERNAL) clock_source = CLK_SRC_MIDI;
    if (out_mode     > OUT_MODE_THRU)    out_mode     = OUT_MODE_OUT;
    io_mask &= IO_ALL;
  }
};

// Wire / record codec, shared by the FB_SETTINGS block (after its magic byte)
// and SysEx 0x31 / 0x32 (after the command byte), so there is exactly one
// layout: channel, clock_source, out_mode, legacy sysex_enable, io_mask. Each
// field was APPENDED in turn (the 606's 0x31 discipline), so a decoder that
// reads three or four values is unaffected, and settings_unpack accepts all
// three lengths:
//   3 values  pre-2026-08-28: io_mask untouched
//   4 values  the single sysex_enable switch: sets BOTH SysEx bits from it,
//             leaves the note bits alone
//   5 values  io_mask verbatim (sanitized to its four bits)
static constexpr uint8_t SETTINGS_VALUES = 5;

inline void settings_pack(const Settings &s, uint8_t *out) {
  out[0] = s.midi_channel;
  out[1] = s.clock_source;
  out[2] = s.out_mode;
  out[3] = s.legacy_sysex();
  out[4] = s.io_mask;
}

// Returns false, with s untouched, when fewer than three values arrived.
inline bool settings_unpack(Settings &s, const uint8_t *in, uint8_t n) {
  if (n < 3) return false;
  s.midi_channel = in[0];
  s.clock_source = in[1];
  s.out_mode     = in[2];
  if (n >= SETTINGS_VALUES) s.io_mask = in[4];
  else if (n == 4)          s.io_mask = in[3] ? (uint8_t)(s.io_mask | IO_SYSEX)
                                              : (uint8_t)(s.io_mask & ~IO_SYSEX);
  s.sanitize();
  return true;
}

// Single global instance, defined in main.cpp and read from midi.cpp.
extern Settings g_settings;
