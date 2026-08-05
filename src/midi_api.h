// SuperOS-808 — MIDI IN/OUT plumbing for the web editor
// (implementation in midi.cpp; the editor lives in tools/web-editor/)
//
// SysEx envelope: F0 7D <cmd> ... F7 — the same manufacturer id the bootloader
// uses. Command spaces don't collide: the bootloader owns 0x01/0x02 (and only
// runs at boot), the app owns 0x10+.
//
// Patterns (228 bytes, see pattern.h), tracks (65 bytes) and the panel map
// travel 7-bit packed (bootloader scheme: 7 data bytes -> 1 MSB byte + 7 low-7
// bytes) with an XOR checksum of the raw bytes sent as two 7-bit values.
//
//   editor -> 808
//     F0 7D 10 <pat 0-31> F7                       request pattern
//     F0 7D 12 <pat> <xor_lo> <xor_hi> <packed> F7 push pattern
//     F0 7D 13 <pat> <inst> <step> <on> F7        set ONE step (live edit)
//     F0 7D 1A <n 1-16> <slot0..> F7               select a chain of slots
//     F0 7D 1D <slot 0-15> F7                      select slot
//     F0 7D 24 <trk 0-11> F7                       request track
//     F0 7D 26 <trk> <xor_lo> <xor_hi> <packed> F7 push track
//     F0 7D 30 F7                                  request settings
//     F0 7D 32 <chan 0-16> <clk 0-1> <out 0-1> F7  set settings
//     F0 7D 33 F7                                  request the panel decode table
//     F0 7D 3B F7                                  raw panel-matrix probe
//     F0 7D 3E <en> <lo7> <hi7> F7                 LED frame override (bench)
//     F0 7D 3F <dwell_us> F7                       LED column dwell (bench)
//     F0 7D 4D <fw 0|1> F7                         switch firmware (combined)
//     F0 7D 50 <xor_lo> <xor_hi> <packed map> F7   write the panel decode table
//     F0 7D 51 F7                                  begin guided calibration
//     F0 7D 60 F7                                  request device info
//   808 -> editor
//     F0 7D 11 <pat> <xor_lo> <xor_hi> <packed> F7 pattern dump
//     F0 7D 14 <status> F7                         push ack (0 = ok)
//     F0 7D 15 <pat> <step 0-63> F7                playhead, sent on every step
//     F0 7D 16 <pat> <inst 0-11> <step 0-63> <on> F7  panel step edit
//     F0 7D 18 <pat> <s0> <s1> <s2> <s3> <pre> F7  section lengths + prescale
//     F0 7D 1E <pat> F7                            selected pattern (stopped)
//     F0 7D 25 <trk> <xor_lo> <xor_hi> <packed> F7 track dump
//     F0 7D 31 <chan> <clk> <out> F7               settings dump
//     F0 7D 3C <pa0-3> <pb0-3> <status> <sw1b> F7  raw panel probe reply (10
//                                                  bytes, ONE NIBBLE EACH)
//     F0 7D 53 <xor_lo> <xor_hi> <packed map> F7   panel decode table dump
//     F0 7D 61 <maj> <min> <flags> F7              device info
//
// PANEL CALIBRATION. The rotary decode is data, not code (see
// docs/HARDWARE_MAP.md §7). 0x3B returns exactly what the matrix reads right
// now; the editor's calibration page walks the operator through each rotary
// position and button, captures the codes, and writes the finished table with
// 0x50. 0x51 puts the firmware into a calibration mode where it streams a 0x3C
// probe on every change, so the editor does not have to poll.
//
// MIDI-IN Note Ons whose note matches INSTRUMENT_NOTE fire that drum voice with
// a one-shot trigger pulse — this is how the editor auditions steps, and it
// makes the 808 playable from a DAW or pads. Velocity >= 100 asserts accent.
//
// Pattern selection by MIDI Program Change. PC 0-15 = variation A slots 1-16,
// PC 16-31 = variation B, so all 32 patterns are reachable from program
// changes alone; higher numbers repeat the pair. CC 32 (Bank Select LSB) also
// picks the variation (0 = A, 1 = B) and takes precedence once sent, for hosts
// that send it. Slots 13-16 are the INTRO/FILL IN bank, so a program change
// there arms the fill rather than switching the basic rhythm.
//
// Selection behaves exactly like a panel press, which means it is QUANTIZED:
// while running the new pattern takes over at the end of the measure (or
// immediately if the change lands on the downbeat), so a DAW firing program
// changes on the bar line switches cleanly.
//
// MIDI channel (Settings.midi_channel): 0 = omni, 1..16 = that channel only;
// gates Note On, Program Change and Bank Select. SysEx is channel-less.
#pragma once
#include <Arduino.h>

class Engine;

/// External MIDI clock-in state, filled by midi_rx_poll() each loop pass so
/// main.cpp can clock the sequencer from an external MIDI source.
struct MidiClockIn {
  uint8_t pulses    = 0;      ///< 0xF8 ticks received since the last poll
  bool    transport = false;  ///< running: between a MIDI Start/Continue and Stop
  bool    started   = false;  ///< a Start/Continue arrived this poll
  bool    stopped   = false;  ///< a Stop arrived this poll
};

/// Drain MIDI IN for this loop pass: SysEx, channel messages and System
/// Real-Time. Received clock is forwarded 1:1 to MIDI OUT here. Call near the
/// top of the loop, before transport/clock processing.
void midi_rx_poll(Engine &eng, MidiClockIn &mc);

/// Release the Start/Continue transport latch without a received Stop. loop()
/// calls this when the external clock has been silent past its timeout.
void midi_transport_clear();

/// Outgoing-side housekeeping: broadcast selection changes, service queued
/// dumps, pump the TX queue. Call once per loop pass, after transport/mode
/// processing.
void midi_tx_service(Engine &eng);

/// Queue one complete MIDI message for transmit. Messages go out atomically and
/// only as fast as the UART can take them, so queuing never blocks the ~1 ms
/// loop. Realtime bytes bypass the queue. Returns false if the queue is full.
bool midi_tx_msg(const uint8_t *msg, uint16_t len);

/// Broadcast the running playhead (0x15). Call on every step while running; the
/// editor draws its moving playhead straight from these.
void midi_send_step_position(uint8_t pat, uint8_t step);

/// Panel-edit broadcasts, so the editor mirrors writes made on the 808 live.
void midi_send_step_update(uint8_t pat, uint8_t inst, uint8_t step, bool on);
void midi_send_layout_update(uint8_t pat);

/// Queue a full pattern dump (0x11) — used after PATTERN CLEAR and the tools,
/// where per-step broadcasts would take dozens of messages.
void midi_send_pattern_dump(uint8_t pat);

/// True exactly once when pushed data is ready to persist. The caller must then
/// call save_dirty(eng); the request is cleared by this call.
bool midi_take_save_request(const Engine &eng);

/// Firmware-switch request from SysEx 0x4D (combined builds): returns the
/// requested firmware id once, then -1.
int8_t midi_take_fw_switch();

/// Queue a settings dump (0x31), sent on a 0x30 request and echoed after a
/// 0x32 set so the editor always sees the applied (sanitized) values.
void midi_send_settings();

/// True exactly once when a settings change pushed over SysEx is ready to
/// persist. The values are already live in g_settings.
bool midi_take_settings_save(const Engine &eng);

/// True exactly once when a panel-map change (0x50, or the end of a guided
/// calibration) is ready to persist. The table is already live in g_panel_map.
bool midi_take_panelmap_save(const Engine &eng);

/// Mirror the live scan into the probe/calibration path. main.cpp calls this
/// once per pass, right after ScanAndDisplay, so 0x3B and the calibration
/// stream report exactly what the matrix read.
void midi_set_probe(const uint8_t cell[4], uint8_t status);

/// Enter guided panel calibration: the firmware streams a 0x3C probe whenever
/// the raw matrix changes, so the editor can capture each detent without
/// polling. Leaves automatically when the editor writes the table with 0x50.
void midi_start_panel_calibration();
bool midi_calibration_active();
