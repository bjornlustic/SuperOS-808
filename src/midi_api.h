// SuperOS-808 — MIDI IN/OUT plumbing for the web editor
// (implementation in midi.cpp; the editor lives in tools/web-editor/)
//
// SysEx envelope: F0 7D <cmd> ... F7 — the same manufacturer id the bootloader
// uses. Command spaces don't collide: the bootloader owns 0x01/0x02 (and only
// runs at boot), the app owns 0x10+.
//
// Patterns (228 bytes, see pattern.h), tracks (65 bytes: a length byte whose
// bit 7 means "runs on into the next track", then 64 slots) and the panel map
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
//     F0 7D 4A F7                                  reboot into the SysEx bootloader
//     F0 7D 4D <fw 0|1> F7                         switch firmware (combined)
//     F0 7D 50 <xor_lo> <xor_hi> <packed map> F7   write the panel decode table
//     F0 7D 51 F7                                  begin guided calibration
//     F0 7D 60 F7                                  request device info
//     F0 7D 70 <idx 0-7> <xor_lo> <xor_hi> <packed> F7  stage a bootloader page
//     F0 7D 71 <crc0> <crc1> <crc2> F7             commit the staged bootloader
//     F0 7D 72 F7                                  request staging state
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
//     F0 7D 73 <status> <crc0> <crc1> <crc2> F7    bootloader-staging status
//
// BOOTLOADER REPLACEMENT (0x70/0x71/0x72). The resident bootloader cannot
// rewrite itself, so the app does it through the boot-section SPM service:
// pages land in a staging window first and are committed only against a CRC16
// the host computed over the same bytes. See boot_update.h for the safety
// model and the one window in which a power cut still costs an ISP recovery.
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

/// Bootloader-reboot request from SysEx 0x4A: returns true once. The caller
/// must flush pending saves, then park BOOT_MAGIC (0xB7) in GPIOR0 and jump to
/// 0x1F000 so the resident bootloader stays up for a USB/DIN SysEx update.
bool midi_take_boot_request();

/// Bootloader-replacement commit request from SysEx 0x71: returns true once and
/// hands back the CRC16 the host computed over the staged image. The caller must
/// flush pending saves, call bootupd_commit(crc), and report the result with
/// midi_send_bootupd_status(). See boot_update.h.
bool midi_take_bootupd_commit(uint16_t &want_crc);

/// Reply 0x73: a BOOTUPD_* status plus the CRC16 currently held in the staging
/// window, so the host can tell a failed upload from a failed commit.
void midi_send_bootupd_status(uint8_t status);

/// Register the USB-MIDI SysEx handler (no-op unless built with
/// SUPEROS_USB_MIDI). Call once from setup().
void midi_usb_init();

/// Queue a settings dump (0x31), sent on a 0x30 request and echoed after a
/// 0x32 set so the editor always sees the applied (sanitized) values.
/// Body is the settings.h codec, 5 values since 2026-09-07: channel,
/// clock_source, out_mode, legacy sysex_enable (a projection: 1 only when
/// SysEx is live in BOTH directions), io_mask (bit0 SysEx in, bit1 SysEx out,
/// bit2 notes in, bit3 notes out; 1 = enabled). Each field was APPENDED so
/// older 3- and 4-value decoders are unaffected. 0x32 mirrors it and accepts
/// 3, 4 or 5 values; a 4-value write sets both SysEx bits from the legacy
/// byte. Over the wire SysEx IN can only ever be switched OFF (a switched-off
/// IN drops all incoming SysEx, 0x32 included; re-enable is config menu step
/// 3 or a settings wipe).
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

#ifdef SUPEROS_TESTHOOKS
// ---------------------------------------------------------------------------
// Test hooks (GOAL.md G12) -- never in a shipping build
// ---------------------------------------------------------------------------
// G12 requires every feature to be drivable AND observable headless, and rule 7
// forbids handing the gap to a human. Two gaps on the 808, both measured in
// docs/gates/G12_808_DRIVE_OBSERVE.md:
//
//   DRIVE:   no SysEx reaches the panel layer, so the 16-tool CLEAR layer, every
//            rotary and every button (including the G4 edits "change length" and
//            "randomize") have no host route at all.
//   OBSERVE: the performance state the tools write lives in Engine, not in
//            Pattern, so the 0x11 dump cannot see it. A chain set with 0x1A can
//            never be read back.
//
// Both are compiled out unless SUPEROS_TESTHOOKS is defined (env:*-test). They
// cost RAM and they let a host forge panel state, neither of which belongs on a
// machine someone plays.

/// Overlay the injected panel cells onto one live scan. main.cpp calls this
/// immediately after ScanAndDisplay and before the debouncers, so an injected
/// press runs the real decode table, the real debounce and the real gesture
/// timer -- the point is to test the panel layer, not to bypass it.
/// Per byte: out = (scan & ~mask) | (val & mask). Mask-and-value, not OR,
/// because a rotary needs its PA nibble REPLACED (a rotary code of 0 is a
/// legitimate detent), while a key press only needs a bit raised.
/// Testhook bodies live in .testhook, which the test envs place at 0x17C60:
/// inside the natcode window, above the 0xD700 app ceiling and below the
/// 0x17E00 arena, which makesyx's own page rule already declares app-legal.
/// The window clears the arena by ~430 B, which holds the two BIG hook
/// bodies (the 0x4B parser and the 0x45 serializer, ~400 B together); the
/// small per-scan hooks stay in the app region. The price is natcode regen
/// clearance: any .natcode growth past 0x17C50 is now a LINK ERROR (overlap)
/// instead of a makesyx warning, which is stricter, not looser. noinline is
/// load-bearing: an inlined copy would land back in the caller's section.
#define TESTHOOK_CODE __attribute__((noinline, used, section(".testhook")))

void midi_panel_inject(uint8_t cell[4], uint8_t &status_hi);

/// Engine runtime state (0x44 request -> 0x45 dump). Defined in main.cpp
/// because the tool/menu UI state are its statics.
uint8_t superos_ui_state();
void    superos_blink_state(uint8_t *out);   // G10: 6 bytes, see main.cpp
#ifdef SUPEROS_PCINT_ATTRIB
void    superos_pcint_counts(uint8_t *out);  // 12 bytes, READ AND CLEARED
#endif
#endif
