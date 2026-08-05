// SuperOS-808 — MIDI IN/OUT implementation. See midi_api.h for the SysEx map.

#include <Arduino.h>
#include <string.h>

#include "midi_api.h"
#include "pattern.h"
#include "engine.h"
#include "settings.h"
#include "controls.h"
#include "hw.h"
#ifdef SUPEROS_COMBINED
#include "combined.h"
#endif

extern Settings g_settings;
extern PanelMap g_panel_map;

// Bench knobs read by main.cpp / hw.h.
uint8_t  g_led_dwell_us      = 25;
uint16_t g_frame_override    = 0;
uint8_t  g_frame_override_en = 0;

static constexpr uint8_t MFR_ID = 0x7D;

// ---------------------------------------------------------------------------
// 7-bit packing (the bootloader's scheme: 1 MSB byte per 7 data bytes)
// ---------------------------------------------------------------------------
static uint16_t pack7(const uint8_t *src, uint16_t n, uint8_t *dst) {
  uint16_t o = 0;
  for (uint16_t i = 0; i < n; i += 7) {
    const uint8_t run = (uint8_t)((n - i) > 7 ? 7 : (n - i));
    uint8_t msb = 0;
    for (uint8_t b = 0; b < run; ++b) if (src[i + b] & 0x80) msb |= (uint8_t)(1 << b);
    dst[o++] = msb;
    for (uint8_t b = 0; b < run; ++b) dst[o++] = (uint8_t)(src[i + b] & 0x7F);
  }
  return o;
}

// Returns the number of raw bytes recovered (capped at cap).
static uint16_t unpack7(const uint8_t *src, uint16_t n, uint8_t *dst, uint16_t cap) {
  uint16_t i = 0, o = 0;
  while (i < n && o < cap) {
    const uint8_t msb = src[i++];
    for (uint8_t b = 0; b < 7 && i < n && o < cap; ++b) {
      dst[o++] = (uint8_t)(src[i++] | (((msb >> b) & 1) << 7));
    }
  }
  return o;
}

static uint8_t xor_sum(const uint8_t *p, uint16_t n) {
  uint8_t x = 0;
  for (uint16_t i = 0; i < n; ++i) x ^= p[i];
  return x;
}

// ---------------------------------------------------------------------------
// Shared buffer arena
// ---------------------------------------------------------------------------
// SuperOS's MIDI buffers and the emulator's 2 KB program-image copy are never
// live at the same time — combined.cpp picks one firmware per boot and the other
// never runs. Giving them separate statics costs 3.2 KB of an 8 KB part, which
// does not fit; overlaying them in one arena costs the larger of the two.
//
// Layout (SuperOS side): [ TX queue | out scratch | RX buffer ]
// The emulator claims the whole thing as its program image (see emu_avr.cpp).
static constexpr uint16_t TXQ_SIZE = 640;
static constexpr uint16_t OUT_SIZE = 300;
static constexpr uint16_t RX_SIZE  = 300;
#ifdef SUPEROS_COMBINED
static constexpr uint16_t ARENA_SIZE = 2048;      // the emulator's image copy
static_assert(ARENA_SIZE >= TXQ_SIZE + OUT_SIZE + RX_SIZE,
              "shared arena too small for the MIDI buffers");
#else
static constexpr uint16_t ARENA_SIZE = TXQ_SIZE + OUT_SIZE + RX_SIZE;
#endif
uint8_t g_shared_arena[ARENA_SIZE];

// A pattern dump is the largest message: 228 raw bytes pack to 261, plus the
// 6-byte envelope = 267. The queue holds one of those plus room for the short
// panel-edit broadcasts that can pile up behind it. Messages are enqueued whole
// and drained a few bytes per loop pass, so a dump can never block the ~1 ms
// heartbeat and can never be split by a note or a settings echo.
static uint8_t *const s_txq = g_shared_arena;
static uint16_t s_tx_head = 0, s_tx_tail = 0;

static uint16_t txq_free() {
  return (uint16_t)(TXQ_SIZE - 1 - ((s_tx_head - s_tx_tail) % TXQ_SIZE));
}

bool midi_tx_msg(const uint8_t *msg, uint16_t len) {
  if (len > txq_free()) return false;
  for (uint16_t i = 0; i < len; ++i) {
    s_txq[s_tx_head] = msg[i];
    s_tx_head = (uint16_t)((s_tx_head + 1) % TXQ_SIZE);
  }
  return true;
}

// Drain as much as the UART will take without blocking.
static void txq_pump() {
  while (s_tx_tail != s_tx_head && Serial1.availableForWrite() > 0) {
    Serial1.write(s_txq[s_tx_tail]);
    s_tx_tail = (uint16_t)((s_tx_tail + 1) % TXQ_SIZE);
  }
}

// Scratch for building outgoing SysEx. Used only from the main-loop-driven
// senders below (never from an ISR).
static uint8_t *const s_out = g_shared_arena + TXQ_SIZE;

static void send_short(uint8_t cmd, const uint8_t *body, uint8_t n) {
  s_out[0] = 0xF0; s_out[1] = MFR_ID; s_out[2] = cmd;
  for (uint8_t i = 0; i < n; ++i) s_out[3 + i] = (uint8_t)(body[i] & 0x7F);
  s_out[3 + n] = 0xF7;
  midi_tx_msg(s_out, (uint16_t)(4 + n));
}

// Envelope + xor + 7-bit packed payload, the shape every bulk dump uses.
static void send_packed(uint8_t cmd, uint8_t id, const uint8_t *raw, uint16_t n) {
  const uint8_t x = xor_sum(raw, n);
  uint16_t o = 0;
  s_out[o++] = 0xF0; s_out[o++] = MFR_ID; s_out[o++] = cmd;
  s_out[o++] = (uint8_t)(id & 0x7F);
  s_out[o++] = (uint8_t)(x & 0x7F);
  s_out[o++] = (uint8_t)((x >> 7) & 0x01);
  o += pack7(raw, n, s_out + o);
  s_out[o++] = 0xF7;
  midi_tx_msg(s_out, o);
}

// ---------------------------------------------------------------------------
// Deferred work flags
// ---------------------------------------------------------------------------
static bool     s_save_req      = false;
static bool     s_settings_req  = false;
static bool     s_panelmap_req  = false;
static uint32_t s_last_rx_ms    = 0;
static int8_t   s_fw_switch     = -1;
static bool     s_calibrating   = false;
static bool     s_probe_once    = false;   // 0x3B: answer in the service pass,
                                           // where the live scan is available

// Queued dumps: the editor can ask for everything at once, and each dump is
// large, so they are serviced one per loop pass rather than all at once.
static uint32_t s_pat_req_mask_lo = 0;   // patterns 0..31, bit per pattern
static uint16_t s_trk_req_mask    = 0;   // tracks 0..11
static bool     s_settings_dump   = false;
static bool     s_panelmap_dump   = false;
static bool     s_info_dump       = false;

// Selection broadcast (0x1E) de-dup.
static uint8_t s_last_sent_pat = 0xFF;
static bool    s_last_running  = false;

// Raw panel probe (0x3C), ONE NIBBLE PER BYTE:
//   [0..3] PA0-3 read while scan column 0..3 is selected  (rotaries + buttons)
//   [4..7] PB0-3 read while scan column 0..3 is selected  (step switches)
//   [8]    status nibble, all selects idle (tempo clock / start-stop / tap / fill)
//   [9]    hw::ModeProbe() — SW1b seen through the PC bus, 0 on an unmodded board
//
// The nibble-per-byte layout is not cosmetic. SysEx data bytes are 7-bit, and
// send_short masks every body byte with 0x7F. The earlier layout sent a whole
// cell as (PA << 4) | PB, which put PA3 in bit 7 — so PA3 was stripped out of
// all four columns on the wire, in every capture, no matter what the panel did.
// That is what made CLEAR, BASIC VARIATION "AB", I/F VARIATION "B" and
// INSTRUMENT codes >= 8 look dead while the firmware's own decode
// (controls.h pa_nib) read them normally. Status survived only because it is a
// nibble already. Keep every byte here a nibble and the 7-bit ceiling cannot be
// reached.
static constexpr uint8_t PROBE_LEN = 10;

// Raw-probe de-dup while calibrating: the last payload actually sent.
static uint8_t s_last_probe[PROBE_LEN] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

void midi_start_panel_calibration() {
  s_calibrating = true;
  memset(s_last_probe, 0xFF, sizeof(s_last_probe));
}
bool midi_calibration_active() { return s_calibrating; }

// ---------------------------------------------------------------------------
// Outgoing helpers used by main.cpp
// ---------------------------------------------------------------------------
// Running playhead. Sent on EVERY step while running, not just at the measure
// start, so the editor can draw a moving playhead instead of guessing between
// anchors. Five bytes per step: 20 B/s at 60 BPM 16ths, 100 B/s at 300 — small
// against the 3125 B/s the DIN port carries.
void midi_send_step_position(uint8_t pat, uint8_t step) {
  const uint8_t b[2] = { pat, step };
  send_short(0x15, b, 2);
}

void midi_send_step_update(uint8_t pat, uint8_t inst, uint8_t step, bool on) {
  const uint8_t b[4] = { pat, inst, step, (uint8_t)(on ? 1 : 0) };
  send_short(0x16, b, 4);
}

void midi_send_settings() { s_settings_dump = true; }

void midi_send_pattern_dump(uint8_t pat) {
  if (pat < NUM_PATTERNS) s_pat_req_mask_lo |= (uint32_t)1 << pat;
}

// Declared here so main.cpp can call it; needs the engine, so it queues the
// pattern id and the service pass reads the layout out of the store.
static uint8_t s_layout_req = 0xFF;
void midi_send_layout_update(uint8_t pat) { s_layout_req = pat; }

bool midi_take_save_request(const Engine &eng) {
  if (!s_save_req || eng.running) return false;
  if ((uint32_t)(millis() - s_last_rx_ms) < 250) return false;   // still mid-transfer
  if (s_tx_tail != s_tx_head) return false;                      // TX still draining
  s_save_req = false;
  return true;
}
bool midi_take_settings_save(const Engine &eng) {
  if (!s_settings_req || eng.running) return false;
  s_settings_req = false;
  return true;
}
bool midi_take_panelmap_save(const Engine &eng) {
  if (!s_panelmap_req || eng.running) return false;
  s_panelmap_req = false;
  return true;
}
int8_t midi_take_fw_switch() {
  const int8_t v = s_fw_switch;
  s_fw_switch = -1;
  return v;
}

// ---------------------------------------------------------------------------
// RX
// ---------------------------------------------------------------------------
// The largest inbound message is a pattern push: 6-byte envelope + 261 packed.
static constexpr uint16_t RXBUF = RX_SIZE;
static uint8_t *const s_rx = g_shared_arena + TXQ_SIZE + OUT_SIZE;
static uint16_t s_rx_len = 0;
static bool     s_in_sysex = false;

static uint8_t s_run_status = 0;
static uint8_t s_ch_buf[2];
static uint8_t s_ch_n = 0;
// 0xFF until a Bank Select LSB actually arrives, so "no bank select sent" is
// distinguishable from "bank 0" and the program number can pick the variation
// on its own. It used to default to 0, which read as a standing "variation A".
static uint8_t s_bank_lsb = 0xFF;

static bool s_transport_latch = false;
void midi_transport_clear() { s_transport_latch = false; }

static bool channel_ok(uint8_t status) {
  if (g_settings.midi_channel == 0) return true;
  return (uint8_t)((status & 0x0F) + 1) == g_settings.midi_channel;
}

static void handle_sysex(Engine &eng) {
  if (s_rx_len < 2 || s_rx[0] != MFR_ID) return;
  const uint8_t cmd = s_rx[1];
  const uint8_t *a  = s_rx + 2;               // arguments
  const uint16_t an = (uint16_t)(s_rx_len - 2);

  switch (cmd) {
    case 0x10:                                 // request pattern
      if (an >= 1 && a[0] < NUM_PATTERNS) s_pat_req_mask_lo |= (uint32_t)1 << a[0];
      break;

    case 0x12: {                               // push pattern
      if (an < 4) break;
      const uint8_t pat = a[0];
      if (pat >= NUM_PATTERNS) break;
      const uint8_t want = (uint8_t)((a[1] & 0x7F) | ((a[2] & 1) << 7));
      uint8_t raw[PATTERN_BYTES];
      const uint16_t got = unpack7(a + 3, (uint16_t)(an - 3), raw, PATTERN_BYTES);
      uint8_t st = 0;
      if (got != PATTERN_BYTES)          st = 1;
      else if (xor_sum(raw, got) != want) st = 2;
      if (!st) {
        Pattern p;
        deserialize_pattern(p, raw);
        eng.WritePattern(pat, p);
        s_save_req = true;
      }
      send_short(0x14, &st, 1);
      break;
    }

    case 0x13: {                               // set ONE step (live editor edit)
      // The counterpart of the 0x16 the panel sends when a step key is pressed,
      // so an edit made on either side lands on the other immediately.
      if (an < 4) break;
      const uint8_t pat  = a[0];
      const uint8_t inst = a[1];
      const uint8_t s    = a[2];
      if (pat >= NUM_PATTERNS || inst >= NUM_INSTRUMENTS || s >= MAX_STEPS) break;
      eng.SetPatternStep(pat, inst, s, (a[3] & 1) != 0);
      s_save_req = true;
      break;
    }

    case 0x1A: {                               // select a chain of slots
      if (an < 2) break;
      uint8_t n = a[0];
      if (n > CHAIN_MAX) n = CHAIN_MAX;
      if (an < (uint16_t)(1 + n)) break;
      uint8_t slots[CHAIN_MAX];
      for (uint8_t i = 0; i < n; ++i) slots[i] = (uint8_t)(a[1 + i] & 15);
      eng.SelectChain(slots, n);
      break;
    }

    case 0x1D:                                 // select slot
      if (an >= 1) eng.SelectSlot((uint8_t)(a[0] & 15));
      break;

    case 0x24:                                 // request track
      if (an >= 1 && a[0] < NUM_TRACKS) s_trk_req_mask |= (uint16_t)1 << a[0];
      break;

    case 0x26: {                               // push track
      if (an < 4) break;
      const uint8_t trk = a[0];
      if (trk >= NUM_TRACKS) break;
      const uint8_t want = (uint8_t)((a[1] & 0x7F) | ((a[2] & 1) << 7));
      uint8_t raw[TRACK_BYTES];
      const uint16_t got = unpack7(a + 3, (uint16_t)(an - 3), raw, TRACK_BYTES);
      uint8_t st = 0;
      if (got != TRACK_BYTES)             st = 1;
      else if (xor_sum(raw, got) != want) st = 2;
      if (!st) {
        deserialize_track(eng.track[trk], raw);
        eng.mark_trk_dirty(trk);
        s_save_req = true;
      }
      send_short(0x14, &st, 1);
      break;
    }

    case 0x30: s_settings_dump = true; break;

    case 0x33: s_panelmap_dump = true; break;   // request the panel decode table

    case 0x32:                                 // set settings
      if (an >= 3) {
        g_settings.midi_channel = a[0];
        g_settings.clock_source = a[1];
        g_settings.out_mode     = a[2];
        g_settings.sanitize();
        s_settings_req  = true;
        s_settings_dump = true;
      }
      break;


    case 0x3B:                                 // raw panel probe (one shot)
      s_probe_once = true;
      break;

    case 0x3E:                                 // LED frame override (bench)
      if (an >= 3) {
        g_frame_override_en = a[0];
        g_frame_override    = (uint16_t)((a[1] & 0x7F) | ((uint16_t)(a[2] & 0x7F) << 7));
      }
      break;

    case 0x3F:                                 // LED column dwell (bench)
      if (an >= 1) g_led_dwell_us = a[0];
      break;

#ifdef SUPEROS_COMBINED
    case 0x4D:                                 // switch firmware
      if (an >= 1) s_fw_switch = (int8_t)a[0];
      break;
#endif

    case 0x50: {                               // write the panel decode table
      if (an < 3) break;
      const uint8_t want = (uint8_t)((a[0] & 0x7F) | ((a[1] & 1) << 7));
      uint8_t raw[PANELMAP_BYTES];
      const uint16_t got = unpack7(a + 2, (uint16_t)(an - 2), raw, PANELMAP_BYTES);
      uint8_t st = 0;
      if (got != PANELMAP_BYTES)          st = 1;
      else if (xor_sum(raw, got) != want) st = 2;
      if (!st) {
        memcpy(&g_panel_map, raw, PANELMAP_BYTES);
        g_panel_map.magic = PANELMAP_MAGIC;
        g_panel_map.Sanitize();
        s_panelmap_req = true;
        s_calibrating  = false;      // the table just landed; stop streaming
        s_panelmap_dump = true;
      }
      send_short(0x14, &st, 1);
      break;
    }

    case 0x51:                                 // begin guided calibration
      midi_start_panel_calibration();
      break;

    // 0x52 (indicator-LED pin remap) is retired: SuperOS no longer drives those
    // lamps at all, because their bus lines ARE instrument-data bits and holding
    // one high arms that voice. See the note in engine.h.

    case 0x60: s_info_dump = true; break;

    default: break;
  }
}

static void rx_byte(Engine &eng, uint8_t b, MidiClockIn &mc) {
  // System Real-Time can interleave anywhere, including inside a SysEx.
  if (b >= 0xF8) {
    switch (b) {
      case 0xF8: mc.pulses++; if (g_settings.out_mode == OUT_MODE_OUT &&
                                  Serial1.availableForWrite() > 0) Serial1.write(b); break;
      case 0xFA: s_transport_latch = true;  mc.started = true; break;
      case 0xFB: s_transport_latch = true;  mc.started = true; break;
      case 0xFC: s_transport_latch = false; mc.stopped = true; break;
      default: break;
    }
    mc.transport = s_transport_latch;
    return;
  }

  if (b == 0xF0) { s_in_sysex = true; s_rx_len = 0; return; }
  if (b == 0xF7) {
    if (s_in_sysex) handle_sysex(eng);
    s_in_sysex = false;
    s_last_rx_ms = millis();
    return;
  }
  if (s_in_sysex) {
    if (s_rx_len < RXBUF) s_rx[s_rx_len++] = b;
    return;
  }

  // Channel-voice messages.
  if (b & 0x80) {
    s_run_status = b;
    s_ch_n = 0;
    return;
  }
  if (!s_run_status) return;
  s_ch_buf[s_ch_n++] = b;

  const uint8_t type = (uint8_t)(s_run_status & 0xF0);
  const uint8_t need = (type == 0xC0 || type == 0xD0) ? 1 : 2;
  if (s_ch_n < need) return;
  s_ch_n = 0;

  if (!channel_ok(s_run_status)) return;

  if (type == 0x90 && s_ch_buf[1] > 0) {              // Note On
    for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
      if (INSTRUMENT_NOTE[i] == s_ch_buf[0]) {
        eng.TriggerNow((uint16_t)1 << i, s_ch_buf[1] >= 100);
        break;
      }
  } else if (type == 0xB0) {                          // Control Change
    if (s_ch_buf[0] == 32) s_bank_lsb = s_ch_buf[1];  // Bank Select LSB = variation
  } else if (type == 0xC0) {                          // Program Change
    // All 32 patterns from Program Change alone: PC 0-15 = variation A slots
    // 1-16, PC 16-31 = variation B. Bank Select LSB still picks the variation
    // for hosts that send it and takes precedence, but it is awkward to send
    // from a DAW arrangement and the low bit of the program number is not, so
    // the second bank is reachable either way. Higher program numbers repeat
    // the pair rather than being ignored.
    const uint8_t pc  = (uint8_t)(s_ch_buf[0] & 0x7F);
    const uint8_t var = (s_bank_lsb <= 1) ? s_bank_lsb : (uint8_t)((pc >> 4) & 1);
    eng.SetVariation(var ? VAR_B : VAR_A);
    eng.SelectSlot((uint8_t)(pc & 15));
  }
}

void midi_rx_poll(Engine &eng, MidiClockIn &mc) {
  uint8_t guard = 64;      // bound the work per pass so a burst can't stall the loop
  while (Serial1.available() > 0 && guard--) rx_byte(eng, Serial1.read(), mc);
  mc.transport = s_transport_latch;
}

// ---------------------------------------------------------------------------
// Service pass
// ---------------------------------------------------------------------------
// main.cpp owns the live scan and mirrors it here once per pass (midi_set_probe),
// so the probe and calibration paths can report exactly what the matrix read.
static uint8_t s_probe_cell[4] = {0, 0, 0, 0};
static uint8_t s_probe_status  = 0;
void midi_set_probe(const uint8_t cell[4], uint8_t status) {
  for (uint8_t i = 0; i < 4; ++i) s_probe_cell[i] = cell[i];
  s_probe_status = status;
}

// Split the live scan into the nibble-per-byte 0x3C payload described above.
// hw::ModeProbe() is sampled here rather than in the scan so it costs nothing on
// the ~1 ms heartbeat.
static void build_probe(uint8_t b[PROBE_LEN]) {
  for (uint8_t i = 0; i < 4; ++i) {
    b[i]     = (uint8_t)((s_probe_cell[i] >> 4) & 0x0F);   // PA — rotaries, buttons
    b[4 + i] = (uint8_t)(s_probe_cell[i] & 0x0F);          // PB — step switches
  }
  b[8] = (uint8_t)(s_probe_status & 0x0F);
  b[9] = (uint8_t)(hw::ModeProbe() & 0x0F);
}

static void send_probe() {
  uint8_t b[PROBE_LEN];
  build_probe(b);
  send_short(0x3C, b, PROBE_LEN);
  memcpy(s_last_probe, b, PROBE_LEN);
}

void midi_tx_service(Engine &eng) {
  // 1. One-shot probe request (0x3B).
  if (s_probe_once) { s_probe_once = false; send_probe(); }

  // 2. Calibration streaming: emit a probe whenever the raw matrix changes, so
  // the editor sees each detent without polling. The step keys and CLEAR are the
  // only inputs the calibration itself needs, and those are unambiguous, which
  // is what makes this bootstrap work on an uncalibrated device.
  //
  // Compare the built payload against the last one SENT, like for like. The
  // previous version compared the raw 8-bit cells against the masked wire bytes,
  // so a high PA3 made them differ forever and the stream ran flat out.
  if (s_calibrating) {
    uint8_t b[PROBE_LEN];
    build_probe(b);
    if (memcmp(b, s_last_probe, PROBE_LEN) != 0) {
      send_short(0x3C, b, PROBE_LEN);
      memcpy(s_last_probe, b, PROBE_LEN);
    }
  }

  // 3. Selection broadcast: tell the editor which pattern is selected whenever
  // it changes while stopped, and once on every run -> stop.
  if (!eng.running) {
    if (eng.cur_pat != s_last_sent_pat || s_last_running) {
      const uint8_t b[1] = { eng.cur_pat };
      send_short(0x1E, b, 1);
      s_last_sent_pat = eng.cur_pat;
    }
  }
  s_last_running = eng.running;

  // 4. Layout update (section lengths + prescale).
  if (s_layout_req != 0xFF) {
    Pattern p;
    eng.ReadPattern(s_layout_req, p);
    const uint8_t b[6] = { s_layout_req, p.seclen[0], p.seclen[1],
                           p.seclen[2], p.seclen[3], p.prescale };
    send_short(0x18, b, 6);
    s_layout_req = 0xFF;
  }

  // 5. Settings / panel map / info dumps.
  if (s_settings_dump) {
    s_settings_dump = false;
    const uint8_t b[3] = { g_settings.midi_channel, g_settings.clock_source,
                           g_settings.out_mode };
    send_short(0x31, b, 3);
  }
  if (s_panelmap_dump) {
    s_panelmap_dump = false;
    send_packed(0x53, 0, (const uint8_t *)&g_panel_map, PANELMAP_BYTES);
  }
  if (s_info_dump) {
    s_info_dump = false;
    uint8_t flags = 0;
#ifdef SUPEROS_COMBINED
    flags |= 0x01;
#endif
    if (g_panel_map.magic == PANELMAP_MAGIC) flags |= 0x04;   // calibrated
    // Version comes from platformio.ini via tools/version_flag.py, so a release
    // cannot ship a binary that reports the previous version.
#ifndef SUPEROS_VER_MAJOR
#define SUPEROS_VER_MAJOR 0
#define SUPEROS_VER_MINOR 0
#endif
    const uint8_t b[3] = { (uint8_t)SUPEROS_VER_MAJOR, (uint8_t)SUPEROS_VER_MINOR, flags };
    send_short(0x61, b, 3);
  }

  // 6. One bulk dump per pass: they are ~270 bytes each and the queue must stay
  // able to carry the live panel-edit broadcasts.
  if (s_pat_req_mask_lo) {
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
      if (!(s_pat_req_mask_lo & ((uint32_t)1 << p))) continue;
      if (txq_free() < 280) break;                 // no room yet; try next pass
      s_pat_req_mask_lo &= ~((uint32_t)1 << p);
      Pattern pt;
      eng.ReadPattern(p, pt);
      uint8_t raw[PATTERN_BYTES];
      serialize_pattern(pt, raw);
      send_packed(0x11, p, raw, PATTERN_BYTES);
      break;
    }
  } else if (s_trk_req_mask) {
    for (uint8_t t = 0; t < NUM_TRACKS; ++t) {
      if (!(s_trk_req_mask & ((uint16_t)1 << t))) continue;
      if (txq_free() < 90) break;
      s_trk_req_mask &= (uint16_t)~((uint16_t)1 << t);
      uint8_t raw[TRACK_BYTES];
      serialize_track(eng.track[t], raw);
      send_packed(0x25, t, raw, TRACK_BYTES);
      break;
    }
  }

  txq_pump();
}
