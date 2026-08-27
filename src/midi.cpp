// SuperOS-808 — MIDI IN/OUT implementation. See midi_api.h for the SysEx map.

#include <Arduino.h>
#include <string.h>

#include "midi_api.h"
#include "lamp_probe.h"
#include "pattern.h"
#include "engine.h"
#include "settings.h"
#include "controls.h"
#include "hw.h"
#include "usb_sof.h"
#include "boot_update.h"
// Outside the SUPEROS_COMBINED block on purpose: stack_hwm.h is inside it, which
// is why the -hwm envs can only ever be combined envs. The Phase 1 counter set
// runs on env:app, which is not a combined build.
#include "loop_acct.h"
#ifdef SUPEROS_COMBINED
#include "combined.h"
#include "stack_hwm.h"
#ifdef SUPEROS_STACK_HWM
extern "C" volatile uint8_t g_hwm_sink = 0;   // definition for stack_hwm_burn
#endif
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
static constexpr uint16_t RX_SIZE  = 320;

// The largest inbound message is a staged bootloader page (SysEx 0x70): 256 raw
// bytes pack to 293, plus a 6-byte envelope and the closing F7. The USB path
// buffers the whole message, both delimiters included, and a message that does
// not fit is dropped in SILENCE (usb_sysex_partial sets s_usb_rx_drop), so this
// bound has to be checked at build time rather than discovered on the bench.
static constexpr uint16_t MAX_RX_MSG = 6 + 293 + 1;   // 300
static_assert(RX_SIZE >= MAX_RX_MSG, "RX_SIZE cannot hold a staged boot page");
// Off-stack scratch for the bulk pattern dump/layout paths. A pattern dump needs
// a Pattern (read from the engine) AND its PATTERN_BYTES serialization live at
// once: ~456 bytes. On the combined build that lands in midi_tx_service as a
// 470-byte stack frame, and superos_loop(253) -> midi_tx_service = 723 against
// 663 bytes of stack -> overflow with no pattern push involved (claim-016,
// finding 12). Moving those two buffers here drops the frame to a handful of
// bytes. In SuperOS mode the emulator is not running, so on the combined build
// this reuses the free tail of its 2 KB image arena; on app builds it costs 456
// bytes of otherwise idle RAM. This is a stack fix, orthogonal to the txq_free
// wrap fix (which was ring correctness, not stack).
static constexpr uint16_t DUMP_SCRATCH = sizeof(Pattern) + PATTERN_BYTES;
#ifdef SUPEROS_COMBINED
static constexpr uint16_t ARENA_SIZE = 2048;      // the emulator's image copy
static_assert(ARENA_SIZE >= TXQ_SIZE + OUT_SIZE + RX_SIZE + DUMP_SCRATCH,
              "shared arena too small for the MIDI buffers + dump scratch");
#else
static constexpr uint16_t ARENA_SIZE = TXQ_SIZE + OUT_SIZE + RX_SIZE + DUMP_SCRATCH;
#endif
uint8_t g_shared_arena[ARENA_SIZE];
static uint8_t *const s_dump_scratch = g_shared_arena + TXQ_SIZE + OUT_SIZE + RX_SIZE;

// A pattern dump is the largest message: 228 raw bytes pack to 261, plus the
// 6-byte envelope = 267. The queue holds one of those plus room for the short
// panel-edit broadcasts that can pile up behind it. Messages are enqueued whole
// and drained a few bytes per loop pass, so a dump can never block the ~1 ms
// heartbeat and can never be split by a note or a settings echo.
static uint8_t *const s_txq = g_shared_arena;
static uint16_t s_tx_head = 0, s_tx_tail = 0;

static uint16_t txq_free() {
  // head and tail are each kept in [0, TXQ_SIZE), so head-tail can be negative
  // when the write index has wrapped past 0 more recently than the read index.
  // On this 8-bit AVR int is 16-bit, so uint16_t operands promote to *unsigned*
  // int and the subtraction wraps mod 65536, not mod TXQ_SIZE (640). 65536 is
  // not a multiple of 640, so (head - tail) % TXQ_SIZE is wrong by 256 in every
  // head<tail state: it over-reports free by up to 384 (accepts an oversize
  // message and writes past the tail, corrupting unread bytes) or under-reports
  // by up to 256 (drops a message that fits). Adding TXQ_SIZE keeps the
  // subtraction positive (range [1, 2*TXQ_SIZE)) so the modulo is exact.
  uint16_t used = (uint16_t)((s_tx_head + TXQ_SIZE - s_tx_tail) % TXQ_SIZE);
  return (uint16_t)(TXQ_SIZE - 1 - used);
}

bool midi_tx_msg(const uint8_t *msg, uint16_t len) {
#ifdef SUPEROS_USB_MIDI
  // Mirror every whole message to the USB-MIDI port first, and never gated on
  // DIN ring space: the web editor over USB must see replies even when the
  // slow DIN ring is congested. usb_sof_alive() (usb_sof.h) drops the mirror
  // when no host is servicing the bus, so a dead port can't wedge the loop.
  if (usb_sof_alive() && len >= 2) {
    if (msg[0] == 0xF0 && msg[len - 1] == 0xF7) {
      // 3-arg overload: inner bytes only, the core adds F0/F7 itself.
      usbMIDI.sendSysEx((uint16_t)(len - 2), msg + 1, false);
      usbMIDI.send_now();
    } else if ((msg[0] & 0xF0) == 0x90 && len >= 3) {
      usbMIDI.sendNoteOn(msg[1], msg[2], (uint8_t)((msg[0] & 0x0F) + 1));
    } else if ((msg[0] & 0xF0) == 0x80 && len >= 3) {
      usbMIDI.sendNoteOff(msg[1], msg[2], (uint8_t)((msg[0] & 0x0F) + 1));
    }
  }
#endif
  if (len > txq_free()) return false;   // ring full -> DIN drops it (USB already sent)
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
#ifdef SUPEROS_LAMP_PROBE
static bool s_lamp_dump = false;
#endif
static uint32_t s_pat_req_mask_lo = 0;   // patterns 0..31, bit per pattern
static uint16_t s_trk_req_mask    = 0;   // tracks 0..11
static bool     s_settings_dump   = false;
static bool     s_panelmap_dump   = false;
static bool     s_info_dump       = false;

#ifdef SUPEROS_LOOP_ACCT
// 0x5A -> 0x5B. Deferred for the same reason every other dump is, and for one
// more that is specific to this instrument: answering from inside handle_sysex
// would put the reply's own work inside the pass it is reporting on. It is a
// duty-cycle counter; measuring itself is the failure mode.
static bool     s_la_dump         = false;
#endif

#ifdef SUPEROS_STACK_HWM
// 0x46 -> 0x47. Deferred like every other dump, and the deferral is the point:
// answering from inside handle_sysex would add this reply's frame to the
// deepest chain in the firmware, so the instrument would be measuring itself.
static bool     s_hwm_dump        = false;
#endif

#ifdef SUPEROS_TESTHOOKS
// G12 test hooks. s_inj_* is one panel scan's worth of override (cell[0..3] plus
// status_hi, the whole panel per controls.h) applied in midi_panel_inject();
// s_state_dump answers 0x44 with the 0x45 engine-runtime dump.
static bool    s_state_dump  = false;
static bool    s_inj_on      = false;
static uint8_t s_inj_mask[5] = {0, 0, 0, 0, 0};
static uint8_t s_inj_val[5]  = {0, 0, 0, 0, 0};

static void send_short(uint8_t cmd, const uint8_t *body, uint8_t n);

// The 0x4B body, out of handle_sysex and into the .testhook section (see
// midi_api.h): hook code must not spend the app ceiling's headroom.
TESTHOOK_CODE static void inj_parse(const uint8_t *a, uint16_t an) {
  s_inj_on = (a[0] != 0);
  if (an >= 19) {
    for (uint8_t i = 0; i < 4; ++i) {
      s_inj_mask[i] = (uint8_t)(((a[1  + i] & 0x0F) << 4) | (a[5  + i] & 0x0F));
      s_inj_val[i]  = (uint8_t)(((a[10 + i] & 0x0F) << 4) | (a[14 + i] & 0x0F));
    }
    s_inj_mask[4] = (uint8_t)(a[9]  & 0x0F);   // status group, PA nibble only
    s_inj_val[4]  = (uint8_t)(a[18] & 0x0F);
  }
  const uint8_t st = 0;
  send_short(0x14, &st, 1);
}

// The 0x45 dump body, .testhook section like inj_parse above: a straight
// serializer whose bytes must not come out of the app ceiling. Declared after
// the Engine include; defined here where the statics live.
TESTHOOK_CODE static void state_dump_send(Engine &eng) {
#ifdef SUPEROS_PCINT_ATTRIB
  uint8_t b[32 + CHAIN_MAX];
#else
  uint8_t b[20 + CHAIN_MAX];
#endif
  b[0]  = eng.cur_pat;
  b[1]  = eng.live_variation();
  b[2]  = eng.running ? 1 : 0;
  b[3]  = (uint8_t)(eng.position() & 0x7F);
  b[4]  = eng.play_dir;
  b[5]  = (uint8_t)(eng.mute_mask & 0x7F);
  b[6]  = (uint8_t)((eng.mute_mask >> 7) & 0x7F);
  b[7]  = eng.ResliceActive() ? 1 : 0;   // derived: b[7] and b[8] cannot disagree
  b[8]  = eng.ResliceWindow();
  b[9]  = eng.arp_len;
  b[10] = eng.global_len;          // the LENGTH tool's runtime override
  b[11] = eng.chain_len;
  b[12] = eng.chain_pos;
  b[13] = superos_ui_state();      // tool | menu bit, from main.cpp
  superos_blink_state(&b[14]);     // G10: phase, source, LED, period (6 B)
#ifdef SUPEROS_PCINT_ATTRIB
  superos_pcint_counts(&b[20]);    // pin-change ISR attribution (12 B)
  uint8_t n = 32;
#else
  uint8_t n = 20;                  // env:app-test-pcint carries the counters
#endif
  for (uint8_t i = 0; i < eng.chain_len && i < CHAIN_MAX; ++i)
    b[n++] = (uint8_t)(eng.chain[i] & 0x7F);
  send_short(0x45, b, n);
}
#endif

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

// SysEx 0x4A: reboot into the resident SysEx bootloader. Deferred like the
// firmware switch so main.cpp can flush saves before the jump.
static bool s_boot_req = false;
bool midi_take_boot_request() {
  const bool v = s_boot_req;
  s_boot_req = false;
  return v;
}

// SysEx 0x71: commit a staged bootloader. Deferred for the same reason and one
// more: the copy halts the CPU for tens of milliseconds, so it must not run
// from inside the RX path with saves still pending.
static bool     s_bootupd_req = false;
static uint16_t s_bootupd_crc = 0;
bool midi_take_bootupd_commit(uint16_t &want_crc) {
  const bool v = s_bootupd_req;
  want_crc = s_bootupd_crc;
  s_bootupd_req = false;
  return v;
}
void midi_send_bootupd_status(uint8_t status) {
  const uint16_t crc = bootupd_staged_crc();
  const uint8_t b[4] = { status, (uint8_t)(crc & 0x7F),
                         (uint8_t)((crc >> 7) & 0x7F), (uint8_t)((crc >> 14) & 0x03) };
  send_short(0x73, b, 4);
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

static void handle_sysex(Engine &eng, uint8_t *body, uint16_t blen) {
  if (blen < 2 || body[0] != MFR_ID) return;
  const uint8_t cmd = body[1];
  const uint8_t *a  = body + 2;               // arguments
  const uint16_t an = (uint16_t)(blen - 2);

  switch (cmd) {
    case 0x10:                                 // request pattern
      if (an >= 1 && a[0] < NUM_PATTERNS) s_pat_req_mask_lo |= (uint32_t)1 << a[0];
      break;

    case 0x12: {                               // push pattern
      if (an < 4) break;
      const uint8_t pat = a[0];
      if (pat >= NUM_PATTERNS) break;
      const uint8_t want = (uint8_t)((a[1] & 0x7F) | ((a[2] & 1) << 7));
      // Unpack over the message being parsed instead of into a stack buffer.
      //
      // A 228-byte local here overflowed the stack on COMBINED builds and hung
      // the machine, every time, on any pattern push. The chain is
      // raw[228] + Pattern p (228) + flash_pat_write's buf[243] = 699 bytes
      // live at once, against the 663 that build has left over 91.9% RAM. The
      // SuperOS-only builds have four times the headroom and never showed it,
      // and so does the 606, which takes the identical push without complaint.
      // Measured on hardware 2026-08-25: USB kept enumerating (it is
      // interrupt-driven) while the main loop was gone, so it presented as a
      // dead machine that still appeared connected, and only a power cycle
      // recovered it.
      //
      // In-place is safe, and not by luck: unpack7 consumes 8 bytes for every 7
      // it writes, and the destination starts five bytes BEHIND the source
      // (body vs a+3 = body+5), so the write pointer falls further behind on
      // every group and can never overtake the read. pat and want are both
      // taken from the envelope above, before any of it is overwritten.
      uint8_t *const raw = body;
      const uint16_t got = unpack7(a + 3, (uint16_t)(an - 3), raw, PATTERN_BYTES);
      uint8_t st = 0;
      if (got != PATTERN_BYTES)          st = 1;
      else if (xor_sum(raw, got) != want) st = 2;
      if (!st) {
        // Deserialize through s_dump_scratch, not a 228-byte stack local: this is
        // the other half of the claim-016 combined-build overflow (the non-cached
        // push chain superos_loop -> handle_sysex -> flash_pat_write -> write).
        Pattern &p = *reinterpret_cast<Pattern *>(s_dump_scratch);
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
      // Read the fourth operand into a local BEFORE the call, like the three
      // above. It is not stylistic: `scratch` below aliases the buffer `a`
      // points into (s_dump_scratch and the USB reassembly buffer s_usb_rx are
      // the same address on combined builds), so the call clobbers the message.
      // Passing `(a[3] & 1) != 0` as an argument would leave that safe only by
      // argument evaluation order, and A303's claim-055 was a live bug that was
      // exactly an unspecified-evaluation-order dependency.
      const bool on = (a[3] & 1) != 0;
      if (pat >= NUM_PATTERNS || inst >= NUM_INSTRUMENTS || s >= MAX_STEPS) break;
      // Off-stack scratch, same arena and same reason as the 0x12 push above:
      // the 228-byte local this replaces was claim-080's overflow.
      eng.SetPatternStep(pat, inst, s, on,
                         *reinterpret_cast<Pattern *>(s_dump_scratch));
      s_save_req = true;
      break;
    }

#ifndef SUPEROS_NO_SYSEX_RANDOMIZE
    case 0x2E: {                               // RANDOMIZE (fleet contract)
      // <pat> <lane> <density> <seed_lo> <seed_hi>, ack 0x14.
      // Ratified fleet-wide at `343fbb9`; the 303 is the reference (rule 5) and
      // this matches its payload, its ack codes and its density scale. The range
      // rule and the ack status live in Engine::HostRandomize, one place, so the
      // wire path and any future panel path cannot drift apart.
      //
      // lane 0 is AC here, ACCENT on the 606, a note on the 101 and the WHOLE
      // PATTERN on the 303, so 0x7F is the only lane a cross-machine schedule
      // can use. Recorded, not fixable: it is what each machine's own 0x16
      // numbering already means.
      if (an < 5) { const uint8_t st = 1; send_short(0x14, &st, 1); break; }
      // EVERY operand into a local BEFORE the call, exactly as 0x13 does and for
      // the same reason: `scratch` below aliases the buffer `a` points into
      // (s_dump_scratch and the USB reassembly buffer s_usb_rx are the same
      // address on combined builds), so the call clobbers the message.
      const uint8_t  pat     = (uint8_t)(a[0] & 0x7F);
      const uint8_t  lane    = (uint8_t)(a[1] & 0x7F);
      const uint8_t  density = (uint8_t)(a[2] & 0x7F);
      const uint16_t seed    = (uint16_t)((a[3] & 0x7F) | ((uint16_t)(a[4] & 0x7F) << 7));
      const uint8_t st = eng.HostRandomize(pat, lane, density, seed,
                                           *reinterpret_cast<Pattern *>(s_dump_scratch));
      // Deferred save, never a flash write inline: it blocks long enough to drop
      // the next SysEx.
      if (st == 0) s_save_req = true;
      send_short(0x14, &st, 1);
      break;
    }
#endif  // SUPEROS_NO_SYSEX_RANDOMIZE

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

#ifdef SUPEROS_TESTHOOKS
    // --- G12 test hooks (never in a shipping build) ------------------------
    case 0x44:                                 // request the engine runtime dump
      s_state_dump = true;
      break;

    // 0x4B <en> <mask nibbles x9> <val nibbles x9>: overlay one panel scan.
    // Nibble-per-byte like the 0x3C probe, so the message is 7-bit clean with no
    // packing and the host reuses the probe's own byte order: [0..3] = the four
    // selects' PA nibble, [4..7] = their PB nibble, [8] = status_hi. en=0
    // releases the panel back to the matrix.
    case 0x4B:
      if (an >= 1) inj_parse(a, an);
      break;
#endif

    // 0x52 (indicator-LED pin remap) is retired: SuperOS no longer drives those
    // lamps at all, because their bus lines ARE instrument-data bits and holding
    // one high arms that voice. See the note in engine.h.

    case 0x4A:                                 // reboot into the SysEx bootloader
      s_boot_req = true;
      break;

#ifdef SUPEROS_LAMP_PROBE
    // 0x5C 0x00 -> report both firmwares' lamp figures as 0x5D
    // 0x5C 0x01 -> zero them and arm
    case 0x5C:
      if (an >= 1 && a[0] == 1) lamp_probe_reset();
      else                      s_lamp_dump = true;
      break;
#endif

    case 0x60: s_info_dump = true; break;

#ifdef SUPEROS_LOOP_ACCT
    // 0x5A: the Phase 1 loop-accounting counter set (claim-085).
    //   0x5A 0x00                      read       -> 0x5B
    //   0x5A 0x01                      zero and arm
    //   0x5A 0x02 <us_lo7> <us_hi7> <div>   set the ladder
    //
    // The ladder takes BOTH rungs in one command rather than two opcodes: the
    // sleep rung moves 3a's ratio and the decimation rung moves 4's, and a run
    // that needs to move both should not have to interleave two commands into
    // the pass boundary. A bare `0x5A 0x02` with no argument is the OFF rung.
    case 0x5A:
      if (an == 0 || a[0] == 0x00) { s_la_dump = true; }
      else if (a[0] == 0x01)       { la_arm(); }
      else if (a[0] == 0x02) {
        const uint16_t us  = (an >= 3) ? (uint16_t)(a[1] | ((uint16_t)a[2] << 7)) : 0;
        const uint8_t  div = (an >= 4) ? a[3] : 1;
        la_ladder_set(us, div);
      }
      break;
#endif
#ifdef SUPEROS_STACK_HWM
    case 0x46: s_hwm_dump = true; break;   // report the stack high-water mark
    // 0x48 <depth>: burn a known amount of stack. The RED CONDITION for 0x46:
    // an instrument that cannot be made to go UP is not measuring anything.
    case 0x48: if (an >= 1) stack_hwm_burn_capped(a[0]); break;
#endif

    // --- bootloader replacement over USB-C or DIN (see boot_update.h) -------
    case 0x70: {                               // stage one bootloader page
      if (an < 3) break;
      const uint8_t idx  = a[0];
      const uint8_t want = (uint8_t)((a[1] & 0x7F) | ((a[2] & 1) << 7));
      // Unpack in place over the packed bytes: the output index always trails
      // the input index (7 bytes out per 8 in), so nothing unread is clobbered.
      // A separate 256-byte buffer would not fit the combined build's RAM.
      uint8_t *raw = body + 5;
      const uint16_t got = unpack7(raw, (uint16_t)(an - 3), raw, 256);
      uint8_t st = BOOTUPD_OK;
      if (idx >= BOOTUPD_PAGES)           st = BOOTUPD_BAD_INDEX;
      else if (got != 256)                st = BOOTUPD_BAD_LENGTH;
      else if (xor_sum(raw, got) != want) st = BOOTUPD_BAD_CKSUM;
      else                                st = bootupd_stage(idx, raw);
      midi_send_bootupd_status(st);
      break;
    }

    case 0x71:                                 // commit the staged image
      if (an < 3) break;
      s_bootupd_crc = (uint16_t)((a[0] & 0x7F) | ((uint16_t)(a[1] & 0x7F) << 7) |
                                 ((uint16_t)(a[2] & 0x03) << 14));
      s_bootupd_req = true;
      break;

    case 0x72:                                 // report staging state
      midi_send_bootupd_status(flash_boot_service_present() ? BOOTUPD_OK
                                                            : BOOTUPD_NO_SERVICE);
      break;

    default: break;
  }
}

// System Real-Time handling, shared by the DIN and USB paths. Incoming clock
// is echoed to MIDI OUT (both ports) so downstream gear can chase.
static void rx_realtime(uint8_t b, MidiClockIn &mc) {
  switch (b) {
    case 0xF8:
      mc.pulses++;
      if (g_settings.out_mode == OUT_MODE_OUT) {
        if (Serial1.availableForWrite() > 0) Serial1.write(b);
#ifdef SUPEROS_USB_MIDI
        if (usb_sof_alive()) usbMIDI.sendRealTime(b);
#endif
      }
      break;
    case 0xFA: s_transport_latch = true;  mc.started = true; break;
    case 0xFB: s_transport_latch = true;  mc.started = true; break;
    case 0xFC: s_transport_latch = false; mc.stopped = true; break;
    default: break;
  }
  mc.transport = s_transport_latch;
}

// One complete channel-voice message (status + data bytes), shared by the DIN
// running-status parser and the USB drain.
static void handle_channel_msg(Engine &eng, uint8_t status, const uint8_t *d) {
  if (!channel_ok(status)) return;
  const uint8_t type = (uint8_t)(status & 0xF0);

  if (type == 0x90 && d[1] > 0) {              // Note On
    for (uint8_t i = INST_BD; i < NUM_INSTRUMENTS; ++i)
      if (INSTRUMENT_NOTE[i] == d[0]) {
        eng.TriggerNow((uint16_t)1 << i, d[1] >= 100);
        break;
      }
  } else if (type == 0xB0) {                   // Control Change
    if (d[0] == 32) s_bank_lsb = d[1];         // Bank Select LSB = variation
  } else if (type == 0xC0) {                   // Program Change
    // All 32 patterns from Program Change alone: PC 0-15 = variation A slots
    // 1-16, PC 16-31 = variation B. Bank Select LSB still picks the variation
    // for hosts that send it and takes precedence, but it is awkward to send
    // from a DAW arrangement and the low bit of the program number is not, so
    // the second bank is reachable either way. Higher program numbers repeat
    // the pair rather than being ignored.
    const uint8_t pc  = (uint8_t)(d[0] & 0x7F);
    const uint8_t var = (s_bank_lsb <= 1) ? s_bank_lsb : (uint8_t)((pc >> 4) & 1);
    eng.SetVariation(var ? VAR_B : VAR_A);
    eng.SelectSlot((uint8_t)(pc & 15));
  }
}

static void rx_byte(Engine &eng, uint8_t b, MidiClockIn &mc) {
  // System Real-Time can interleave anywhere, including inside a SysEx.
  if (b >= 0xF8) { rx_realtime(b, mc); return; }

  if (b == 0xF0) { s_in_sysex = true; s_rx_len = 0; return; }
  if (b == 0xF7) {
    if (s_in_sysex) handle_sysex(eng, s_rx, s_rx_len);
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

  handle_channel_msg(eng, s_run_status, s_ch_buf);
}

#ifdef SUPEROS_USB_MIDI
// USB SysEx reassembly. The Teensy usb_midi core's RX buffer is only 60 bytes
// (USB_MIDI_SYSEX_MAX), but the editor's pattern push (0x12, ~270 bytes) and
// the panel-map write (0x50) exceed it. So we register the CHUNKED (partial)
// handler: the core hands the message over in <=60-byte pieces (complete=0)
// plus a final piece (complete=1), and the whole F0..F7 message is reassembled
// here, then dispatched exactly like the DIN path. A separate buffer from the
// DIN s_rx, so a USB message can't corrupt a DIN SysEx in flight.
#ifdef SUPEROS_COMBINED
// Combined builds are RAM-tight: carve the buffer out of the shared arena's
// free tail (the emulator claims the whole arena, but it never runs in the
// same boot as this code — same overlay rule as the other MIDI buffers).
static_assert(ARENA_SIZE >= TXQ_SIZE + OUT_SIZE + RX_SIZE + RX_SIZE,
              "no room for the USB SysEx buffer in the shared arena");
static uint8_t *const s_usb_rx = g_shared_arena + TXQ_SIZE + OUT_SIZE + RX_SIZE;
#else
static uint8_t  s_usb_rx_store[RX_SIZE];
static uint8_t *const s_usb_rx = s_usb_rx_store;
#endif
static uint16_t s_usb_rx_len  = 0;
static bool     s_usb_rx_drop = false;   // overflow: skip the rest of the msg
static Engine  *s_usb_eng     = nullptr; // set by midi_rx_poll before draining
static void usb_sysex_partial(const uint8_t *data, uint16_t length, bool complete) {
  if (length && data[0] == 0xF0) {       // first chunk of a new message
    s_usb_rx_len  = 0;
    s_usb_rx_drop = false;
  }
  if (!s_usb_rx_drop) {
    if (s_usb_rx_len + length <= RX_SIZE) {
      for (uint16_t i = 0; i < length; ++i) s_usb_rx[s_usb_rx_len++] = data[i];
    } else {
      s_usb_rx_drop = true;              // too big -> ignore the remainder
    }
  }
  if (complete) {
    if (!s_usb_rx_drop && s_usb_rx_len >= 4 && s_usb_eng &&
        s_usb_rx[0] == 0xF0 && s_usb_rx[s_usb_rx_len - 1] == 0xF7) {
      handle_sysex(*s_usb_eng, s_usb_rx + 1, (uint16_t)(s_usb_rx_len - 2));
      s_last_rx_ms = millis();
    }
    s_usb_rx_len  = 0;
    s_usb_rx_drop = false;
  }
}
#endif

void midi_usb_init() {
#ifdef SUPEROS_USB_MIDI
  // 3-arg (partial) overload -> reassembled above (handles >60-byte messages).
  usbMIDI.setHandleSystemExclusive(usb_sysex_partial);
#endif
}

void midi_rx_poll(Engine &eng, MidiClockIn &mc) {
  uint8_t guard = 64;      // bound the work per pass so a burst can't stall the loop
  while (Serial1.available() > 0 && guard--) rx_byte(eng, Serial1.read(), mc);

#ifdef SUPEROS_USB_MIDI
  // USB MIDI RX — mirror of the DIN handling above. Bounded drain: read()
  // returns false for SysEx-continuation packets as well as for "empty", so
  // keep going past a false return or a multi-packet SysEx dribbles in at one
  // packet per poll. The OUT endpoint is 64 B x 2 banks; 48 iterations flush a
  // full bank plus slack, and an empty read() is only a few register checks.
  s_usb_eng = &eng;
  for (uint8_t n = 0; n < 48; ++n) {
    if (!usbMIDI.read()) continue;  // false = empty OR SysEx continuation consumed
    const uint8_t ut = usbMIDI.getType();
    if (ut >= 0xF8) {
      rx_realtime(ut, mc);
    } else if (ut == usbMIDI.NoteOn || ut == usbMIDI.NoteOff ||
               ut == usbMIDI.ControlChange || ut == usbMIDI.ProgramChange) {
      const uint8_t status = (uint8_t)(ut | (uint8_t)(usbMIDI.getChannel() - 1));
      const uint8_t d[2] = { usbMIDI.getData1(), usbMIDI.getData2() };
      handle_channel_msg(eng, status, d);
    }
  }
#endif
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

#ifdef SUPEROS_TESTHOOKS
// G12 panel inject (0x4B). Applied to the live scan before the debouncers, so
// the injected state also reaches midi_set_probe below it: a host verifies what
// it injected by reading 0x3B back, exactly as it verifies a real press.
void midi_panel_inject(uint8_t cell[4], uint8_t &status_hi) {
  if (!s_inj_on) return;
  for (uint8_t i = 0; i < 4; ++i)
    cell[i] = (uint8_t)((cell[i] & (uint8_t)~s_inj_mask[i]) |
                        (s_inj_val[i] & s_inj_mask[i]));
  status_hi = (uint8_t)((status_hi & (uint8_t)~s_inj_mask[4]) |
                        (s_inj_val[4] & s_inj_mask[4]));
}
#endif

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
    Pattern &p = *reinterpret_cast<Pattern *>(s_dump_scratch);   // off-stack, claim-016
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
#ifdef SUPEROS_TESTHOOKS
  // 5b. G12 engine runtime dump (0x44 -> 0x45): the performance state the CLEAR
  // tools write, none of which is in the pattern and none of which the 0x11 dump
  // can therefore see. Every field is one 7-bit byte; mute_mask is 12 bits so it
  // is split. Short on purpose -- it must not compete with the bulk dumps for
  // the TX ring (claim-002) or add a frame to the stack (claim-016).
  if (s_state_dump) {
    s_state_dump = false;
    state_dump_send(eng);
  }
#endif
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

#ifdef SUPEROS_LOOP_ACCT
  if (s_la_dump) {
    s_la_dump = false;
    // Four u32 in the 0x47 encoding widened to five bytes each (35 bits), then
    // the status byte. Reported in CYCLES and not in the timer's own ticks
    // because 0x5B's registry row names its fields "elapsed cycles" and "cycles
    // spent OUT of sleep", and four machines must not have to negotiate a unit.
    const uint32_t el_t = (uint32_t)(la_ticks() - g_la_arm_ticks);
    const uint32_t sl_t = g_la_sleep_ticks;
    uint8_t st = g_la_status;
    // u32 cycles runs out at 268.4 s. Say so rather than wrapping in silence.
    if (el_t > (0xFFFFFFFFUL / LA_CYCLES_PER_TICK)) st |= LA_ST_SATURATED;
    const uint32_t v[4] = {
      el_t * LA_CYCLES_PER_TICK,                  // [0] elapsed cycles
      (uint32_t)(el_t - sl_t) * LA_CYCLES_PER_TICK,  // [1] cycles OUT of sleep
      g_la_scan_passes,                           // [2] scan passes
      g_la_deb_passes,                            // [3] debounce passes
    };
    uint8_t b[21];
    for (uint8_t i = 0; i < 4; ++i) {
      const uint32_t x = v[i];
      b[i * 5 + 0] = (uint8_t)( x        & 0x7F);
      b[i * 5 + 1] = (uint8_t)((x >>  7) & 0x7F);
      b[i * 5 + 2] = (uint8_t)((x >> 14) & 0x7F);
      b[i * 5 + 3] = (uint8_t)((x >> 21) & 0x7F);
      b[i * 5 + 4] = (uint8_t)((x >> 28) & 0x0F);
    }
    b[20] = (uint8_t)(st & 0x7F);
    send_short(0x5B, b, 21);
  }
#endif

#ifdef SUPEROS_STACK_HWM
  if (s_hwm_dump) {
    s_hwm_dump = false;
    // Three 16-bit values, low 7 bits first. `base` is here so the host can
    // check the running binary's `_end` against the ELF it is comparing
    // against, rather than assuming the machine is running the tree on disk.
    const uint16_t unused = stack_hwm_unused();
    const uint16_t freesz = stack_hwm_free();
    const uint16_t base   = stack_hwm_base();
    const uint8_t b[9] = {
      (uint8_t)(unused & 0x7F), (uint8_t)((unused >> 7) & 0x7F), (uint8_t)((unused >> 14) & 0x03),
      (uint8_t)(freesz & 0x7F), (uint8_t)((freesz >> 7) & 0x7F), (uint8_t)((freesz >> 14) & 0x03),
      (uint8_t)(base   & 0x7F), (uint8_t)((base   >> 7) & 0x7F), (uint8_t)((base   >> 14) & 0x03),
    };
    send_short(0x47, b, 9);
  }
#endif

  // 6. One bulk dump per pass: they are ~270 bytes each and the queue must stay
  // able to carry the live panel-edit broadcasts.
#ifdef SUPEROS_LAMP_PROBE
  if (s_lamp_dump) {
    s_lamp_dump = false;
    // Five 32-bit fields per firmware, 7 bits at a time, SuperOS then D650C.
    // Three fields per firmware (low, dips, max), 32 bits as five 7-bit bytes,
    // SuperOS then D650C. The host times the run with its own clock.
    uint8_t b[30]; uint8_t n = 0;
    const LampAcc *src[2] = { &g_lamp_sos, &g_lamp_emu };
    for (uint8_t k = 0; k < 2; ++k) {
      const uint32_t f[3] = { src[k]->low_us, src[k]->dips, src[k]->max_us };
      for (uint8_t i = 0; i < 3; ++i)
        for (uint8_t j = 0; j < 5; ++j) b[n++] = (uint8_t)((f[i] >> (7 * j)) & 0x7F);
    }
    send_short(0x5D, b, n);
  }
#endif

  if (s_pat_req_mask_lo) {
    for (uint8_t p = 0; p < NUM_PATTERNS; ++p) {
      if (!(s_pat_req_mask_lo & ((uint32_t)1 << p))) continue;
      if (txq_free() < 280) break;                 // no room yet; try next pass
      s_pat_req_mask_lo &= ~((uint32_t)1 << p);
      // Serialize through s_dump_scratch, not the stack: Pattern + raw is ~456 B
      // and this frame is the claim-016 overflow (723 vs 663 on combined).
      Pattern &pt = *reinterpret_cast<Pattern *>(s_dump_scratch);
      uint8_t *raw = s_dump_scratch + sizeof(Pattern);
      eng.ReadPattern(p, pt);
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
