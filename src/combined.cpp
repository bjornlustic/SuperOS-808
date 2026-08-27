// combined.cpp -- entry dispatch for the combined SuperOS-808 + µPD650C build.
// Empty TU in non-combined builds.
#ifdef SUPEROS_COMBINED
#include <Arduino.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <string.h>
#include <util/delay.h>
#include "pins.h"
#include "hw.h"
#include "combined.h"
#include "controls.h"   // g_panel_map: which status bit is the clock
#include "usb_sof.h"
#include "lamp_probe.h"

// Pre-main boot guard. The Teensy core's usb_init() runs BEFORE setup() and
// busy-waits on USB PLL lock. If the PLL misses lock — seen on the 303 after a
// watchdog reboot landed while the previous firmware's USB/PLL was still
// running — the unit wedges dead until power-off. Arm a 2 s watchdog from .init3
// so ANY pre-setup hang retries the boot. setup() replaces it with the permanent
// 8 s loop watchdog.
// Why the last reset happened, captured before anything clears it. .noinit so
// it survives the reset that produced it, and read out by the telemetry probe:
// a watchdog reset (WDRF) is a hang, a brown-out (BORF) is a power problem and
// a power-on (PORF) is someone reaching for the switch. Guessing between those
// from a description of "it crashed" is not possible.
#ifdef SUPEROS_LAMP_PROBE
// claim-130. One set per firmware; .noinit so the emulator's figures survive
// the watchdog reset that switches back to SuperOS, which is the only side that
// can answer a SysEx and therefore the only side that can report them.
LampAcc  g_lamp_sos   __attribute__((section(".noinit")));
LampAcc  g_lamp_emu   __attribute__((section(".noinit")));
uint32_t g_lamp_magic __attribute__((section(".noinit")));

void lamp_probe_reset() {
  memset(&g_lamp_sos, 0, sizeof(g_lamp_sos));
  memset(&g_lamp_emu, 0, sizeof(g_lamp_emu));
  g_lamp_magic = LAMP_MAGIC;
}

// `wanted` is passed in rather than inferred: the two firmwares decide what the
// lamp should show in completely different ways (SuperOS from live_variation(),
// the emulator from its hysteresis filter), and the probe must not have an
// opinion about which is right -- only about how long the pin disagrees with it.
void lamp_probe_edge(LampAcc &a, uint8_t level, uint8_t wanted) {
  static uint8_t  s_bad[2] = {0, 0};
  static uint32_t s_t0[2]  = {0, 0};
  if (g_lamp_magic != LAMP_MAGIC) return;
  const uint8_t k = (&a == &g_lamp_sos) ? 0 : 1;
  const uint8_t bad = (uint8_t)(level != wanted);
  if (bad == s_bad[k]) return;                  // no transition
  const uint32_t now = micros();
  if (bad) {
    s_t0[k] = now;                              // the excursion starts
  } else {
    const uint32_t d = now - s_t0[k];
    a.low_us += d;
    ++a.dips;
    if (d > a.max_us) a.max_us = d;
  }
  s_bad[k] = bad;
}
#endif

uint8_t g_reset_cause __attribute__((section(".noinit")));
uint8_t g_boot_count  __attribute__((section(".noinit")));

extern "C" __attribute__((naked, used, section(".init3")))
void combined_boot_guard(void) {
  g_reset_cause = MCUSR;
  MCUSR = 0;
  wdt_enable(WDTO_2S);
}

// main.cpp and d650/emu_avr.cpp rename their entry points to these.
void superos_setup(); void superos_loop(); void superos_pcint();
void emu_setup();     void emu_loop();     void emu_pcint();

static uint8_t g_fw = FW_SUPEROS;

// Both firmwares use PCINT0 on the PA status lines; dispatch by active side.
ISR(PCINT0_vect) {
  if (g_fw == FW_D650) emu_pcint(); else superos_pcint();
}

// Power-on escape: STEP 1 + STEP 16 held while switching on forces SuperOS.
//
// Step keys, not a button chord: this probe runs before the panel decode table
// is loaded, and the step matrix is the one part of the panel whose wiring needs
// no calibration. STEP 1 is row 0 / PB0, STEP 16 is row 3 / PB3 — on the AVR,
// PINB bits 4 and 7 with the corresponding select engaged.
static bool escape_held() {
  hw::Init();                       // selects inactive, inputs defined
  PORTF = (uint8_t)(0x0F & ~(1 << 0));
  delayMicroseconds(90);            // PA/PB lines settle slowly (see hw.h)
  const bool s1 = (PINB & _BV(4)) != 0;
  PORTF = (uint8_t)(0x0F & ~(1 << 3));
  delayMicroseconds(90);
  const bool s16 = (PINB & _BV(7)) != 0;
  PORTF = 0x0F;
  return s1 && s16;
}

void setup() {
  MCUSR = 0;
  // Permanent 8 s freeze-recovery watchdog, petted every loop() pass.
  wdt_enable(WDTO_8S);
  const uint8_t sel = eeprom_read_byte(EE_FW_SELECT);
  g_fw = (sel == FW_D650) ? FW_D650 : FW_SUPEROS;
  if (g_fw == FW_D650 && escape_held()) {
    g_fw = FW_SUPEROS;
    eeprom_update_byte(EE_FW_SELECT, FW_SUPEROS);
  }
  if (g_fw == FW_D650) emu_setup(); else superos_setup();
#ifdef SUPEROS_USB_MIDI
  // VBUS gate (usb_sof.h): the core's usb_init ran pre-main with the engine
  // live; park it detached+frozen, then let the gate attach it right back if
  // a cable is already present. Unplugged, the USB engine clock never runs.
  UDCON  |= _BV(DETACH);
  USBCON |= _BV(FRZCLK);
  usb_vbus_gate();
#endif
}

void loop() {
  wdt_reset();
#ifdef SUPEROS_USB_MIDI
  usb_vbus_gate();   // USB engine runs only while a USB-C cable is present
#endif
  if (g_fw == FW_D650) emu_loop(); else superos_loop();
}

void combined_switch_firmware(uint8_t fw) {
  wdt_reset();                       // fresh 8 s budget for the lockout below
  eeprom_update_byte(EE_FW_SELECT, fw);
  // Input lockout before the reboot: a key still held through the reset can land
  // in the bootloader's stay-resident window (STEP 1 sampled ~40 ms after reset;
  // the diode-less matrix can ghost held keys onto it). Wait until every
  // momentary key reads released for 250 ms straight. Hard 4 s timeout so a
  // stuck line can't block the switch forever.
  //
  // Two lines in here are not keys and must not be polled as if they were, or
  // the loop below can never see an idle panel and every switch burns its whole
  // 4 s timeout. Measured 4.80 s of frozen panel and dead MIDI before the
  // machine came back, which is indistinguishable from a crash.
  //
  //   TEMPO CLOCK, in the status group: a free-running square wave, high in
  //   1954 of 3907 samples across a 90 s capture, so it read as "held" on
  //   roughly every other pass.
  //
  //   The MODE rotary, in column 0's PA nibble (mode_mask): a rotary is always
  //   in SOME position, so those bits are never zero. On the bench they read a
  //   constant 6 (1ST PART) for 90 s straight. This was the dominant blocker;
  //   masking the clock alone left the switch still taking 4.80 s.
  //
  // What is left is what the comment above actually means by a momentary key:
  // START/STOP, TAP and FILL IN in the status group, CLEAR in column 0, and the
  // step keys, which read a clean zero when nothing is held.
  const uint8_t status_keys =
      (uint8_t)(0x0F & ~(1u << (g_panel_map.clock_bit & 3)));
  const uint8_t col0_keys = (uint8_t)(1u << (g_panel_map.clear_bit & 3));
  {
    const uint32_t t0 = millis();
    uint32_t idle_since = millis();
    while (millis() - t0 < 4000) {
      bool any = false;
      PORTF = 0x0F; delayMicroseconds(90);
      if (PINB & status_keys) any = true;             // START/STOP, TAP, FILL IN
      for (uint8_t s = 0; s < 4; ++s) {
        PORTF = (uint8_t)(0x0F & ~(1 << s)); delayMicroseconds(90);
        const uint8_t b = PINB;
        if (b & 0xF0) any = true;                     // step keys
        if (s == 0 && (b & col0_keys)) any = true;    // CLEAR (not the MODE rotary)
      }
      PORTF = 0x0F;
      const uint32_t now = millis();
      if (any) idle_since = now;
      else if (now - idle_since >= 250) break;
      delay(2);
    }
  }
  // Shut USB and its PLL down cleanly before the reset so the next boot's
  // usb_init() starts the PLL cold.
  cli();
  UDIEN = 0;
  UDCON |= (1 << DETACH);
  _delay_ms(30);
  USBCON = 0;
  PLLCSR = 0;
  wdt_enable(WDTO_15MS);             // hardware reset via watchdog
  for (;;) {}
}
#endif
