#pragma once
// SUPEROS_USB_MIDI builds: USB-C liveness + VBUS gating.
//
// usb_sof_alive(): gate for every usbMIDI send. The Teensy core's usb-midi
// TX wait (send_raw in cores/usb_midi/usb_api.cpp) uses the USB frame
// counter UDFNUML as its timeout clock. SOFs stop when the host parks the
// port (failed enumeration, reset storm, cable pulled at the wrong moment)
// but usb_configuration can still read as set, and then every send spins
// forever waiting for bank room. With the firmware mirroring the MIDI clock
// to USB ~70 times a second, that spin trips the loop watchdog (bench
// boot-loop, 2026-08-01). SOF = 1 ms cadence when the bus is alive; 5 ms of
// stillness = USB is down, drop the mirror byte instead of risking the spin.
#ifdef SUPEROS_USB_MIDI
#include <Arduino.h>

static inline bool usb_sof_alive() {
  static uint8_t  last_fn     = 0;
  static uint32_t last_adv_ms = 0;
  const uint8_t  fn  = UDFNUML;
  const uint32_t now = millis();
  if (fn != last_fn) { last_fn = fn; last_adv_ms = now; return true; }
  return (uint32_t)(now - last_adv_ms) < 5;
}

// VBUS gate: the USB engine runs exactly when a cable is present. VBUS high
// -> PLL up, unfreeze, attach; VBUS low -> detach + freeze + PLL off. The
// unplugged branch is ENFORCED every pass, never edge-triggered, with the
// USB device interrupts masked: with the cable out, the floating D+/D-
// lines fake wake-up events and the Teensy core's WAKEUPI handler restarts
// the PLL and clears FRZCLK behind the gate's back (seen in the field on
// the 606). UDIEN=0 silences that ISR entirely and the
// per-pass re-assert closes any window something still finds. The plugged
// branch only intervenes when WE parked the port (DETACH set); legitimate
// host suspend stays the core's business. Called once per loop() pass.
// VBUS is not optional, and there is no firmware way around it. If the board
// does not route USB-C VBUS to the AVR's VBUS pin, USB cannot work at all:
//   - "The D+ or D- pull-up will be activated as soon as the DETACH bit is
//      cleared and VBUS is present." (datasheet 22.2)
//   - OTGPADE is the VBUS pad, and "must be enabled in both Host and Device
//      modes in order to allow USB operation (attaching, transmitting...)"
//      (datasheet, USBCON bit 4)
// So the pull-up is gated on VBUS in silicon, and disabling VBUS detection to
// dodge the check is explicitly not allowed either. Clearing DETACH on a board
// with no VBUS sense leaves the device invisible: the host never sees a
// pull-up, never drives a bus reset, and EORSTI never fires. Diagnosed exactly
// that way on an MSW board, 2026-08-24. Fix the wire; do not add a bypass.
static inline void usb_vbus_gate() {
  // Boot settle: stay parked for the first 150 ms even with a cable in, so
  // setup()-time scans and init run with the engine off.
  if ((USBSTA & _BV(VBUS)) && millis() >= 150) {
    if (UDCON & _BV(DETACH)) {                // parked by us: bring USB up
      if (!(PLLCSR & _BV(PLOCK))) {
        PLLCSR = 0x16;                        // core's PLL_CONFIG() for the
        while (!(PLLCSR & _BV(PLOCK))) {}     // AT90USB1286 @ 16 MHz; ~100 us
      }
      USBCON &= (uint8_t)~_BV(FRZCLK);        // engine clock on
      UDIEN   = _BV(EORSTE) | _BV(SOFE);      // the core's normal ISR set
      UDCON  &= (uint8_t)~_BV(DETACH);        // attach: host enumerates
    }
  } else {
    UDIEN   = 0;                              // no wake/suspend ISR ping-pong
    UDCON  |= _BV(DETACH);
    USBCON |= _BV(FRZCLK);
    PLLCSR  = 0;
  }
}
#endif  // SUPEROS_USB_MIDI
