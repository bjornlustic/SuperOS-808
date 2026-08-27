// ---------------------------------------------------------------------------
// Loop accounting: the BEFORE numbers for ARCHITECTURE.md Phase 1 conditions
// 3a (awake duty cycle) and 4 (debounce passes vs scan passes). claim-085.
// SysEx 0x5A -> 0x5B, allocated fleet-wide by CORE before this code existed.
//
// ONE COUNTER SET SERVES BOTH CONDITIONS, deliberately: two conditions that
// come out of one build, one run and one artifact cannot disagree about which
// run they describe.
//
// WHY THE LADDER RUNS THE OTHER WAY THAN THE SPEC ASKED FOR. CORE specified
// `0x5A 0x02 <depth>` as a BURN ladder, by analogy with 0x48 burning stack
// frames, having read main.cpp's loop() and nothing else. On this tree that
// ladder cannot work, and the reason is the measurement's own headline:
//
//   THERE IS NO SLEEP INSTRUCTION ANYWHERE IN src/. The loop is a free-running
//   busy loop, so the BEFORE awake duty cycle is 1.000 BY CONSTRUCTION.
//
// A ratio pinned at 1.000 cannot be moved by burning MORE awake cycles. So the
// ladder INJECTS A KNOWN AMOUNT OF IDLE SLEEP per loop pass instead, and the
// ratio must fall by a predicted amount. That inversion is not a convenience:
// it makes the ladder a direct test of the one thing CORE asked to be overruled
// on rather than assumed, which is whether the tick source keeps counting
// ACROSS the sleep. If it does not, sleep is accounted as zero, awake stays
// equal to elapsed, the duty ratio stays 1.000 at every rung, and the host tool
// refuses to print. THE FLATTERING BIAS AND THE RED CONDITION ARE THE SAME
// EVENT, which is the only arrangement in which a duty figure is worth quoting.
//
// THE TICK SOURCE, and why this one. Timer3, normal mode, clk_IO/8. IDLE stops
// clk_CPU and clk_FLASH and leaves clk_I/O running (AT90USB1286 datasheet, the
// sleep-mode clock table), so a clk_I/O timer counts through the sleep. Timer3
// is otherwise untouched: nothing in src/ writes any TCCRn, the core sets it up
// for 8-bit PWM in _init_Teensyduino_internal_ and only analogWrite() and
// tone() would use it, and neither appears in this firmware. Timer0 is NOT used
// as the source even though micros() already exists: micros() is div-64 here,
// so its resolution is 4 us, against 0.5 us for this.
//
//   1 tick = 8 CPU cycles = 0.500 us at 16 MHz     RESOLUTION
//   32-bit tick accumulator wraps at 35.8 minutes  INTERNAL RANGE
//   reported in CYCLES (ticks * 8), which wraps at 268.4 s  REPORTED RANGE
//
// The reported unit is cycles and not ticks on purpose: 0x5B's registry row
// names its fields "elapsed cycles" and "cycles spent OUT of sleep", so there
// is no unit to negotiate between four machines. The cost is the 268.4 s
// reported range, which is why status bit 4 exists and says so.
//
// NOT A CURRENT MEASUREMENT, and it must never be quoted as one. CORE stated
// this limit before the run rather than after the number looked good and it is
// restated here where the code is: IDLE leaves the peripherals clocked, so the
// current delta is not proportional to the duty delta, and converting duty to
// milliamps through datasheet figures produces a PREDICTED current, which
// COMMON rule 1 files as simulated. G6's power half needs a meter in series.
//
// NEVER SHIP. It claims Timer3, adds an ISR, and 0x5A 0x02 makes the machine
// sleep and decimate its own debouncers on command.
// ---------------------------------------------------------------------------
#pragma once

#ifdef SUPEROS_LOOP_ACCT

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#if F_CPU != 16000000L
#error "loop_acct assumes 16 MHz: the tick-to-cycle and us-to-tick conversions are compile-time constants at F_CPU/8"
#endif

#define LA_CYCLES_PER_TICK  8u          // clk_IO/8
#define LA_TICKS_PER_US     2u          // 16 MHz / 8 / 1e6

// Ladder limits. The sleep rung is capped so one pass cannot stall the panel
// past the point where the machine looks hung; the divisor is capped because
// a decimated debouncer holds its shift register, so an edge that IS present
// re-fires once per skipped pass (see the caveat at the call site in loop()).
// 16 ms, not 4 ms. The first hardware ladder ran 250..4000 us and the useful
// part of it was only 1000..4000, a 4x range on three points, because every
// rung below ~615 us collapses into ONE wake-up quantum (see the quantum note
// below). L105 is exactly this: a constant chosen at one operating point keeps
// passing at that point, so sweep the parameter it was chosen against. 16 ms
// makes the swept range 16x on five points. A pass this long makes the LED
// multiplex visibly flicker, which is expected and is another reason a
// laddered machine is an instrument and not a playable one.
#define LA_LADDER_US_MAX    16000u
#define LA_DEB_DIV_MAX      8u
// Bail-out for the sleep rung. Its ONLY job is to turn "the tick source does
// not advance in IDLE" from a hang into a reading: with a dead source the
// deadline is never reached, so the loop would spin forever. Every sleep_cpu()
// here returns on an interrupt (Timer0 overflow alone is ~976 Hz), so 256
// wakeups is far more than LA_LADDER_US_MAX needs and only a dead source
// reaches it.
#define LA_LADDER_GUARD     256u

// THE WAKE-UP QUANTUM, measured on hardware 2026-08-26 and written here because
// it is the reason the first ladder MISSED its prediction and the tool was
// right to refuse. `sleep_cpu()` returns on the NEXT INTERRUPT, not after a
// requested duration, so the deadline loop always OVERSHOOTS by the residual
// time to that interrupt. This firmware has Timer0 overflow at ~976 Hz and USB
// SOF at 1 kHz, so the residual is a few hundred microseconds:
//
//     request   accounted sleep   overshoot
//        250 us      616.0 us       +366.0     below the quantum: the two
//        500 us      614.8 us       +114.8     rungs collapse onto ONE sleep
//       1000 us     1446.2 us       +446.2
//       2000 us     2410.6 us       +410.6
//       4000 us     4412.3 us       +412.3
//
// SLOPE, which is the part with no free parameter in it: 0.964 over 1000..2000
// and 1.001 over 2000..4000. One microsecond requested buys one microsecond
// accounted. The intercept is the quantum and is a property of this machine's
// interrupt rate, not of the counter set.
//
// This is the same shape as the 0x48 stack ladder in finding 43, whose 8..10
// interval read 25.0 B/frame because it straddled the 494 B the boot path had
// already reached, while 10..16 read a flat 36.0. The straddling interval is
// not a second slope, in either instrument.

extern volatile uint16_t g_la_ovf;          // Timer3 overflow count
extern volatile uint32_t g_la_sleep_ticks;  // ticks accounted INSIDE sleep
extern volatile uint32_t g_la_scan_passes;  // condition 4 DENOMINATOR
extern volatile uint32_t g_la_deb_passes;   // condition 4 numerator
extern uint32_t g_la_arm_ticks;             // tick stamp at zero-and-arm
extern uint16_t g_la_ladder_us;             // sleep rung, us per pass, 0 = off
extern uint8_t  g_la_deb_div;               // decimation rung, 1 = every pass
extern uint8_t  g_la_deb_phase;
extern uint8_t  g_la_status;

#define LA_ST_DUTY_MOVED   0x01   // CORE's registered bit0
#define LA_ST_DEB_MOVED    0x02
#define LA_ST_LADDER_ON    0x04
#define LA_ST_STALL        0x08   // guard tripped: the tick source did not advance
#define LA_ST_SATURATED    0x10   // cycle fields would exceed u32; see the range note

ISR(TIMER3_OVF_vect);

// Timer3 free-running at clk_IO/8, normal mode, overflow into g_la_ovf.
static inline void la_init(void) {
  const uint8_t s = SREG; cli();
  TCCR3A = 0;
  TCCR3B = (1 << CS31);        // clk_IO/8, no prescaler change to any other timer
  TCNT3  = 0;
  TIFR3  = (1 << TOV3);
  TIMSK3 |= (1 << TOIE3);
  g_la_ovf = 0;
  SREG = s;
}

// 32-bit tick count. The TOV3-pending fixup is the standard one and it is
// correct only while interrupts are never masked for longer than half a timer
// period (32768 ticks = 16.4 ms). This firmware does exceed that in exactly one
// place, an SPM page write with interrupts off, so a measurement run must not
// write flash. The run protocol in tools/diag/g_loop_acct.py does not.
static inline uint32_t la_ticks(void) {
  const uint8_t s = SREG; cli();
  const uint16_t c = TCNT3;
  uint16_t o = g_la_ovf;
  if ((TIFR3 & (1 << TOV3)) && c < 0x8000u) ++o;
  SREG = s;
  return ((uint32_t)o << 16) | c;
}

// One IDLE sleep, with the time spent in it accounted. This is the ONLY place
// that accounts sleep, so it is the function the Phase 1 refactor should call
// too: an AFTER run that sleeps somewhere else and does not come through here
// would report the sleep as awake, which is the conservative direction but is
// still wrong.
static inline void la_sleep_once(void) {
  const uint32_t t0 = la_ticks();
  set_sleep_mode(SLEEP_MODE_IDLE);
  cli();
  sleep_enable();
  sei();                       // sei + sleep is one instruction pair: no wakeup
  sleep_cpu();                 // can be missed between them
  sleep_disable();
  g_la_sleep_ticks += (uint32_t)(la_ticks() - t0);
}

// The sleep rung: sleep a KNOWN number of microseconds this pass.
static inline void la_ladder_sleep(void) {
  if (!g_la_ladder_us) return;
  const uint32_t t0     = la_ticks();
  const uint32_t target = (uint32_t)g_la_ladder_us * LA_TICKS_PER_US;
  uint16_t guard = 0;
  while ((uint32_t)(la_ticks() - t0) < target) {
    if (++guard > LA_LADDER_GUARD) { g_la_status |= LA_ST_STALL; break; }
    la_sleep_once();
  }
  if (g_la_sleep_ticks) g_la_status |= LA_ST_DUTY_MOVED;
}

static inline void la_count_scan(void) { ++g_la_scan_passes; }

// True on the passes the debounce block is allowed to run. At the default
// divisor of 1 this is every pass and the block is byte-for-byte what it was.
static inline bool la_debounce_due(void) {
  if (g_la_deb_div <= 1) { ++g_la_deb_passes; return true; }
  if (++g_la_deb_phase >= g_la_deb_div) {
    g_la_deb_phase = 0;
    ++g_la_deb_passes;
    return true;
  }
  g_la_status |= LA_ST_DEB_MOVED;
  return false;
}

static inline void la_arm(void) {
  const uint8_t s = SREG; cli();
  g_la_sleep_ticks = 0;
  g_la_scan_passes = 0;
  g_la_deb_passes  = 0;
  g_la_deb_phase   = 0;
  g_la_arm_ticks   = la_ticks();
  g_la_status     &= (uint8_t)~LA_ST_STALL;   // MOVED bits are per power cycle
  SREG = s;
}

// depth encoding for 0x5A 0x02: <us_lo7> <us_hi7> <deb_div>. Both rungs in one
// command so a run can move either ratio, or both, without a second opcode.
static inline void la_ladder_set(uint16_t us, uint8_t div) {
  if (us  > LA_LADDER_US_MAX) us  = LA_LADDER_US_MAX;
  if (div > LA_DEB_DIV_MAX)   div = LA_DEB_DIV_MAX;
  if (div < 1)                div = 1;
  g_la_ladder_us = us;
  g_la_deb_div   = div;
  g_la_deb_phase = 0;
  if (us || div > 1) g_la_status |=  LA_ST_LADDER_ON;
  else               g_la_status &= (uint8_t)~LA_ST_LADDER_ON;
}

#endif  // SUPEROS_LOOP_ACCT
