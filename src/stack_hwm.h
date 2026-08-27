// ---------------------------------------------------------------------------
// Stack high-water mark, measured on the machine instead of derived from .su
// sidecars. claim-080 / claim-085 Phase 0.
//
// WHY THIS EXISTS. `tools/diag/stack_walk.py` is a static walker and it has
// been wrong by 318 B on this firmware inside one session: it did not match
// `ijmp`, so `Engine::WritePattern`'s tail call took the entire flash store out
// of the call graph without reporting a gap. It still costs an unfollowed
// `icall` as zero, still costs a library function with no `.su` entry as zero,
// and its INDIRECT_MAP is hand-written, so a MISSING entry is a silent edge.
// Every one of those makes the published number a LOWER bound. A painted
// high-water mark has none of those failure modes: it follows every indirect
// call, every library call and every path nobody thought to walk, because it is
// not a model of the program, it is the RAM the program actually touched.
//
// HOW IT WORKS. Fill the whole free region with a known byte before the C
// runtime sets SP, then later find the lowest address that is no longer that
// byte. Everything from there up to RAMEND has been under the stack at some
// point since reset.
//
// WHAT MAKES IT SOUND HERE, and it is worth checking on any machine that
// copies this: the painted region must be stack and nothing else. On this
// firmware it is. `malloc` is not linked into any 808 env (no `malloc` and no
// `__brkval` in the symbol table), so no heap ever grows up out of `_end`, and
// `_end == __heap_start == 0x801e73` with RAMEND at 0x20FF leaves 653 B that
// only the stack can reach. If a future build links malloc, this instrument
// reports the heap as stack usage and must be re-derived, not trusted.
//
// THE ONE FALSE READING IT CAN GIVE, stated so nobody has to discover it: if
// the deepest frame happened to store 0xC5 at the very byte the walk stops on,
// the mark reads shallower than the truth. That understates by whatever run of
// 0xC5 sits at the bottom of that frame. It is the same direction of error as
// the static walker, so a hardware number that is WORSE than the static one is
// unambiguous, and a hardware number that is better is the case to be careful
// with. Drive it differentially (read, exercise, read again) rather than
// trusting a single absolute figure.
// ---------------------------------------------------------------------------
#pragma once

#ifdef SUPEROS_STACK_HWM

#include <stdint.h>

// Linker-provided bounds of the free region. `_end` is the first byte past
// .bss; `__stack` is RAMEND. Both are addresses in the data space.
extern "C" uint8_t _end;
extern "C" uint8_t __stack;

// Not 0x00 and not 0xFF: both of those are what uninitialised RAM and cleared
// .bss look like, so either would make "the paint never ran" indistinguishable
// from "the stack never got here".
// A macro and not a constexpr: the painter needs it as an assembler immediate
// in a `naked` function, and gcc will not fold a C++ constant into an "M"
// operand there.
#define STACK_HWM_PAINT 0xC5

// Runs in .init1, which is BEFORE avr-libc sets SP in .init2 and before .bss is
// cleared in .init4. So the whole region including the top of the stack is
// painted, and nothing is live on the stack to destroy: this is reached by
// fall-through from .init0 with no return address pushed. `naked` is what keeps
// it that way, and it must not return: control falls through to .init2.
//
// Deliberately assembly rather than a C loop. A C loop here would need SP set
// up to be safe, which is exactly what has not happened yet.
__attribute__((used, naked, section(".init1")))
static void stack_hwm_paint(void) {
  __asm__ volatile (
    "  ldi r30, lo8(_end)      \n"
    "  ldi r31, hi8(_end)      \n"
    "  ldi r24, 0xC5           \n"   // STACK_HWM_PAINT
    "  ldi r25, hi8(__stack)   \n"
    "  rjmp 2f                 \n"
    "1:                        \n"
    "  st   Z+, r24            \n"
    "2:                        \n"
    "  cpi  r30, lo8(__stack)  \n"
    "  cpc  r31, r25           \n"
    "  brlo 1b                 \n"
    "  breq 1b                 \n"
  );
}

// Bytes at the bottom of the free region that are still painted, i.e. RAM the
// stack has never reached. The peak is stack_hwm_free() - stack_hwm_unused().
static uint16_t stack_hwm_unused(void) {
  const uint8_t *p   = &_end;
  const uint8_t *top = &__stack;
  uint16_t n = 0;
  while (p <= top && *p == STACK_HWM_PAINT) { ++p; ++n; }
  return n;
}

// Total size of the region the stack has to live in. This is the 653 B figure,
// computed by the machine from its own linker symbols rather than carried
// between sessions as a number, which is L39.
static uint16_t stack_hwm_free(void) {
  return (uint16_t)(&__stack - &_end + 1);
}

// Deliberate stack burn, so the instrument has a RED CONDITION.
//
// The first hardware run read 494 B and did not move under ANY drive: 16 slot
// dumps, four accepted 0x12 pushes (ack 0x14 status 0, so the deep flash-append
// chain really executed), 16 randomizes, a transport start with 400 clocks, and
// a slot/edit storm. An unmoving mark is exactly what a broken instrument looks
// like, and `g_stack_hwm.py exercise` says so in as many words. This is the
// control that separates the two: burn a KNOWN amount of stack and require the
// mark to rise by about that much. If it cannot go up, nothing it reports down
// here means anything.
//
// Bench only: it lives inside SUPEROS_STACK_HWM, which is `env:combined-nat-hwm`
// and nothing that ships. The depth is CAPPED so the burn cannot itself be the
// overflow it is measuring for.
extern "C" volatile uint8_t g_hwm_sink;

#define STACK_HWM_BURN_PAD   32
// 16 frames is ~630 B of burn on top of the ~126 B the SysEx handler is already
// at, which leaves ~265 B untouched against the 26 the worst ISR needs. Chosen
// from the measured frame size below, not guessed: the sweep 10..16 is what
// calibrates this instrument in BYTES rather than in "it moved".
#define STACK_HWM_BURN_MAX   16

__attribute__((noinline)) static void stack_hwm_burn(uint8_t depth) {
  volatile uint8_t pad[STACK_HWM_BURN_PAD];
  for (uint8_t i = 0; i < STACK_HWM_BURN_PAD; ++i) pad[i] = (uint8_t)(depth + i);
  if (depth) stack_hwm_burn((uint8_t)(depth - 1));
  g_hwm_sink = (uint8_t)(g_hwm_sink + pad[0]);
}

static inline void stack_hwm_burn_capped(uint8_t depth) {
  stack_hwm_burn(depth > STACK_HWM_BURN_MAX ? STACK_HWM_BURN_MAX : depth);
}

// The address the paint started from. Published so the host can check it
// against the ELF it is analysing: if these disagree, the flashed binary is not
// the one the static numbers describe, and that has been a real failure here
// (L12/L14, the `57cf8fedirty` image).
static uint16_t stack_hwm_base(void) {
  return (uint16_t)(uintptr_t)&_end;
}

#endif  // SUPEROS_STACK_HWM
