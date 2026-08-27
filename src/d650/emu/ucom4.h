// ucom4.h — bit-accurate NEC uCOM-43 / uPD650 (D650C) core, standalone C99.
//
// Faithful port of MAME's ucom4 CPU core (src/devices/cpu/ucom4, BSD-3-Clause,
// copyright hap), lifted out of the MAME device framework so it builds for AVR
// and desktop. Semantics, cycle counts, interrupt vector and PC page-wrap all
// match MAME instruction-for-instruction.
//
// uPD650 specifics baked in: UCOM-43 instruction set, 2 KB program space
// (prgmask 0x7FF),
// 96x4 data RAM (datamask 0x7F, DPh 3-bit), 3-level stack, INT vector 0x003C.
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Program fetch — on AVR the 2 KB image lives in flash (PROGMEM); elsewhere it
// is a plain array. Define UCOM4_PRG_RD before including to override.
// D650_IMG_IN_RAM (combined build): the image is a user-uploaded SRAM copy
// (image_store.h), so the fetch is a plain load even on AVR — one cycle cheaper
// than the LPM it replaces.
#ifndef UCOM4_PRG_RD
#  if defined(D650_IMG_IN_RAM)
#    define UCOM4_PRG_RD(prg, addr) ((prg)[(addr)])
#  elif defined(__AVR__)
#    include <avr/pgmspace.h>
#    define UCOM4_PRG_RD(prg, addr) ((uint8_t)pgm_read_byte(&(prg)[(addr)]))
#  else
#    define UCOM4_PRG_RD(prg, addr) ((prg)[(addr)])
#  endif
#endif

// Port indices — identical to MAME (OP/IP address a port by DPl using these).
enum { UCOM4_PORTA=0, UCOM4_PORTB, UCOM4_PORTC, UCOM4_PORTD,
       UCOM4_PORTE, UCOM4_PORTF, UCOM4_PORTG, UCOM4_PORTH, UCOM4_PORTI };

typedef struct ucom4 ucom4_t;

// Host I/O. read_* return the 4-bit input on ports A/B/C/D (C/D are OR'd with the
// last output latch inside the core, matching MAME open-drain behaviour). Return
// 0 for unused ports. write_port receives port index (C..I) and the 4-bit value.
typedef struct {
  uint8_t (*read_a)(void *user);
  uint8_t (*read_b)(void *user);
  uint8_t (*read_c)(void *user);
  uint8_t (*read_d)(void *user);
  void    (*write_port)(void *user, int port, uint8_t data);
  void   *user;
} ucom4_io;

// FIELD ORDER IS LOAD-BEARING ON AVR. ldd/std reach only 63 bytes past a
// pointer, so anything further out costs an address computation instead of a
// two-clock access. With ram[128] at the front, every field was out of reach:
// the translated core's entry sequence built fifteen field addresses into a
// 60-byte frame before executing a single emulated instruction, ~1400 clocks
// per entry, measured. Everything the hot path touches now lives in the first
// 64 bytes; the cold tail (trace state, io vectors, the RAM array itself,
// which is reached through a pointer loaded once) follows.
struct ucom4 {
  uint16_t stack[3];       // 0
  uint8_t  port_out[16];   // 6
  uint8_t  acc, dpl, dph;  // 22
  uint8_t  carry_f, carry_s_f, timer_f, int_f, inte_f;  // 25
  uint8_t  op, prev_op, arg, bitmask;                   // 30
  uint8_t  datamask;       // 34   0x7F
  uint8_t  dph_mask;       // 35   0x07
  uint8_t  step_cyc;       // 36   cycles consumed by the most recent step (1..3)
  int16_t  timer_cnt;      // 37   STM countdown in machine cycles, <0 = disabled
  uint16_t pc;             // 39
  int      skip;           // 41
  const uint8_t *prg;      // 43   2 KB program
  uint16_t prgmask;        // 45   0x7FF
  int      stack_levels;   // 47   3
  int      int_line;       // 49   last /INT pin level (edge detect)

  uint16_t prev_pc;        // cold from here down
  uint32_t cyc;            // total machine cycles executed (monotonic)
  ucom4_io io;
  uint8_t  ram[128];       // 96x4 mapped in 0x00-0x4F, 0x70-0x7F; rest reads 0
};

#ifdef UCOM4_SINGLETON
// THE emulated CPU, as one object with a link-time address.
//
// The translated core reaches CPU state through this rather than through the
// pointer it is handed, and that is a measurable difference on AVR: through a
// pointer, every fixed RAM cell and every field is `base + constant`, which gcc
// hoists to the top of the function as a pile of precomputed addresses in the
// stack frame -- paid on every entry to the core whether the path taken uses
// them or not. Against a known object they are absolute two-clock loads with
// nothing to hoist. The device has exactly one emulated CPU, so nothing is lost.
//
// nat_probe refuses to run the translated core if the pointer it is given is
// not this object, so a build that gets this wrong falls back to the
// interpreter instead of corrupting a second CPU's state.
extern ucom4_t ucom4_the_cpu;
#endif

// Reset to power-on state (PC=0, interrupts disabled, ports cleared).
void ucom4_reset(ucom4_t *c, const uint8_t *prg, const ucom4_io *io);

// Execute exactly one instruction (plus a pending interrupt entry if due).
// Returns the number of machine cycles consumed (also in c->step_cyc).
uint32_t ucom4_step(ucom4_t *c);

// Execute instructions until at least `cycles` machine cycles have elapsed.
// Returns the cycles actually consumed (may overshoot by one instruction).
uint32_t ucom4_run(ucom4_t *c, uint32_t cycles);

#ifdef UCOM4_NATIVE
// Same contract as ucom4_run, but enters the statically translated core first
// and only falls back to the interpreter for an uncovered pc. Cycles must fit
// an int16 (the callers batch in the low hundreds).
uint32_t ucom4_run_native(ucom4_t *c, uint16_t cycles);
#endif

// Drive the /INT pin. Rising edge (0->1) latches the interrupt flag.
void ucom4_set_int(ucom4_t *c, int level);

#ifdef __cplusplus
}
#endif
