// ucom4_hle.h -- native execution of this program image's interrupt routine.
//
// Measured on the device with UCOM4_PC_PROFILE: about 93% of every instruction
// the interpreter runs is one ISR (entry 0x03C, body 0x080-0x0BF). Interpreting
// it costs ~141 AVR clocks per emulated machine cycle, which is why a divided
// core cannot keep up: at 4 MHz there are only 35 clocks per machine cycle to
// spend. Running that one routine natively removes ~93% of the interpreted work
// instead of the 40% an equivalent effort bought on the 101.
//
// The routine is the display multiplex and the panel scan. Its shape:
//
//   prologue  save H,L,A into R,S,Z; park PORTH high; sample PORTA; set F bit 3
//             IF that sample has bit 1 set; stash the sample in RAM[0x0B]; read
//             a slot counter from RAM[0x0F]; JPA on it
//   slot n    RAM[n] -> PORTG (the LED rows for this column)
//             PORTH bit n low   (drive scan column n)
//             if RAM[4+n] == 0, sample PORTB into it, else zero it
//             sample PORTA into RAM[0x10+n]
//             set the slot counter to the next slot
//   epilogue  restore A,L,H; EI; RT
//
// The counter walks 3, 11, 9, 10 for slots 0..3, which JPA turns into
// 0x040 | (counter << 2). The key read is a handshake, not a debounce: the slot
// only samples PORTB when the previous sample has been consumed (zeroed) by the
// main program, and otherwise clears the cell.
//
// EXACTNESS. This must be indistinguishable from interpreting the routine, in
// architectural state AND in machine cycles, because the host's clock stretcher
// and the emulated timer are both driven off the cycle count. Cycle totals below
// were derived from the interpreter's own per-handler step_cyc, including the
// two-cycle handlers (IA, JPA, RT, XAZ, XLS, XHR, SFB) and the two-byte LDI.
//
// SAFETY. Everything here is specific to one program image, so it is gated on a
// checksum of the routine's own bytes. A different image, or a corrupted one,
// falls back to interpretation rather than running code written for something
// else. The slot counter is also validated before any state is touched: an
// unexpected value declines the routine instead of guessing.
#ifndef UCOM4_HLE_H
#define UCOM4_HLE_H

#define HLE_ISR_LO   0x03Cu
#define HLE_ISR_HI   0x0C0u
#define HLE_ISR_SUM  0x2C98u   /* sum16 of [LO,HI) in the image this was written for */

static uint8_t hle_state;      /* 0 unprobed, 1 usable, 2 not this image */

static void hle_probe(const ucom4_t *c) {
  uint16_t sum = 0;
  for (uint16_t a = HLE_ISR_LO; a < HLE_ISR_HI; ++a)
    sum = (uint16_t)(sum + UCOM4_PRG_RD(c->prg, a));
  hle_state = (sum == HLE_ISR_SUM) ? 1u : 2u;
}

// JPA lands at 0x040 | (counter << 2). Slots 0 and 1 land directly on their
// body; slots 2 and 3 land on a JMP trampoline that costs two more cycles.
// Cycle costs are the interpreter's own, two-cycle handlers included.
static const uint8_t hle_slot_of[16] = { 0,0,0,0, 0,0,0,0, 0,2,3,1, 0,0,0,0 };
static const uint8_t hle_next_ctr[4] = { 11, 9, 10, 3 };
static const uint8_t hle_pad_cyc[4]  = { 0, 0, 2, 2 };
static const uint8_t hle_body_smp[4] = { 24, 24, 24, 22 };  /* PORTB sampled */
static const uint8_t hle_body_clr[4] = { 20, 20, 20, 18 };  /* cell cleared   */
#define HLE_PROLOGUE_CYC  19u   /* 0x03C..0x04A */
#define HLE_SFB_CYC        2u   /* 0x045, only when the PORTA sample has bit 1 */
#define HLE_EPILOGUE_CYC   9u   /* 0x0B1..0x0B5 */

// Run the interrupt routine natively. Returns machine cycles consumed, or 0 if
// this image or this state is not one it can serve, in which case the caller
// must interpret as usual. The caller has already accounted the one cycle the
// interrupt entry itself costs.
static uint8_t hle_isr(ucom4_t *c) {
  if (hle_state != 1u) return 0;

  uint8_t *ram = c->ram;
  const uint8_t dm = c->datamask;

  // The slot counter is read by LDZ 15 / LM 0, so its address does not depend
  // on the register swaps above it and can be checked before anything moves.
  const uint8_t ctr = (uint8_t)(ram[0x0F & dm] & 0x0F);
  if (ctr != 3u && ctr != 9u && ctr != 10u && ctr != 11u) return 0;
  const uint8_t n    = hle_slot_of[ctr];
  const uint8_t next = hle_next_ctr[n];

  #define R_(i)     (ram[(uint8_t)(dm - (i)) & 0x7F] & 0x0F)
  #define W_(i, v)  (ram[(uint8_t)(dm - (i)) & 0x7F] = (uint8_t)((v) & 0x0F))

  uint8_t acc = c->acc, dpl = c->dpl, dph = c->dph, t;

  // ---- prologue 0x03C..0x04A ------------------------------------------------
  uint8_t cyc = (uint8_t)(HLE_PROLOGUE_CYC + hle_pad_cyc[n]);
  t = dph; dph = (uint8_t)R_(REG_R); W_(REG_R, t);          // XHR
  t = dpl; dpl = (uint8_t)R_(REG_S); W_(REG_S, t);          // XLS
  t = acc; acc = (uint8_t)R_(REG_Z); W_(REG_Z, t);          // XAZ
  acc = 15;                                                 // LI 15
  dph = 0; dpl = 7;                                         // LDZ 7
  output_w(c, 7, acc);                                      // OP   PORTH = 0xF
  acc = input_r(c, UCOM4_PORTA);                            // IA
  // TAB 1 skips the JCP when bit 1 is set, which lands on SFB. When it is
  // clear the JCP runs and jumps PAST SFB. So the flag is conditional, and the
  // two paths differ by SFB's two cycles.
  if (acc & 2u) {                                           // TAB 1 ; JCP ; SFB 3
    W_(REG_F, (uint8_t)(R_(REG_F) | 8u));
    cyc = (uint8_t)(cyc + HLE_SFB_CYC);
  }
  dph = 0; dpl = 11; ram[0x0B & dm] = (uint8_t)(acc & 0x0F);// LDZ 11 ; S
  dph = 0; dpl = 15;                                        // LDZ 15
  acc = (uint8_t)(ram[0x0F & dm] & 0x0F);                   // LM 0 ; JPA

  // ---- slot body -----------------------------------------------------------
  acc = (uint8_t)(ram[n & dm] & 0x0F);                      // LDZ n ; LM 0
  output_w(c, 6, acc);                                      // LDZ 6 ; OP  PORTG
  output_w(c, 7, (uint8_t)(c->port_out[7] & ~(1u << n)));   // LDZ 7 ; RPB n
  if ((ram[(4u + n) & dm] & 0x0F) == 0u) {                  // LI 0 ; CM skips
    acc = input_r(c, 1);                                    // LDZ 1 ; IP  PORTB
    cyc = (uint8_t)(cyc + hle_body_smp[n]);
  } else {                                                  // CM did not skip
    acc = 0;                                                // JCP straight to S
    cyc = (uint8_t)(cyc + hle_body_clr[n]);
  }
  ram[(4u + n) & dm] = (uint8_t)(acc & 0x0F);               // S
  acc = input_r(c, UCOM4_PORTA);                            // LDI 1n ; IA
  ram[(0x10u + n) & dm] = (uint8_t)(acc & 0x0F);            // S
  dph = 0; dpl = 15;                                        // LDZ 15
  acc = next;                                               // LI next
  ram[0x0F & dm] = next;                                    // S  (each slot
                                                            // stores it once)

  // ---- epilogue 0x0B1..0x0B5 ------------------------------------------------
  t = acc; acc = (uint8_t)R_(REG_Z); W_(REG_Z, t);          // XAZ
  t = dpl; dpl = (uint8_t)R_(REG_S); W_(REG_S, t);          // XLS
  t = dph; dph = (uint8_t)R_(REG_R); W_(REG_R, t);          // XHR
  c->inte_f = 1;                                            // EI
  c->pc = pop_stack(c);                                     // RT
  cyc = (uint8_t)(cyc + HLE_EPILOGUE_CYC);

  #undef R_
  #undef W_

  c->acc = acc; c->dpl = dpl; c->dph = dph;
  c->op = 0x48;          // RT, what the interrupt test sees on the next pass
  c->prev_op = 0x31;     // EI
  c->arg = (uint8_t)(0x10u + n);   // the LDI the slot fetched
  c->skip = 0;
  return cyc;
}

#endif /* UCOM4_HLE_H */
