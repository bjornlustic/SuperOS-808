#!/usr/bin/env python3
"""gen_native.py -- static binary translation of the program image's hot regions.

Third shape, informed by on-device measurement of the first two:
  v1  one function, computed-goto site table: address-taken labels forced the
      compiler to spill every local around every site -- SLOWER than the
      interpreter (0.79x).
  v2  one function per basic block + trampoline: registers flowed inside a
      block, but blocks averaged 3.8 instructions, so prologue/epilogue and
      trampoline dominated; the ROM's idle delay loop also straddled a range
      cut, bouncing native->interpreter every spin. 1.64-2.2x.
  v3  (this) one function per contiguous RANGE: internal control flow is plain
      goto between non-address-taken labels (registers live across whole
      loops), entry via a switch over leader pcs, exit only on leaving the
      range or exhausting the cycle budget. Skip-checks are elided where no
      predecessor can leave skip set.

The interpreter remains the fallback and the correctness oracle
(tools/d650/difftest.sh, 48M cycles in lockstep). Output is generated from the
copyrighted program image and gitignored like image_embed.h.

Usage: python3 tools/d650/gen_native.py [lo-hi ...]   (hex ranges, hi exclusive)
"""
import re, sys, os

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
IMG_H = os.path.join(ROOT, "src", "d650", "image_embed.h")
OUT_H = os.path.join(ROOT, "src", "d650", "emu", "ucom4_native.h")

txt = open(IMG_H).read()
body = txt[txt.index('{') + 1: txt.rindex('}')]
img = bytes(int(x, 16) for x in re.findall(r'0x([0-9a-fA-F]{2})', body))
assert len(img) == 2048, len(img)
PRGMASK = 0x7FF

def sum16(lo, hi):
    s = 0
    for b in img[lo:hi]:
        s = (s + b) & 0xFFFF
    return s

TWO_BYTE = {0x14, 0x15, 0x16, 0x17, 0x1E}
def length(op):
    return 2 if (op in TWO_BYTE or (op & 0xF0) == 0xA0) else 1

EXTRA = set(range(0x40, 0x50)) - {0x45}
EXTRA |= set(range(0x20, 0x24)) | set(range(0x5C, 0x60)) | set(range(0x64, 0x68))
EXTRA |= set(range(0x6C, 0x70)) | set(range(0x74, 0x78)) | set(range(0x7C, 0x80))

SKIP_SETTERS = set()
for _op in range(256):
    if _op in (0x03,0x04,0x05,0x08,0x09,0x0C,0x0D,0x0F,0x13,0x16,0x17,0x1D,0x1F,0x33,0x49):
        SKIP_SETTERS.add(_op)
    if 0x20 <= _op < 0x28 or 0x2C <= _op < 0x30 or 0x34 <= _op < 0x38 \
       or 0x3C <= _op < 0x40 or 0x50 <= _op < 0x60:
        SKIP_SETTERS.add(_op)

def wrap_next(a, n):
    return (a & 0x700) | ((a + n) & 0xFF)

def static_targets(a):
    op = img[a]; hi = op & 0xF0
    if hi == 0xA0:
        return [(((op & 7) << 8) | img[wrap_next(a, 1)]) & PRGMASK]
    if hi == 0xB0:
        return [(op & 0xF) << 2]
    if hi >= 0xC0:
        return [(wrap_next(a, 1) & ~0x3F) | (op & 0x3F)]
    if op == 0x41:
        p = wrap_next(a, 1) & ~0x3F
        return [p | (x << 2) for x in range(16)]
    return []

reach = set(); work = [0x000, 0x03C]
while work:
    a = work.pop()
    if a in reach: continue
    reach.add(a)
    for t in static_targets(a) + [wrap_next(a, length(img[a]))]:
        if t not in reach: work.append(t)

if len(sys.argv) > 1:
    RANGES = [(int(lo,16), int(hi,16)) for lo,hi in (x.split('-') for x in sys.argv[1:])]
else:
    RANGES = [(0x000,0x180),(0x2C0,0x380),(0x500,0x680),(0x700,0x740)]

def in_ranges(a): return any(lo <= a < hi for lo,hi in RANGES)
csums = [(lo,hi,sum16(lo,hi)) for lo,hi in RANGES]

leaders = {0x000, 0x03C}
for a in reach:
    for t in static_targets(a):
        leaders.add(t)
    # Return addresses are entry points too, or the RT that comes back lands
    # mid-block, the dispatch misses, and the interpreter grinds on from there
    # at ~6x the cost per cycle until it happens to reach a leader. CAL (0xA8)
    # was registered here from the start; CZP (0xBx) is a call as well and was
    # not, which quietly handed 28 return sites to the interpreter -- one of
    # them 0x11A -> 0x11B, in the middle of the hot mainline. That single
    # omission was 76% of all interpreted cycles.
    if (img[a] & 0xF8) == 0xA8:
        leaders.add(wrap_next(a, 2))
    if (img[a] & 0xF0) == 0xB0:
        leaders.add(wrap_next(a, 1))
leaders = sorted(t for t in leaders if t in reach and in_ranges(t))

def cls_of(op):
    if (op & 0xF0) == 0x90: return "0x90"
    if op == 0x31: return "0x31"
    return "0"

# ---------------------------------------------------------------------------
# v4: ONE function for every covered range.
#
# v3 measured 75 AVR clk per emulated cycle on the device. The disassembly said
# why: the mainline at 0x100 leaves its range five times in sixty instructions
# (0x2C0, 0x33C, 0x554, 0x600, 0x631), every RT leaves it as well, and each of
# those crossings is a function return plus a thirteen-field spill plus a
# re-entry through the leader switch. Measured cost of one crossing: ~1200
# clocks, against ~8 for the instruction it interrupts.
#
# So the ranges are welded into a single function. Cross-range jumps are plain
# gotos, RT re-enters through an internal `dispatch:` label instead of the
# caller, and the state stays in registers across all of it. The only exits
# left are an uncovered pc and an exhausted cycle budget.
def emit_all():
    sites = sorted(a for a in reach if in_ranges(a))
    siteset = set(sites)
    ents = list(leaders)
    need_skip = set(ents)
    for a in sites:
        if img[a] in SKIP_SETTERS:
            need_skip.add(wrap_next(a, length(img[a])))

    # --- where the LI-after-LI flag actually has to exist --------------------
    # LI loads the accumulator UNLESS the instruction before it was also an LI,
    # so `li` was maintained at every site: one store per emulated instruction,
    # ~1200 of them, for a flag that is read at 130 LI sites and whose value is
    # decided by the single preceding instruction.
    #
    # Most of those reads are static. An LI whose only predecessors are non-LI
    # is just `acc = n`. Only an LI that some other LI can fall into, or that
    # the dispatch can be entered at (where the predecessor is whatever ran
    # before the core was entered), needs to ask at run time -- 42 of 130 here.
    # So the flag is written only at the immediate predecessors of those, plus
    # at the sites whose successor is not known statically.
    def li_succs(a):
        op = img[a]; hi4 = op & 0xF0; n = length(op)
        ft = wrap_next(a, n)
        if hi4 == 0xA0 and not (op & 8):        # JMP: no fall-through
            return static_targets(a)
        if op in (0x48, 0x49):                  # RT: dynamic
            return None
        if op == 0x41:                          # indirect: dynamic-ish
            return static_targets(a)
        return static_targets(a) + [ft]

    amb = set()
    for a in sites:
        if (img[a] & 0xF0) != 0x90:
            continue
        if a in leaders:                        # entered from the dispatch
            amb.add(a); continue
        for b in sites:                         # an LI can fall into it
            if (img[b] & 0xF0) == 0x90 and wrap_next(b, length(img[b])) == a:
                amb.add(a); break
    li_live = set()
    for a in sites:
        succ = li_succs(a)
        if succ is None:
            li_live.add(a)                      # RT: target decided at run time
        elif any(t in amb for t in succ):
            li_live.add(a)
    b = []
    b.append("NAT_SEC static uint16_t nat_all(ucom4_t *nat_c_, int16_t *remp, uint16_t pc) {")
    b.append("#ifdef UCOM4_SINGLETON")
    b.append("  ucom4_t *const c = &ucom4_the_cpu;  (void)nat_c_;   /* link-time address */")
    b.append("#else")
    b.append("  ucom4_t *const c = nat_c_;")
    b.append("#endif")
    b.append("  int16_t rem = *remp; const int16_t t0 = rem; int16_t tsync = 0; (void)tsync;")
    b.append("  uint8_t *ram = c->ram; (void)ram;")
    b.append("  uint8_t acc = c->acc, dpl = c->dpl, dph = c->dph;")
    b.append("  uint8_t carry = c->carry_f, skip = (uint8_t)(c->skip != 0), inte = c->inte_f;")
    b.append("  uint8_t li = ((c->prev_op & 0xF0) == 0x90) ? 1 : 0;")
    b.append("  uint8_t pv = 0;")
    b.append("  uint16_t ret;")
    b.append("dispatch:")
    b.append("  switch (pc) {")
    for e in ents:
        b.append(f"    case 0x{e:03X}: goto L_{e:03X};")
    b.append("    default: ret = pc; goto done;")
    b.append("  }")
    for a in sites:
        op = img[a]; hi4 = op & 0xF0; n = length(op)
        arg = img[wrap_next(a,1)] if n == 2 else 0
        ft = wrap_next(a, n)
        cyc = n + (1 if op in EXTRA and hi4 < 0x80 else 0)
        bm2 = 1 << (op & 3)
        def jump_to(t, backward):
            if t in siteset:
                chk = f"if (rem <= 0) {{ ret = 0x{t:03X}; goto done; }} " if backward else ""
                return f"{{ {chk}goto L_{t:03X}; }}"
            return f"{{ ret = 0x{t:03X}; goto done; }}"
        b.append(f"L_{a:03X}: ;")
        if a == 0x0BE:
            b.append("  ucom4_idle_hits++;   /* mainline reached its wait loop */")
        b.append("#ifdef UCOM4_NATIVE_DIFF")
        b.append(f"  if (rem <= 0) {{ ret = 0x{a:03X}; goto done; }}")
        b.append("#endif")
        if a in need_skip:
            tgt = f"goto L_{ft:03X};" if ft in siteset else f"ret = 0x{ft:03X}; goto done;"
            li_clr = " li = 0;" if ft in amb else ""
            b.append(f"  if (skip) {{ skip = 0;{li_clr} pv = 0; rem -= {n}; {tgt} }}")
        body = None; flow = None
        if hi4 == 0x80:   body = f"dph = 0; dpl = {op & 0xF};"
        elif hi4 == 0x90:
            load = f"if (!li) acc = {op & 0xF};" if a in amb else f"acc = {op & 0xF};"
            body = load + (" li = 1;" if a in li_live else "")
        elif hi4 == 0xA0:
            t = (((op & 7) << 8) | arg) & PRGMASK
            body = f"NPUSH(0x{ft:03X});" if (op & 8) else ""
            flow = jump_to(t, t <= a)
        elif hi4 == 0xB0:
            body = f"NPUSH(0x{ft:03X});"
            flow = jump_to((op & 0xF) << 2, True)
        elif hi4 >= 0xC0:
            t = (wrap_next(a,1) & ~0x3F) | (op & 0x3F)
            body = ""; flow = jump_to(t, t <= a)
        elif op == 0x00: body = ""
        elif op == 0x01: body = "inte = 0;"
        elif op == 0x02: body = "NRAM_W(acc);"
        elif op == 0x03: body = "skip = (c->int_f != 0); c->int_f = 0;"
        elif op == 0x04: body = "skip = (carry != 0);"
        elif op == 0x05: body = "TSYNC(); skip = (c->timer_f != 0);"
        elif op == 0x06: body = "acc = (acc + 6) & 0xf;"
        elif op == 0x07: body = "dpl = acc;"
        elif op == 0x08: body = "acc += NRAM_R(); skip = (acc & 0x10) != 0; acc &= 0xf;"
        elif op == 0x09: body = "acc += NRAM_R() + carry; carry = (acc >> 4) & 1; acc &= 0xf; skip = (carry != 0);"
        elif op == 0x0A: body = "acc = (acc + 10) & 0xf;"
        elif op == 0x0B: body = "carry = 0;"
        elif op == 0x0C: body = "skip = (acc == NRAM_R());"
        elif op == 0x0D: body = "acc = (acc + 1) & 0xf; skip = (acc == 0);"
        elif op == 0x0E: body = "output_w(c, dpl, acc);"
        elif op == 0x0F: body = "acc = (acc - 1) & 0xf; skip = (acc == 0xf);"
        elif op == 0x10: body = "acc ^= 0xf;"
        elif op == 0x11: body = "acc = ((acc ^ 0xf) + 1) & 0xf;"
        elif op == 0x12: body = "acc = dpl;"
        elif op == 0x13: body = "dpl = (dpl - 1) & 0xf; skip = (dpl == 0xf);"
        elif op == 0x14:
            body = (f"TSYNC(); c->timer_f = 0; c->timer_cnt = {63 * ((arg & 0x3F) + 1)};"
                    f" nat_timer(c, {cyc}); tsync += {cyc};")
        elif op == 0x15: body = f"dph = {(arg >> 4) & 0xF}; dpl = {arg & 0xF};"
        elif op == 0x16: body = f"skip = (dpl == {arg & 0xF});"
        elif op == 0x17: body = f"skip = (acc == {arg & 0xF});"
        elif op == 0x18: body = "acc ^= NRAM_R();"
        elif op == 0x19: body = "acc += NRAM_R() + carry; carry = (acc >> 4) & 1; acc &= 0xf;"
        elif op == 0x1A: body = "{ uint8_t t = carry; carry = c->carry_s_f; c->carry_s_f = t; }"
        elif op == 0x1B: body = "carry = 1;"
        elif op == 0x1D: body = "{ uint8_t v = (NRAM_R() + 1) & 0xf; NRAM_W(v); skip = (v == 0); }"
        elif op == 0x1E: body = f"output_w(c, UCOM4_PORTD, {arg >> 4}); output_w(c, UCOM4_PORTC, {arg & 0xF});"
        elif op == 0x1F: body = "{ uint8_t v = (NRAM_R() - 1) & 0xf; NRAM_W(v); skip = (v == 0xf); }"
        elif op == 0x30: body = "{ uint8_t cf = acc & 1; acc = (acc >> 1) | (carry << 3); carry = cf; }"
        elif op == 0x31: body = "inte = 1;"
        elif op == 0x32: body = "acc = input_r(c, dpl);"
        elif op == 0x33: body = "dpl = (dpl + 1) & 0xf; skip = (dpl == 0);"
        elif op == 0x40: body = "acc = input_r(c, UCOM4_PORTA);"
        elif op == 0x41:
            pgt = wrap_next(a,1) & ~0x3F
            cases = " ".join(f"case {x}: " + jump_to(pgt | (x << 2), (pgt | (x << 2)) <= a)
                             for x in range(16))
            body = ""
            flow = f"switch (acc & 0xF) {{ {cases} }}"
        elif op == 0x42: body = "NREG_W(REG_Z, acc);"
        elif op == 0x43: body = "NREG_W(REG_W, acc);"
        elif op == 0x44: body = "output_w(c, UCOM4_PORTE, acc);"
        elif op == 0x46: body = "NREG_W(REG_Y, dpl);"
        elif op == 0x47: body = "NREG_W(REG_X, dph);"
        elif op in (0x48, 0x49):
            body = "skip = 1;" if op == 0x49 else ""
            # RT back through the internal dispatch, not through the caller:
            # a return used to cost a full exit + re-entry.
            flow = "{ NPOP(pc); if (rem <= 0) { ret = pc; goto done; } goto dispatch; }"
        elif op == 0x4A: body = "{ uint8_t t = acc; acc = NREG_R(REG_Z); NREG_W(REG_Z, t); }"
        elif op == 0x4B: body = "{ uint8_t t = acc; acc = NREG_R(REG_W); NREG_W(REG_W, t); }"
        elif op == 0x4C: body = "{ uint8_t t = dpl; dpl = NREG_R(REG_S); NREG_W(REG_S, t); }"
        elif op == 0x4D: body = "{ uint8_t t = dph; dph = NREG_R(REG_R); NREG_W(REG_R, t); }"
        elif op == 0x4E: body = "{ uint8_t t = dpl; dpl = NREG_R(REG_Y); NREG_W(REG_Y, t); }"
        elif op == 0x4F: body = "{ uint8_t t = dph; dph = NREG_R(REG_X); NREG_W(REG_X, t); }"
        elif 0x20 <= op < 0x24: body = f"skip = ((NREG_R(REG_F) & {bm2}) == 0);"
        elif 0x24 <= op < 0x28: body = f"skip = ((acc & {bm2}) != 0);"
        elif 0x28 <= op < 0x2C: body = f"{{ uint8_t x = acc; acc = NRAM_R(); NRAM_W(x); dph ^= {op & 3}; }}"
        elif 0x2C <= op < 0x30: body = f"{{ uint8_t x = acc; acc = NRAM_R(); NRAM_W(x); dph ^= {op & 3}; }} dpl = (dpl - 1) & 0xf; skip = (dpl == 0xf);"
        elif 0x34 <= op < 0x38: body = f"skip = ((acc & {bm2}) == (NRAM_R() & {bm2}));"
        elif 0x38 <= op < 0x3C: body = f"acc = NRAM_R(); dph ^= {op & 3};"
        elif 0x3C <= op < 0x40: body = f"{{ uint8_t x = acc; acc = NRAM_R(); NRAM_W(x); dph ^= {op & 3}; }} dpl = (dpl + 1) & 0xf; skip = (dpl == 0);"
        elif 0x50 <= op < 0x54: body = f"skip = ((input_r(c, dpl) & {bm2}) != 0);"
        elif 0x54 <= op < 0x58: body = f"skip = ((input_r(c, UCOM4_PORTA) & {bm2}) != 0);"
        elif 0x58 <= op < 0x5C: body = f"skip = ((NRAM_R() & {bm2}) != 0);"
        elif 0x5C <= op < 0x60: body = f"skip = ((NREG_R(REG_F) & {bm2}) != 0);"
        elif 0x60 <= op < 0x64: body = f"output_w(c, dpl, c->port_out[dpl] & ~{bm2}u);"
        elif 0x64 <= op < 0x68: body = f"output_w(c, UCOM4_PORTE, c->port_out[UCOM4_PORTE] & ~{bm2}u);"
        elif 0x68 <= op < 0x6C: body = f"NRAM_W(NRAM_R() & ~{bm2}u);"
        elif 0x6C <= op < 0x70: body = f"NREG_W(REG_F, NREG_R(REG_F) & ~{bm2}u);"
        elif 0x70 <= op < 0x74: body = f"output_w(c, dpl, c->port_out[dpl] | {bm2}u);"
        elif 0x74 <= op < 0x78: body = f"output_w(c, UCOM4_PORTE, c->port_out[UCOM4_PORTE] | {bm2}u);"
        elif 0x78 <= op < 0x7C: body = f"NRAM_W(NRAM_R() | {bm2}u);"
        elif 0x7C <= op < 0x80: body = f"NREG_W(REG_F, NREG_R(REG_F) | {bm2}u);"
        else: body = ""
        li_upd = "" if hi4 == 0x90 else (" li = 0;" if a in li_live else "")
        b.append(f"  {body}{li_upd} rem -= {cyc};".rstrip())
        b.append("#ifdef UCOM4_NATIVE_DIFF")
        b.append(f"  pv = {cls_of(op)};")
        b.append("#endif")
        if flow is not None:
            b.append(f"  pv = {cls_of(op)};")
            b.append(f"  {flow}")
        elif ft not in siteset:
            b.append(f"  pv = {cls_of(op)}; ret = 0x{ft:03X}; goto done;")
    b.append("done:")
    b.append("  nat_timer(c, (int16_t)(t0 - rem) - tsync);")
    b.append("  c->acc = acc; c->dpl = dpl; c->dph = dph;")
    b.append("  c->carry_f = carry; c->skip = skip; c->inte_f = inte;")
    b.append("  c->prev_op = pv; c->op = pv;")
    b.append("  *remp = rem;")
    b.append("  return ret;")
    b.append("}")
    return "\n".join(b), ents, len(sites)

o = []; w = o.append
w("// ucom4_native.h -- GENERATED by tools/d650/gen_native.py. DO NOT EDIT.")
w("// One flat function over every covered range; see the generator header.")
w("// Included by ucom4.c under UCOM4_NATIVE.")
w("")
w("#ifdef __AVR__")
w("#  include <avr/pgmspace.h>")
w("#  define NAT_SEC __attribute__((section(\".natcode\"), noinline))")
w("#else")
w("#  ifndef PROGMEM")
w("#    define PROGMEM")
w("#    define pgm_read_byte(x) (*(x))")
w("#    define pgm_read_word(x) (*(x))")
w("#  endif")
w("#  define NAT_SEC")
w("#endif")
w("")
w("extern uint16_t ucom4_idle_hits;   /* defined in ucom4.c */")
w("extern uint16_t ucom4_isr_count;")
w("#define NDM 0x7Fu")
w("static uint8_t nat_state;")
w("static void nat_probe(const ucom4_t *c) {")
w("  uint16_t s; nat_state = 1;")
w("  /* NPUSH/NPOP are open-coded for the 3-level stack and the 2 KB space. */")
w("#ifdef UCOM4_SINGLETON")
w("  if (c != &ucom4_the_cpu) { nat_state = 2; return; }")
w("#endif")
w("  if (c->stack_levels != 3 || c->prgmask != 0x7FF || c->datamask != NDM)")
w("    { nat_state = 2; return; }")
for lo,hi,cs in csums:
    w(f"  s = 0; for (uint16_t a = 0x{lo:03X}; a < 0x{hi:03X}; ++a) s = (uint16_t)(s + UCOM4_PRG_RD(c->prg, a));")
    w(f"  if (s != 0x{cs:04X}u) {{ nat_state = 2; return; }}")
w("}")
w("")
w("static void nat_timer(ucom4_t *c, int16_t d) {")
w("  if (c->timer_cnt >= 0) {")
w("    c->timer_cnt = (int16_t)(c->timer_cnt - d);")
w("    if (c->timer_cnt <= 0) { c->timer_f = 1; c->timer_cnt = -1; }")
w("  }")
w("}")
w("")
# datamask is a constant here, not c->datamask. As a runtime value every
# NREG_x index and every address mask is loop-invariant, and gcc duly hoisted
# about twenty of them into the entry frame -- recomputed on every entry to the
# core whether or not the path taken used any. nat_probe rejects an image whose
# CPU is not the 0x7F/96x4 shape these constants assume.
w("#define NRAM_A() ((uint8_t)((((uint16_t)dph << 4) | dpl) & NDM))")
w("#define NRAM_R() (ram[NRAM_A()] & 0xf)   /* the hole is held at 0; see ram_mapped */")
w("#define NRAM_W(v) do { uint8_t a_ = NRAM_A(); if (ram_mapped(a_)) ram[a_] = (v) & 0xf; } while (0)")
w("#define NREG_R(i) (ram[(uint8_t)((NDM - (i)) & 0x7F)] & 0xf)")
w("#define NREG_W(i, v) (ram[(uint8_t)((NDM - (i)) & 0x7F)] = (v) & 0xf)")
w("#define TSYNC() do { nat_timer(c, (int16_t)(t0 - rem) - tsync); tsync = (int16_t)(t0 - rem); } while (0)")
w("/* open-coded stack: nat_probe has already checked the 3-level shape */")
w("#define NPUSH(v) do { c->stack[2] = c->stack[1]; c->stack[1] = c->stack[0]; \\")
w("                      c->stack[0] = (uint16_t)(v); } while (0)")
w("#define NPOP(dst) do { (dst) = (uint16_t)(c->stack[0] & 0x7FF); \\")
w("                       c->stack[0] = c->stack[1]; c->stack[1] = c->stack[2]; } while (0)")
w("")
# NOTE: a service callback from inside the core was tried here and reverted.
# It removed the 650-clock entry per batch, but a call on every hot back edge
# made gcc spill around all ~150 of them: nat_all grew 3 KB and throughput fell
# from 97.7k to 44.8k cycles/s on the device. Paying the entry is cheaper.

w("")
code, ents, nsites = emit_all()
w(code); w("")
# entry gate as a bitmap: 2 KB of app flash for a byte-per-pc table was more
# than the app region had left, and this is read once per native entry.
ent_tbl = bytearray(256)
for e in ents: ent_tbl[e >> 3] |= 1 << (e & 7)
w("static const uint8_t nat_ent[256] PROGMEM = {")
for i in range(0, 256, 16):
    w("  " + ",".join("0x%02X" % x for x in ent_tbl[i:i+16]) + ",")
w("};")
w("#define NAT_HAS(pc) ((pgm_read_byte(&nat_ent[((pc) & 0x7FF) >> 3]) >> ((pc) & 7)) & 1)")
w("")
# noinline: gcc inlined this body into BOTH ucom4.c call sites (line 242 and
# ucom4_run_native), so it was emitted twice. Out of line it costs one rcall
# per BATCH of emulated cycles, which is nothing beside the batch itself, and
# it returns ~240 B of flash to a build that has ~270 B under the 0xD700 app
# ceiling. NOT re-measured on hardware: G5 has not been re-taken since.
w("static int16_t __attribute__((noinline))")
w("ucom4_native_run(ucom4_t *c, int16_t rem) {")
w("  uint16_t pc = c->pc;")
w("  /* interrupts arrive only between batches; one gate at entry is exact */")
w("  if (c->int_f && c->inte_f && !c->skip &&")
w("      (c->op & 0xF0) != 0x90 && c->op != 0x31) {")
w("    push_stack_pc(c, pc); pc = 0x03C; c->int_f = 0; c->inte_f = 0; rem -= 1;")
w("    ucom4_isr_count++;")
w("  }")
w("  while (rem > 0) {")
w("    if (!NAT_HAS(pc)) break;")
w("    pc = nat_all(c, &rem, pc);")
w("  }")
w("  c->pc = pc;")
w("  return rem;")
w("}")
open(OUT_H, "w").write("\n".join(o) + "\n")
print(f"gen_native: one function, {nsites} instructions, {len(ents)} entries, "
      + ", ".join(f"0x{lo:03X}-0x{hi:03X}" for lo,hi,_ in csums)
      + f" -> {os.path.relpath(OUT_H, ROOT)} ({os.path.getsize(OUT_H)} bytes)")
