// d650_host.c — TR-808 machine glue. See d650_host.h.

// Hot I/O decode path; see the ucom4.c pragma note.
#if defined(SUPEROS_COMBINED) && defined(__AVR__)
#pragma GCC optimize("O3")
#endif

#include "d650_host.h"

// ---- external µPD444C RAM (4x 1024x4, IC7/IC8/IC9/IC10) ---------------------
// Service manual p.3/p.4: "The upper two bits PE2 and PE3 of CPU designate a
// RAM, and the memory select is enabled by a signal from PI-1 (CE). Cell
// addresses are designated by bits from PD, PE and PF."
//   chip    = PE3<<1 | PE2      (IC5 decodes 2-to-4)
//   address = PE1<<9 | PE0<<8 | PF<<4 | PD
static inline uint8_t ext_on(const d650_host *h) {
  return ((h->lat_i >> 1) & 1) == D650_CE_ACTIVE;             // PI1 = CE gate
}
static inline uint16_t ext_addr(const d650_host *h) {
  return (uint16_t)(((h->lat_e >> 2) & 3) << 10)              // PE2,PE3 = chip
       | (uint16_t)(((h->lat_e >> 1) & 1) << 9)               // PE1 = A9
       | (uint16_t)((h->lat_e & 1) << 8)                      // PE0 = A8
       | (uint16_t)(h->lat_f << 4)                            // PF  = A4-A7
       |  h->lat_d;                                           // PD  = A0-A3
}
static inline uint8_t ext_get(const d650_host *h, uint16_t a) {
  return (h->ext[a >> 1] >> ((a & 1) << 2)) & 0x0F;
}
static inline void ext_put(d650_host *h, uint16_t a, uint8_t v) {
  uint8_t sh  = (uint8_t)((a & 1) << 2);
  uint8_t old = h->ext[a >> 1];
  uint8_t nw  = (uint8_t)((old & (0xF0 >> sh)) | ((v & 0x0F) << sh));
  if (nw != old) { h->ext[a >> 1] = nw; h->ext_dirty = 1; }
}
// Write-transparent while WE is active and a chip is enabled: any data/address/
// strobe latch change commits the PORTC latch, like the real SRAM. The CPU only
// strobes with a stable address + data.
static void ext_refresh(d650_host *h) {
  if (((h->lat_i >> 0) & 1) != D650_WE_ACTIVE) return;
  if (!ext_on(h)) return;
  ext_put(h, ext_addr(h), h->lat_c);
}

// Selected scan row from PORTH (active-low at the CPU, one row at a time; the
// CPU raises all four bits then clears one). All-high = the STATUS group.
static inline uint8_t row_of(uint8_t ph) {
  static const uint8_t lut[16] =
    { 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0, 4 };
  return lut[ph & 0x0f];
}

// ---- ucom4 I/O callbacks (host is passed as `user`) -------------------------
static uint8_t io_read_a(void *u) {
  d650_host *h = (d650_host *)u;
  uint8_t row = row_of(h->lat_h), v = 0;
  if (row < 4) {
    for (uint8_t j = 0; j < 4; j++) v |= (uint8_t)((h->in[16 + row * 4 + j] & 1) << j);
  } else {                                   // status group
    for (uint8_t j = 0; j < 4; j++) v |= (uint8_t)((h->in[D650_IN_STATUS + j] & 1) << j);
    if (h->drv.read_clock)                   // live clock beats the snapshot
      v = (uint8_t)((v & 0x07) | ((h->drv.read_clock(h->drv.u) & 1) << 3));
  }
  return v & 0x0F;
}
static uint8_t io_read_b(void *u) {
  d650_host *h = (d650_host *)u;
  uint8_t row = row_of(h->lat_h), v = 0;
  if (row < 4) for (uint8_t j = 0; j < 4; j++) v |= (uint8_t)((h->in[row * 4 + j] & 1) << j);
  return v & 0x0F;
}
static uint8_t io_read_c(void *u) {          // µPD444C data bus read
  d650_host *h = (d650_host *)u;
  if (!ext_on(h) || ((h->lat_i & 1) == D650_WE_ACTIVE)) return 0;
  return ext_get(h, ext_addr(h));
}
static uint8_t io_read_d(void *u) { (void)u; return 0; }

static void io_write(void *u, int port, uint8_t data) {
  d650_host *h = (d650_host *)u;
  data &= 0x0F;
  switch (port) {
    case UCOM4_PORTC: h->lat_c = data; ext_refresh(h); break;  // RAM data latch
    case UCOM4_PORTD: h->lat_d = data; ext_refresh(h); break;  // A0-A3 / CH,OH,CY,CB
    case UCOM4_PORTE: h->lat_e = data; ext_refresh(h); break;  // A8,A9 + chip / CP,RS,HT,MT
    case UCOM4_PORTF: h->lat_f = data; ext_refresh(h); break;  // A4-A7 / LT,SD,BD,AC
    case UCOM4_PORTG: h->lat_g = data; break;                  // STEP LED drive
    case UCOM4_PORTH: h->lat_h = data; break;                  // scan-select
    case UCOM4_PORTI: h->lat_i = data; ext_refresh(h); break;  // WE, CE, COMMON TRIG
    default: return;
  }
  if (h->drv.port_write) h->drv.port_write(h->drv.u, port, data);
}

// ---- public API -------------------------------------------------------------
void d650_init(d650_host *h, const uint8_t *prg, const d650_drivers *drv) {
  for (unsigned i = 0; i < sizeof *h; i++) ((uint8_t *)h)[i] = 0;
  if (drv) h->drv = *drv;
  ucom4_io io = { io_read_a, io_read_b, io_read_c, io_read_d, io_write, h };
  ucom4_reset(&h->cpu, prg, &io);
}
uint32_t d650_step(d650_host *h) { return ucom4_step(&h->cpu); }
void d650_clock(d650_host *h, int level) { ucom4_set_int(&h->cpu, level); }
