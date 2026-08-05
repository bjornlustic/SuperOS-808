// chase_probe.c -- run the µPD650C-085 program image on a desktop host and
// measure what its step-LED display actually does, so SuperOS can reproduce the
// stock behaviour from a measurement instead of from an impression.
//
// The emulator core is deliberately portable (src/d650/emu/*), which is what
// makes this possible: same core the firmware runs, same 1.9 ms /INT cadence,
// same stretched tempo clock the AVR bridge feeds it. The panel is a static
// input snapshot plus a small key script. No hardware involved.
//
// The display is multiplexed one column per /INT, so a LED counts as lit if it
// was driven during its column of a given ~7.6 ms frame. Anything slower than
// that shows up as whole frames present or missing.
//
// WHAT THIS ESTABLISHED (2026-08-04), and how:
//   The chase light is lit for exactly ONE 24-PPQN tempo-clock tick at the top
//   of each step. It is not a blink at some rate, and not a fixed fraction of
//   the step — but at a fixed PRE SCALE those three readings are identical, so
//   a regular clock cannot tell them apart.
//
//   The jitter option separates them. Run the tempo clock with tick periods
//   cycling 53 / 83 / 113 ms while the step stays at 666 ms, and the flash
//   follows the individual TICK (spread 68 ms). A rate-based blink or a
//   fractional one would both have sat at ~83 ms. Period-3 matters: a step is
//   8 ticks, so a two-value alternation puts every step start on the same phase
//   and the flash comes out constant either way.
//
//   Engine::chase_lit() in src/engine.h is that result.
//
// Build (the image is yours to supply; none ships with this project):
//   python3 -c "import re,sys; b=open('src/d650/image_embed.h').read(); \
//     v=[int(x,16) for x in re.findall(r'0x([0-9A-Fa-f]{2})', b.split('{',1)[1])]; \
//     open('image.bin','wb').write(bytes(v[:2048]))"
//   cc -O2 -std=c99 -Isrc/d650/emu -o chase_probe tools/d650/chase_probe.c \
//      src/d650/emu/d650_host.c src/d650/emu/ucom4.c
//
// Usage: chase_probe <image.bin> [mode_code] [bpm] [run_ms] [step_key]
//                    [quiet] [pre_raw] [jitter_ms]
//   mode_code  PH0 PA0-2 code for the MODE dial (6 = 1ST PART, see controls.h)
//   step_key   step index chorded with CLEAR to set the part's step count
//   quiet      1 = summary only, no per-frame CSV
//   pre_raw    PH1 PA0-1 code, committed with a bare CLEAR tap
//   jitter_ms  period-3 tempo-clock jitter (see above); 0 = a regular clock
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "d650_host.h"

#define NS_PER_CYC   8800ULL            // 113.636 kcyc/s (ucom4.c)
#define INT_HALF_NS  950000ULL          // 1.9 ms /INT square wave
#define FRAME_NS     7600000ULL         // 4 columns x /INT = one display frame

static d650_host H;

// --- tempo clock (the stretched pulses emu_avr.cpp feeds the CPU) -----------
static uint64_t g_now_ns = 0, g_tick_ns = 0, g_clk_fall = 0, g_tick_period_ns = 0;
static uint8_t  g_clk_lvl = 0;
// Deliberate tick-to-tick jitter. A regular clock cannot tell "the chase is lit
// for one tempo-clock tick" apart from "the chase is lit for 1/N of a step",
// because at a fixed prescale those are the same span. Alternating long and
// short tick periods separates them: a one-tick flash follows the tick it lands
// on, a fractional one stays put at the mean.
static uint64_t g_jitter_ns = 0;
static int      g_tick_par  = 0;
static uint64_t g_tick_at[8192];
static uint32_t g_tick_n = 0;

static uint8_t read_clock(void *u) { (void)u; return g_clk_lvl; }

static void clock_service(void) {
  if (g_clk_lvl && g_now_ns >= g_clk_fall) g_clk_lvl = 0;
  if (!g_clk_lvl && g_now_ns >= g_tick_ns) {
    g_clk_lvl  = 1;
    if (g_tick_n < 8192) g_tick_at[g_tick_n++] = g_now_ns;
    // Period-3, not period-2: a step is 8 ticks, so a two-value alternation puts
    // every step start on the same parity and the flash comes out constant
    // either way. 8 mod 3 = 2 walks the phase instead.
    const uint64_t p = g_tick_period_ns + (g_tick_par == 0 ? -(int64_t)g_jitter_ns
                                        : g_tick_par == 2 ?  (int64_t)g_jitter_ns : 0);
    g_clk_fall = g_now_ns + (p * 2) / 5;      // ~40% high, like the bridge
    g_tick_ns += p;
    g_tick_par = (g_tick_par + 1) % 3;
  }
}

// --- COMMON TRIG watch: is the sequencer actually stepping? ------------------
static uint32_t g_trig_count = 0;
static uint64_t g_trig_first = 0, g_trig_last = 0;
static uint8_t  g_pi2_prev = 0;
static uint64_t g_trig_ns[4096];

static void hook_port(void *u, int port, uint8_t v) {
  (void)u;
  if (port != UCOM4_PORTI) return;
  const uint8_t pi2 = (v >> 2) & 1;
  if (pi2 && !g_pi2_prev) {
    if (!g_trig_count) g_trig_first = g_now_ns;
    if (g_trig_count < 4096) g_trig_ns[g_trig_count] = g_now_ns;
    g_trig_last = g_now_ns;
    g_trig_count++;
  }
  g_pi2_prev = pi2;
}

// --- step-LED capture, binned per display frame -----------------------------
// LED n is at scan select (n >> 2), LED row (n & 3) — hw.h LightStep(). The CPU
// raises all four PH bits and clears the one it wants; PG bits high are lit.
#define MAX_FRAMES 4000
static uint32_t g_on_us[16][MAX_FRAMES];
static uint32_t g_frames = 0;

static void led_accumulate(uint64_t dt_ns) {
  const uint8_t ph = H.lat_h & 0x0F, pg = H.lat_g & 0x0F;
  int row = -1;
  for (int s = 0; s < 4; ++s) if (!((ph >> s) & 1)) { row = s; break; }
  if (row < 0 || !pg) return;
  const uint32_t fr = (uint32_t)(g_now_ns / FRAME_NS);
  if (fr >= MAX_FRAMES) return;
  if (fr > g_frames) g_frames = fr;
  for (int b = 0; b < 4; ++b)
    if ((pg >> b) & 1) g_on_us[row * 4 + b][fr] += (uint32_t)(dt_ns / 1000);
}

// --- panel ------------------------------------------------------------------
#define PA(row, col) H.in[16 + (row) * 4 + (col)]
static void pa_nibble(int row, uint8_t v) {
  for (int c = 0; c < 4; ++c) PA(row, c) = (v >> c) & 1;
}

int main(int argc, char **argv) {
  const char *img_path  = argc > 1 ? argv[1] : "image.bin";
  const int   mode_code = argc > 2 ? atoi(argv[2]) : 6;
  const double bpm      = argc > 3 ? atof(argv[3]) : 120.0;
  const uint32_t run_ms = argc > 4 ? (uint32_t)atoi(argv[4]) : 8000;
  const int   step_key  = argc > 5 ? atoi(argv[5]) : -1;   // CLEAR + this step
  const int   quiet     = argc > 6 ? atoi(argv[6]) : 0;
  const int   pre_raw   = argc > 7 ? atoi(argv[7]) : -1;   // PH1 PA0-1 raw code
  const double jitter_ms = argc > 8 ? atof(argv[8]) : 0.0;

  static uint8_t img[2048];
  FILE *f = fopen(img_path, "rb");
  if (!f) { perror(img_path); return 1; }
  if (fread(img, 1, sizeof img, f) != sizeof img) { fprintf(stderr, "short image\n"); return 1; }
  fclose(f);

  g_tick_period_ns = (uint64_t)(60.0 / bpm / 24.0 * 1e9);
  g_jitter_ns      = (uint64_t)(jitter_ms * 1e6);

  d650_drivers drv = { hook_port, NULL, read_clock };
  d650_init(&H, img, &drv);
  memset(H.in, 0, sizeof H.in);
  pa_nibble(0, (uint8_t)(mode_code & 0x07));
  pa_nibble(1, (uint8_t)(pre_raw >= 0 ? (pre_raw & 3) : 0));
  pa_nibble(2, 0); pa_nibble(3, 0);

  uint64_t next_int = 0;
  int int_lvl = 0, phase = 0;
  const uint64_t end_ns = (uint64_t)run_ms * 1000000ULL;

  while (g_now_ns < end_ns) {
    if (g_now_ns >= next_int) { int_lvl ^= 1; d650_clock(&H, int_lvl); next_int += INT_HALF_NS; }
    clock_service();

    const uint32_t ms = (uint32_t)(g_now_ns / 1000000ULL);
    // Key script: commit PRE SCALE with a bare CLEAR tap (OM p.13), set the
    // part's step count with CLEAR + a step key, enter a few steps, then START.
    if (phase == 0 && pre_raw >= 0 && ms >= 120) {            // CLEAR alone
      PA(0, 3) = 1;
      if (ms >= 260) { PA(0, 3) = 0; phase = 90; }
    } else if (phase == 90 && ms >= 300) {
      phase = 0;                                              // fall into the chord
    }
    if (phase == 0 && step_key >= 0 && ms >= 300) {           // CLEAR + step = STEP NUMBER
      PA(0, 3) = 1; H.in[step_key] = 1; phase = 1;
    } else if (phase == 1 && ms >= 700) {
      H.in[step_key] = 0; PA(0, 3) = 0; phase = 2;
    } else if (phase == 2 && ms >= 900) {                     // enter a few steps
      H.in[0] = 1; phase = 3;
    } else if (phase == 3 && ms >= 1000) {
      H.in[0] = 0; H.in[4] = 1; phase = 4;
    } else if (phase == 4 && ms >= 1100) {
      H.in[4] = 0; H.in[8] = 1; phase = 5;
    } else if (phase == 5 && ms >= 1200) {
      H.in[8] = 0; phase = 6;
    } else if (phase < 7 && ms >= 1500) {
      H.in[D650_IN_RUN] = 1; phase = 7;                       // START
    }

    const uint32_t cyc = d650_step(&H);
    const uint64_t dt  = (uint64_t)cyc * NS_PER_CYC;
    led_accumulate(dt);
    g_now_ns += dt;
  }

  printf("# mode_code=%d bpm=%.1f frames=%u\n", mode_code, bpm, g_frames);
  printf("# COMMON TRIG pulses=%u", g_trig_count);
  if (g_trig_count > 1) {
    printf("  first=%.1fms last=%.1fms  mean gap=%.1fms",
           g_trig_first / 1e6, g_trig_last / 1e6,
           (g_trig_last - g_trig_first) / 1e6 / (g_trig_count - 1));
  }
  printf("\n");
  for (int n = 0; n < 16; ++n) {
    uint32_t lit = 0, tot = 0;
    for (uint32_t fr = 0; fr <= g_frames; ++fr) { if (g_on_us[n][fr] > 200) lit++; tot += g_on_us[n][fr]; }
    printf("# led %2d  lit_frames=%4u/%u  total_on=%ums\n", n + 1, lit, g_frames + 1, tot / 1000);
  }
  if (quiet) return 0;
  printf("frame_ms");
  for (int n = 0; n < 16; ++n) printf(",led%d", n + 1);
  printf("\n");
  for (uint32_t fr = 0; fr <= g_frames; ++fr) {
    printf("%.1f", fr * (FRAME_NS / 1e6));
    for (int n = 0; n < 16; ++n) printf(",%u", g_on_us[n][fr]);
    printf("\n");
  }
  return 0;
}
