// SuperOS-808 - external MIDI clock smoothing (uClock-style, receiver side)
//
// Ported from SuperOS-606 uclock.h, itself from SuperOS-101 midiclock.h (the
// uClock external-sync model, github.com/midilab/uClock). Receiver side only:
// the 808's internal clock is its own TEMPO oscillator / rear sync jack, not a
// firmware timer, so there is no generator here.
//
// The problem this solves: USB-MIDI clock bytes arrive with several ms of
// host+bus jitter (and in bursts), and stepping the engine on raw arrival
// passes all of it to the audio. Tick STRUCTURE stays integer counting (every
// 0xF8 fires exactly one engine tick eventually, so the sequencer can never
// drift against the host); only edge TIMING comes from the smoothed timebase:
// an EMA of the tick period plus a gently-pulled phase estimate, snapped when
// the error exceeds half a tick (tempo jump, fresh stream).
//
// Resolution: the timebase is capped at 96 PPQN per the fleet decision (96,
// not 960): phase corrections quantize no finer than a quarter of the 24 PPQN
// tick period, which is all the 808's 24 PPQN engine can consume anyway.
#pragma once
#include <Arduino.h>

namespace uclk {

// A 0xF8 gap longer than this means the host's clock is gone (one tick at
// 10 BPM is 250 ms; any real tempo ticks far faster). Wider than the main
// loop's MCLK_TIMEOUT_MS handover window, so a stream the main loop considers
// dead is also fresh here.
static const uint32_t FRESH_GAP_US = 400000UL;

struct State {
  uint32_t tickUs   = 0;      // raw micros() of the last 0xF8
  uint32_t predUs   = 0;      // smoothed (predicted) time of the last tick
  uint32_t perUs    = 20833;  // EMA tick period in us (120 BPM until measured)
  bool     smoothOk = false;
  uint8_t  snapVote = 0;      // consecutive ticks past the snap threshold
  uint8_t  rejVote  = 0;      // consecutive tick deltas outside the EMA gate
  uint8_t  owed     = 0;      // ticks received, not yet fired to the engine
  uint32_t dueUs    = 0;      // smoothed time the oldest owed tick is due
};
inline State& S() { static State s; return s; }

// Forget phase and drop any unfired ticks. Called on MIDI Start (the next
// byte is the downbeat and must fire on arrival), and by the main loop when
// the clock handover changes hands (stale owed ticks must not fire later
// against the internal clock).
inline void Reset() {
  State& s = S();
  s.smoothOk = false;
  s.snapVote = 0;
  s.rejVote  = 0;
  s.owed     = 0;
}

// One 0xF8, at receive time (called from the RX path so the timestamp is the
// wire arrival, not the loop pass that drains it).
inline void OnClockByte(uint32_t nowUs) {
  State& s = S();
  const uint32_t dus = nowUs - s.tickUs;
  if (!s.smoothOk || dus > FRESH_GAP_US) {
    // Fresh stream (or first ever): phase unknown, seed from the wire. The
    // period estimate carries over so a resumed stream at the same tempo
    // starts smooth immediately.
    s.predUs   = nowUs;
    s.smoothOk = true;
    s.snapVote = 0;
    s.rejVote  = 0;
  } else {
    // EMA the period, gated to [per/2, 2*per): one wild delta cannot yank it.
    if (dus > (s.perUs >> 1) && dus < (s.perUs << 1)) {
      s.perUs = (uint32_t)((int32_t)s.perUs + (((int32_t)dus - (int32_t)s.perUs) >> 3));
      s.rejVote = 0;
    } else if (dus >= 1000 && dus <= 250000 && ++s.rejVote >= 3) {
      // Three deltas in a row outside the gate: the tempo really is at a
      // different order (a gated EMA can never climb across an exact 2x
      // change). Re-seed period and phase from the wire.
      s.perUs = dus;
      s.predUs = nowUs - dus;
      s.rejVote = 0;
    }
    s.predUs += s.perUs;
    const int32_t err  = (int32_t)(nowUs - s.predUs);
    const int32_t half = (int32_t)(s.perUs >> 1);
    if (err > half || err < -half) {
      // One tick that far out is a delivery burst (USB batches bytes), not a
      // tempo change: cap its pull. TWO in a row is a real jump: snap the
      // phase to the wire.
      if (++s.snapVote >= 2) { s.predUs = nowUs; s.snapVote = 0; }
      else s.predUs += (err > 0 ? half : -half) >> 2;
    } else {
      s.snapVote = 0;
      // Gentle phase pull, quantized to the 96 PPQN grid: never finer than a
      // quarter tick per correction.
      int32_t pull = err >> 2;
      const int32_t q = (int32_t)(s.perUs >> 2);   // one 96 PPQN sub-tick
      if (pull >  q) pull =  q;
      if (pull < -q) pull = -q;
      s.predUs += pull;
    }
  }
  s.tickUs = nowUs;
  if (s.owed == 0) s.dueUs = s.predUs;   // oldest owed tick is this one
  if (s.owed < 250) ++s.owed;
}

// Drain the ticks that are due. Called once per main-loop pass; returns how
// many engine ticks to fire this pass. Rules:
//   - a backlog (owed > 1, USB burst delivery) drains down to one immediately,
//     the survivor keeping its smoothed due time;
//   - the last owed tick waits for its predicted time (bounded: never more
//     than one period past its own arrival, so a mis-estimated period cannot
//     hold a tick hostage).
inline uint8_t Poll(uint32_t nowUs) {
  State& s = S();
  uint8_t n = 0;
  while (s.owed) {
    const int32_t toDue = (int32_t)(s.dueUs - nowUs);
    if (s.owed == 1 && toDue > 0 && toDue <= (int32_t)s.perUs) break;
    --s.owed;
    ++n;
    s.dueUs += s.perUs;
  }
  return n;
}

}  // namespace uclk
