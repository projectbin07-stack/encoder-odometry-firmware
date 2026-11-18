// Quadrature encoder decoding.
//
// Two ways of turning A/B encoder signals into a signed count sit behind one
// interface (`Encoder`):
//
//  * `SoftwareQuadratureEncoder` - a 4x state-table decoder fed from pin-change
//    interrupts. It counts every edge of both channels, records the time of
//    the last counted edge, and counts illegal transitions (both channels
//    changing between two samples), which means an edge was missed.
//  * `HardwareCounterEncoder` - wraps a peripheral counter that already does
//    the decoding (ESP32 PCNT, STM32 timer in encoder mode). The raw counter is
//    narrow and wraps, so it is extended to 32 bits in software. Hardware
//    counters cannot report illegal transitions or exact edge times; the edge
//    time is the sample time at which a change was first seen.
//
// No Arduino headers here: everything is unit-tested on the host.
#pragma once

#include <stdint.h>

namespace odocore {

/// Wrap-safe difference `a - b` of two 32-bit counts.
///
/// Counts are kept modulo 2^32, so the difference is correct as long as the
/// true difference fits in an int32_t, even across INT32_MAX -> INT32_MIN.
inline int32_t count_delta(int32_t a, int32_t b) {
  return static_cast<int32_t>(static_cast<uint32_t>(a) - static_cast<uint32_t>(b));
}

/// Wrap-safe elapsed time `now - then` for a free-running 32-bit microsecond
/// clock (Arduino `micros()` wraps every ~71.6 minutes).
inline uint32_t elapsed_us(uint32_t now, uint32_t then) { return now - then; }

/// State of one encoder at a sampling instant.
struct EncoderSnapshot {
  int32_t count = 0;                 ///< Accumulated 4x count (wraps modulo 2^32).
  uint32_t last_edge_us = 0;         ///< Time of the most recent counted edge.
  uint32_t illegal_transitions = 0;  ///< Missed-edge events since start.
  bool edge_time_exact = false;      ///< True if last_edge_us is an edge time, not a sample time.
};

/// Anything that can be sampled for a count. Implementations must be cheap.
class Encoder {
 public:
  virtual ~Encoder() = default;
  virtual EncoderSnapshot sample(uint32_t now_us) = 0;
};

/// 4x quadrature state machine.
///
/// The state is `(A << 1) | B`. With channel A leading B the sequence is
/// 00 -> 10 -> 11 -> 01 -> 00 (states 0, 2, 3, 1), which counts up. A
/// transition in which both channels change (0 <-> 3, 1 <-> 2) cannot happen
/// if every edge is seen, so it is counted as illegal and contributes nothing
/// to the count: the direction of the missing step is unknowable.
class QuadratureDecoder {
 public:
  /// Seed the state from the current pin levels without counting.
  void reset(bool a, bool b);

  /// Feed new pin levels. Returns the count change: -1, 0 or +1.
  int8_t update(bool a, bool b);

  int32_t count() const { return static_cast<int32_t>(count_); }
  uint32_t illegal_transitions() const { return illegal_; }
  /// +1 or -1 for the direction of the last counted step, 0 before any step.
  int8_t direction() const { return direction_; }
  /// Overwrite the count (e.g. to test wraparound).
  void set_count(int32_t value) { count_ = static_cast<uint32_t>(value); }

  /// Count change for a transition between two 2-bit states, or 0 for no
  /// change. Sets `*illegal` when both bits changed.
  static int8_t transition(uint8_t from, uint8_t to, bool* illegal);

 private:
  uint8_t state_ = 0;
  uint32_t count_ = 0;  // unsigned so that overflow is defined behaviour
  uint32_t illegal_ = 0;
  int8_t direction_ = 0;
};

/// Software decoder plus edge timestamp. `on_pins` is meant to be called from
/// a pin-change interrupt on either channel; the firmware wraps `sample` in a
/// critical section.
class SoftwareQuadratureEncoder : public Encoder {
 public:
  void begin(bool a, bool b, uint32_t now_us);
  void on_pins(bool a, bool b, uint32_t now_us);
  EncoderSnapshot sample(uint32_t now_us) override;
  QuadratureDecoder& decoder() { return decoder_; }

 private:
  QuadratureDecoder decoder_;
  uint32_t last_edge_us_ = 0;
};

/// Extends a narrow, wrapping hardware counter to a 32-bit count.
///
/// `modulus` is the number of distinct raw values before the counter repeats:
/// 65536 for a 16-bit STM32 timer read as uint16, or `L` for an ESP32 PCNT
/// unit whose limits are set to -L / +L (the PCNT resets to 0 when it reaches
/// either limit, so the true count is congruent to the raw value modulo L).
/// Correct as long as fewer than modulus/2 counts happen between two reads.
class CounterExtender {
 public:
  explicit CounterExtender(int32_t modulus = 65536) : modulus_(modulus) {}
  void reset(int32_t raw, int32_t count = 0);
  /// Returns the signed count change since the previous read.
  int32_t update(int32_t raw);
  int32_t count() const { return static_cast<int32_t>(count_); }
  int32_t modulus() const { return modulus_; }

 private:
  int32_t modulus_;
  int32_t prev_raw_ = 0;
  uint32_t count_ = 0;
};

/// A peripheral counter (PCNT unit, timer in encoder mode).
class HardwareCounter {
 public:
  virtual ~HardwareCounter() = default;
  virtual int32_t read_raw() = 0;
};

/// Hardware-decoded encoder behind the common `Encoder` interface.
class HardwareCounterEncoder : public Encoder {
 public:
  HardwareCounterEncoder(HardwareCounter& counter, int32_t modulus)
      : counter_(counter), extender_(modulus) {}
  EncoderSnapshot sample(uint32_t now_us) override;

 private:
  HardwareCounter& counter_;
  CounterExtender extender_;
  bool started_ = false;
  uint32_t last_change_us_ = 0;
};

}  // namespace odocore
