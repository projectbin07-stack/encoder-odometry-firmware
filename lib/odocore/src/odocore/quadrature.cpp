#include "odocore/quadrature.h"

namespace odocore {

namespace {

// Indexed by (from << 2) | to. 0 = no change, +1 / -1 = one step,
// 2 = illegal (both channels changed). States: 0=00, 1=01, 2=10, 3=11 (A,B).
// Forward (A leads B): 0->2, 2->3, 3->1, 1->0.
constexpr int8_t kIllegal = 2;
constexpr int8_t kTable[16] = {
    // to:  0         1         2         3
    0,        -1,       +1,       kIllegal,  // from 0
    +1,       0,        kIllegal, -1,        // from 1
    -1,       kIllegal, 0,        +1,        // from 2
    kIllegal, +1,       -1,       0,         // from 3
};

inline uint8_t state_of(bool a, bool b) {
  return static_cast<uint8_t>((a ? 2u : 0u) | (b ? 1u : 0u));
}

}  // namespace

int8_t QuadratureDecoder::transition(uint8_t from, uint8_t to, bool* illegal) {
  const int8_t step = kTable[((from & 3u) << 2) | (to & 3u)];
  const bool bad = step == kIllegal;
  if (illegal != nullptr) {
    *illegal = bad;
  }
  return bad ? 0 : step;
}

void QuadratureDecoder::reset(bool a, bool b) { state_ = state_of(a, b); }

int8_t QuadratureDecoder::update(bool a, bool b) {
  const uint8_t next = state_of(a, b);
  bool illegal = false;
  const int8_t step = transition(state_, next, &illegal);
  state_ = next;
  if (illegal) {
    ++illegal_;
    return 0;
  }
  if (step != 0) {
    count_ += static_cast<uint32_t>(static_cast<int32_t>(step));
    direction_ = step;
  }
  return step;
}

void SoftwareQuadratureEncoder::begin(bool a, bool b, uint32_t now_us) {
  decoder_.reset(a, b);
  last_edge_us_ = now_us;
}

void SoftwareQuadratureEncoder::on_pins(bool a, bool b, uint32_t now_us) {
  if (decoder_.update(a, b) != 0) {
    last_edge_us_ = now_us;
  }
}

EncoderSnapshot SoftwareQuadratureEncoder::sample(uint32_t /*now_us*/) {
  EncoderSnapshot s;
  s.count = decoder_.count();
  s.last_edge_us = last_edge_us_;
  s.illegal_transitions = decoder_.illegal_transitions();
  s.edge_time_exact = true;
  return s;
}

void CounterExtender::reset(int32_t raw, int32_t count) {
  prev_raw_ = raw;
  count_ = static_cast<uint32_t>(count);
}

int32_t CounterExtender::update(int32_t raw) {
  // |raw - prev| < 2 * modulus, which fits for any modulus up to 2^30.
  int32_t d = (raw - prev_raw_) % modulus_;  // result in (-modulus, modulus)
  const int32_t half = modulus_ / 2;
  if (d >= half) {
    d -= modulus_;
  } else if (d < -half) {
    d += modulus_;
  }
  prev_raw_ = raw;
  count_ += static_cast<uint32_t>(d);
  return d;
}

EncoderSnapshot HardwareCounterEncoder::sample(uint32_t now_us) {
  const int32_t raw = counter_.read_raw();
  if (!started_) {
    extender_.reset(raw);
    last_change_us_ = now_us;
    started_ = true;
  } else if (extender_.update(raw) != 0) {
    last_change_us_ = now_us;
  }
  EncoderSnapshot s;
  s.count = extender_.count();
  s.last_edge_us = last_change_us_;
  s.illegal_transitions = 0;  // not observable on a hardware counter
  s.edge_time_exact = false;
  return s;
}

}  // namespace odocore
