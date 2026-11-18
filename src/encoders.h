// Encoder back-ends. Exactly one is compiled in, chosen by build flag:
//
//   ODO_ENCODER_PCNT          ESP32 / ESP32-S3 pulse-counter units (hardware 4x)
//   ODO_ENCODER_ISR           pin-change interrupts + odocore 4x state table
//   ODO_ENCODER_STM32_TIMER   STM32 general-purpose timers in encoder mode
//
// All of them present odocore::Encoder, so the control loop does not care.
#pragma once

#include <odocore/quadrature.h>

/// Configure the hardware and return the four encoders (FL, FR, RL, RR).
odocore::Encoder** encoders_begin(uint32_t now_us);

/// True if the selected back-end is a hardware counter (no illegal-transition
/// counts, edge times quantised to the sample period).
bool encoders_are_hardware();

/// Applies the per-wheel `invert` flag by negating the count.
class InvertedEncoder : public odocore::Encoder {
 public:
  InvertedEncoder() = default;
  void bind(odocore::Encoder* inner, bool invert) {
    inner_ = inner;
    invert_ = invert;
  }
  odocore::EncoderSnapshot sample(uint32_t now_us) override {
    odocore::EncoderSnapshot s = inner_->sample(now_us);
    if (invert_) {
      s.count = static_cast<int32_t>(0u - static_cast<uint32_t>(s.count));
    }
    return s;
  }

 private:
  odocore::Encoder* inner_ = nullptr;
  bool invert_ = false;
};
