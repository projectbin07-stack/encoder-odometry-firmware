// Per-wheel velocity controller.
//
// Arithmetic choice: single-precision float. Every supported MCU has a
// hardware single-precision FPU (ESP32 / ESP32-S3 Xtensa LX6/LX7 and the
// Cortex-M4F in the STM32F401), so float costs a few cycles per operation and
// avoids the scaling and overflow analysis a fixed-point (Q15/Q16.16)
// controller would need. A port to an FPU-less part (Cortex-M0, AVR) would be
// the time to revisit this.
//
// Output is a signed duty cycle in [-output_limit, +output_limit] where 1.0 is
// full PWM.
//
//   u = kff * target + ks * sign(target)       feed-forward
//     + kp * e + I + kd * d(-measured)/dt       feedback, e = target - measured
//
// * Feed-forward: `kff` is the inverse motor velocity constant (duty per
//   rad/s at steady state), `ks` the duty that just overcomes static friction.
// * Derivative acts on the measurement (no kick on set-point steps) through a
//   first-order low-pass filter with cut-off `d_cutoff_hz` (0 = unfiltered).
// * Anti-windup by conditional integration: the integrator is not advanced
//   while the output is clipped (by saturation or by the slew limiter) and the
//   error would push it further into the clip. The integrator is also clamped
//   to +-output_limit.
// * Slew limiting: the output changes by at most `slew_rate * dt` per update
//   (duty per second, 0 disables). It protects gearboxes and keeps current
//   spikes down on the H-bridge.
#pragma once

#include <stdint.h>

namespace odocore {

struct PidConfig {
  float kp = 0.02f;           ///< duty per (rad/s) of error
  float ki = 0.2f;            ///< duty per (rad/s * s) of accumulated error
  float kd = 0.0f;            ///< duty per (rad/s^2)
  float kff = 0.03f;          ///< feed-forward, duty per rad/s of target
  float ks = 0.0f;            ///< static-friction feed-forward, duty
  float output_limit = 1.0f;  ///< |duty| ceiling, 0..1
  float slew_rate = 4.0f;     ///< max duty change per second, 0 = unlimited
  float d_cutoff_hz = 20.0f;  ///< derivative low-pass cut-off, 0 = none
};

class VelocityPid {
 public:
  explicit VelocityPid(const PidConfig& config = PidConfig()) : config_(config) {}

  void configure(const PidConfig& config) { config_ = config; }
  const PidConfig& config() const { return config_; }

  /// Clear integrator, derivative history and output (output returns to 0).
  void reset();

  /// One control update. `target` and `measured` in rad/s, `dt_s` in seconds.
  /// A non-positive or non-finite dt returns the previous output unchanged.
  float update(float target, float measured, float dt_s);

  float feedforward(float target) const;
  float output() const { return output_; }
  float integral() const { return integral_; }
  /// True if the last update was clipped by the output limit or the slew limit.
  bool limited() const { return limited_; }

 private:
  PidConfig config_;
  float integral_ = 0.0f;
  float output_ = 0.0f;
  float d_state_ = 0.0f;
  float prev_measured_ = 0.0f;
  bool have_prev_ = false;
  bool limited_ = false;
};

/// Rate limiter for any scalar, used for the output and available separately.
float slew_limit(float previous, float requested, float max_rate, float dt_s);

}  // namespace odocore
