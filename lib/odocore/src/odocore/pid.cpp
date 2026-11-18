#include "odocore/pid.h"

#include <cmath>

namespace odocore {

namespace {

constexpr float kTwoPi = 6.28318530717958647692f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

}  // namespace

float slew_limit(float previous, float requested, float max_rate, float dt_s) {
  if (!(max_rate > 0.0f) || !(dt_s > 0.0f)) {
    return requested;
  }
  const float step = max_rate * dt_s;
  return clampf(requested, previous - step, previous + step);
}

void VelocityPid::reset() {
  integral_ = 0.0f;
  output_ = 0.0f;
  d_state_ = 0.0f;
  prev_measured_ = 0.0f;
  have_prev_ = false;
  limited_ = false;
}

float VelocityPid::feedforward(float target) const {
  float ff = config_.kff * target;
  if (target > 1e-6f) {
    ff += config_.ks;
  } else if (target < -1e-6f) {
    ff -= config_.ks;
  }
  return ff;
}

float VelocityPid::update(float target, float measured, float dt_s) {
  if (!(dt_s > 0.0f) || !std::isfinite(dt_s) || !std::isfinite(target) || !std::isfinite(measured)) {
    return output_;
  }
  const float limit = std::fabs(config_.output_limit);
  const float error = target - measured;

  float derivative = 0.0f;
  if (config_.kd != 0.0f && have_prev_) {
    const float raw = -(measured - prev_measured_) / dt_s;
    if (config_.d_cutoff_hz > 0.0f) {
      const float tau = 1.0f / (kTwoPi * config_.d_cutoff_hz);
      const float alpha = dt_s / (dt_s + tau);
      d_state_ += alpha * (raw - d_state_);
    } else {
      d_state_ = raw;
    }
    derivative = config_.kd * d_state_;
  }
  prev_measured_ = measured;
  have_prev_ = true;

  const float candidate_integral = integral_ + config_.ki * error * dt_s;
  const float unclipped =
      feedforward(target) + config_.kp * error + candidate_integral + derivative;
  const float saturated = clampf(unclipped, -limit, limit);
  const float out = slew_limit(output_, saturated, config_.slew_rate, dt_s);

  limited_ = out != unclipped;
  const bool pushing_further = (unclipped > out && error > 0.0f) || (unclipped < out && error < 0.0f);
  if (!(limited_ && pushing_further)) {
    integral_ = candidate_integral;
  }
  integral_ = clampf(integral_, -limit, limit);
  output_ = out;
  return output_;
}

}  // namespace odocore
