#include "odocore/motor_output.h"

#include <cmath>

namespace odocore {

uint32_t duty_to_pwm(float duty, uint32_t max_pwm) {
  if (!std::isfinite(duty)) {
    return 0;
  }
  float magnitude = std::fabs(duty);
  if (magnitude > 1.0f) {
    magnitude = 1.0f;
  }
  return static_cast<uint32_t>(std::lround(magnitude * static_cast<float>(max_pwm)));
}

L298nOutput l298n_output(float duty, uint32_t max_pwm, DecayMode stop_mode) {
  const uint32_t pwm = duty_to_pwm(duty, max_pwm);
  if (pwm == 0) {
    if (stop_mode == DecayMode::kBrake) {
      return L298nOutput{false, false, max_pwm};
    }
    return L298nOutput{false, false, 0};
  }
  if (duty > 0.0f) {
    return L298nOutput{true, false, pwm};
  }
  return L298nOutput{false, true, pwm};
}

DrvOutput drv_output(float duty, uint32_t max_pwm, DecayMode decay) {
  const uint32_t pwm = duty_to_pwm(duty, max_pwm);
  if (pwm == 0) {
    return decay == DecayMode::kBrake ? DrvOutput{max_pwm, max_pwm} : DrvOutput{0, 0};
  }
  const bool forward = duty > 0.0f;
  if (decay == DecayMode::kCoast) {
    return forward ? DrvOutput{pwm, 0} : DrvOutput{0, pwm};
  }
  const uint32_t inverted = max_pwm - pwm;
  return forward ? DrvOutput{max_pwm, inverted} : DrvOutput{inverted, max_pwm};
}

}  // namespace odocore
