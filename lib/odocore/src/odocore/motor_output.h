// Duty cycle -> H-bridge pin states, for the two driver styles the firmware
// supports. Pure functions so the truth tables are unit-tested on the host;
// src/motor_driver.* only writes the results to pins.
//
// L298N style (one PWM enable + two direction inputs per channel):
//
//   duty > 0   IN1=1 IN2=0 EN=PWM(|duty|)   forward
//   duty < 0   IN1=0 IN2=1 EN=PWM(|duty|)   reverse
//   duty == 0  brake: IN1=0 IN2=0 EN=full  (L298 "fast motor stop")
//              coast: EN=0                 (free running)
//
// DRV style (two PWM inputs, e.g. DRV8833 / DRV8871 / TB6612 in IN-IN mode):
//
//   coast (fast) decay: forward IN1=PWM(d) IN2=0, reverse IN1=0 IN2=PWM(d)
//   brake (slow) decay: forward IN1=1 IN2=PWM(1-d), reverse IN1=PWM(1-d) IN2=1
//   duty == 0:          coast -> both 0, brake -> both full
//
// With slow decay the bridge brakes during the off-time, so the speed vs duty
// curve is more linear; with fast decay it coasts.
#pragma once

#include <stdint.h>

namespace odocore {

enum class DecayMode : uint8_t { kCoast = 0, kBrake = 1 };

struct L298nOutput {
  bool in1;
  bool in2;
  uint32_t enable_pwm;  ///< 0..max_pwm
};

struct DrvOutput {
  uint32_t in1_pwm;  ///< 0..max_pwm
  uint32_t in2_pwm;  ///< 0..max_pwm
};

/// |duty| (clamped to 1, NaN -> 0) scaled to 0..max_pwm with rounding.
uint32_t duty_to_pwm(float duty, uint32_t max_pwm);

L298nOutput l298n_output(float duty, uint32_t max_pwm, DecayMode stop_mode);
DrvOutput drv_output(float duty, uint32_t max_pwm, DecayMode decay);

}  // namespace odocore
