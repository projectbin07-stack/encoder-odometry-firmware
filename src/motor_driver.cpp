#include "motor_driver.h"

#include <Arduino.h>

void L298nDriver::begin() {
  pinMode(pins_.in1, OUTPUT);
  pinMode(pins_.in2, OUTPUT);
  digitalWrite(pins_.in1, LOW);
  digitalWrite(pins_.in2, LOW);
  pwm_attach(pins_.en, channel_);
  pwm_write(pins_.en, channel_, 0);
}

void L298nDriver::write(float duty) {
  const odocore::L298nOutput out =
      odocore::l298n_output(pins_.invert ? -duty : duty, board::kPwmMax, stop_mode_);
  digitalWrite(pins_.in1, out.in1 ? HIGH : LOW);
  digitalWrite(pins_.in2, out.in2 ? HIGH : LOW);
  pwm_write(pins_.en, channel_, out.enable_pwm);
}

void DrvDriver::begin() {
  pwm_attach(pins_.in1, ch1_);
  pwm_attach(pins_.in2, ch2_);
  pwm_write(pins_.in1, ch1_, 0);
  pwm_write(pins_.in2, ch2_, 0);
}

void DrvDriver::write(float duty) {
  const odocore::DrvOutput out =
      odocore::drv_output(pins_.invert ? -duty : duty, board::kPwmMax, decay_);
  pwm_write(pins_.in1, ch1_, out.in1_pwm);
  pwm_write(pins_.in2, ch2_, out.in2_pwm);
}
