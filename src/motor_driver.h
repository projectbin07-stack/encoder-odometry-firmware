// H-bridge drivers behind one interface. The pin truth tables live in
// odocore/motor_output.* (unit-tested); these classes only write pins.
#pragma once

#include <stdint.h>

#include <odocore/motor_output.h>

#include "board_config.h"
#include "pwm_out.h"

class MotorDriver {
 public:
  virtual ~MotorDriver() = default;
  virtual void begin() = 0;
  /// Signed duty in [-1, 1]; positive drives the wheel's side forward.
  virtual void write(float duty) = 0;
};

class L298nDriver : public MotorDriver {
 public:
  L298nDriver(const board::L298nPins& pins, uint8_t pwm_channel,
              odocore::DecayMode stop_mode = odocore::DecayMode::kBrake)
      : pins_(pins), channel_(pwm_channel), stop_mode_(stop_mode) {}
  void begin() override;
  void write(float duty) override;

 private:
  board::L298nPins pins_;
  uint8_t channel_;
  odocore::DecayMode stop_mode_;
};

class DrvDriver : public MotorDriver {
 public:
  DrvDriver(const board::DrvPins& pins, uint8_t pwm_channel_in1, uint8_t pwm_channel_in2,
            odocore::DecayMode decay = odocore::DecayMode::kBrake)
      : pins_(pins), ch1_(pwm_channel_in1), ch2_(pwm_channel_in2), decay_(decay) {}
  void begin() override;
  void write(float duty) override;

 private:
  board::DrvPins pins_;
  uint8_t ch1_;
  uint8_t ch2_;
  odocore::DecayMode decay_;
};
