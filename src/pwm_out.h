// Minimal PWM abstraction: LEDC channels on the ESP32 family, analogWrite on
// STM32 (which picks the pin's timer from the core's pin map).
#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "board_config.h"

inline void pwm_attach(uint8_t pin, uint8_t channel) {
#if defined(ARDUINO_ARCH_ESP32)
  ledcSetup(channel, board::kPwmFrequencyHz, board::kPwmResolutionBits);
  ledcAttachPin(pin, channel);
#else
  (void)channel;
  pinMode(pin, OUTPUT);
  analogWriteResolution(board::kPwmResolutionBits);
  analogWriteFrequency(board::kPwmFrequencyHz);
  analogWrite(pin, 0);
#endif
}

inline void pwm_write(uint8_t pin, uint8_t channel, uint32_t value) {
#if defined(ARDUINO_ARCH_ESP32)
  (void)pin;
  ledcWrite(channel, value);
#else
  (void)channel;
  analogWrite(pin, static_cast<int>(value));
#endif
}
