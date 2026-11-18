#include "control_timer.h"

#include <Arduino.h>

namespace {
volatile uint32_t g_ticks = 0;
}  // namespace

#if defined(ARDUINO_ARCH_ESP32)

namespace {
hw_timer_t* g_timer = nullptr;
void IRAM_ATTR on_tick() { g_ticks = g_ticks + 1; }
}  // namespace

void control_timer_begin(uint32_t rate_hz) {
  // Timer 0, 80 MHz APB / 80 = 1 MHz, i.e. 1 tick per microsecond.
  g_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(g_timer, &on_tick, true);
  timerAlarmWrite(g_timer, 1000000u / rate_hz, true);
  timerAlarmEnable(g_timer);
}

#elif defined(ARDUINO_ARCH_STM32)

namespace {
// TIM11 is not used by the encoders (TIM1/2/4/5) or the motor PWM (TIM3).
HardwareTimer* g_timer = nullptr;
void on_tick() { g_ticks = g_ticks + 1; }
}  // namespace

void control_timer_begin(uint32_t rate_hz) {
  g_timer = new HardwareTimer(TIM11);
  g_timer->setOverflow(rate_hz, HERTZ_FORMAT);
  g_timer->attachInterrupt(on_tick);
  g_timer->resume();
}

#else
#error "No control timer implementation for this architecture"
#endif

uint32_t control_timer_ticks() { return g_ticks; }
