// STM32F4 timers in encoder mode (register level, no HAL handles).
#if defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>

#include "board_config.h"
#include "encoders.h"

#if !defined(ODO_ENCODER_STM32_TIMER)
#error "STM32 builds need -D ODO_ENCODER_STM32_TIMER"
#endif

namespace {

struct TimerEncoderConfig {
  TIM_TypeDef* timer;
  GPIO_TypeDef* port_a;
  uint8_t pin_a;
  GPIO_TypeDef* port_b;
  uint8_t pin_b;
  uint8_t alternate_function;
};

// Digital input filter: fDTS/32 with N=8 (IC1F = IC2F = 0b1111). At an 84 MHz
// timer clock that rejects pulses shorter than ~3 us.
constexpr uint32_t kInputFilter = 0xF;

void enable_gpio_clock(GPIO_TypeDef* port) {
  if (port == GPIOA) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  } else if (port == GPIOB) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  } else if (port == GPIOC) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  }
  (void)RCC->AHB1ENR;  // read back: clock is running before the next access
}

void enable_timer_clock(TIM_TypeDef* timer) {
  if (timer == TIM1) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  } else if (timer == TIM2) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  } else if (timer == TIM3) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  } else if (timer == TIM4) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  } else if (timer == TIM5) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
  }
  (void)RCC->APB1ENR;
  (void)RCC->APB2ENR;
}

void set_alternate_function(GPIO_TypeDef* port, uint8_t pin, uint8_t af) {
  const uint32_t shift2 = static_cast<uint32_t>(pin) * 2u;
  port->MODER = (port->MODER & ~(3u << shift2)) | (2u << shift2);  // alternate function
  port->PUPDR = (port->PUPDR & ~(3u << shift2)) | (1u << shift2);  // pull-up
  const uint32_t index = pin >> 3;
  const uint32_t shift4 = (pin & 7u) * 4u;
  port->AFR[index] = (port->AFR[index] & ~(0xFu << shift4)) | (static_cast<uint32_t>(af) << shift4);
}

class TimerCounter : public odocore::HardwareCounter {
 public:
  void begin(const TimerEncoderConfig& c) {
    timer_ = c.timer;
    enable_gpio_clock(c.port_a);
    enable_gpio_clock(c.port_b);
    set_alternate_function(c.port_a, c.pin_a, c.alternate_function);
    set_alternate_function(c.port_b, c.pin_b, c.alternate_function);
    enable_timer_clock(timer_);
    timer_->CR1 = 0;
    // Encoder mode 3: count on every edge of TI1 and TI2 (4x).
    timer_->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    // CC1 / CC2 as inputs mapped to TI1 / TI2, with the input filter.
    timer_->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 | (kInputFilter << TIM_CCMR1_IC1F_Pos) |
                    (kInputFilter << TIM_CCMR1_IC2F_Pos);
    timer_->CCER = 0;  // both inputs non-inverted
    timer_->ARR = 0xFFFF;  // 16-bit wrap on every timer, including 32-bit TIM2/TIM5
    timer_->CNT = 0;
    timer_->CR1 = TIM_CR1_CEN;
  }

  int32_t read_raw() override { return static_cast<int32_t>(timer_->CNT & 0xFFFFu); }

 private:
  TIM_TypeDef* timer_ = nullptr;
};

const TimerEncoderConfig kTimers[4] = {
    {TIM1, GPIOA, 8, GPIOA, 9, 1},   // FL: PA8 / PA9, AF1
    {TIM2, GPIOA, 15, GPIOB, 3, 1},  // FR: PA15 / PB3, AF1
    {TIM4, GPIOB, 6, GPIOB, 7, 2},   // RL: PB6 / PB7, AF2
    {TIM5, GPIOA, 0, GPIOA, 1, 2},   // RR: PA0 / PA1, AF2
};

TimerCounter g_counters[4];
odocore::HardwareCounterEncoder g_hw[4] = {
    odocore::HardwareCounterEncoder(g_counters[0], 65536),
    odocore::HardwareCounterEncoder(g_counters[1], 65536),
    odocore::HardwareCounterEncoder(g_counters[2], 65536),
    odocore::HardwareCounterEncoder(g_counters[3], 65536),
};
InvertedEncoder g_wrapped[4];
odocore::Encoder* g_list[4];

}  // namespace

odocore::Encoder** encoders_begin(uint32_t now_us) {
  for (int i = 0; i < 4; ++i) {
    g_counters[i].begin(kTimers[i]);
    g_wrapped[i].bind(&g_hw[i], board::kEncoderInvert[i]);
    g_list[i] = &g_wrapped[i];
    g_list[i]->sample(now_us);
  }
  return g_list;
}

bool encoders_are_hardware() { return true; }

#endif  // ARDUINO_ARCH_STM32
