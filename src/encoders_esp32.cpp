// ESP32 / ESP32-S3 encoder back-ends: PCNT (default) or pin-change ISR.
#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>

#include "board_config.h"
#include "encoders.h"

#if defined(ODO_ENCODER_PCNT)
#include <driver/pcnt.h>

namespace {

// PCNT units reset to 0 when they reach either limit, so the raw value is the
// true count modulo kLimit (see odocore::CounterExtender).
constexpr int16_t kLimit = 30000;
// Glitch filter in APB clock cycles (80 MHz): pulses shorter than ~1.25 us
// are ignored. Must stay below 1023.
constexpr uint16_t kFilterCycles = 100;

class PcntCounter : public odocore::HardwareCounter {
 public:
  void begin(pcnt_unit_t unit, uint8_t pin_a, uint8_t pin_b) {
    unit_ = unit;
    pinMode(pin_a, INPUT_PULLUP);
    pinMode(pin_b, INPUT_PULLUP);
    // Channel 0 counts edges on A, direction from B; channel 1 counts edges
    // on B, direction from A. Together: all four edges per cycle (4x),
    // counting up when A leads B.
    pcnt_config_t ch0 = {};
    ch0.pulse_gpio_num = pin_a;
    ch0.ctrl_gpio_num = pin_b;
    ch0.channel = PCNT_CHANNEL_0;
    ch0.unit = unit;
    ch0.pos_mode = PCNT_COUNT_DEC;
    ch0.neg_mode = PCNT_COUNT_INC;
    ch0.lctrl_mode = PCNT_MODE_REVERSE;
    ch0.hctrl_mode = PCNT_MODE_KEEP;
    ch0.counter_h_lim = kLimit;
    ch0.counter_l_lim = -kLimit;
    pcnt_unit_config(&ch0);

    pcnt_config_t ch1 = ch0;
    ch1.pulse_gpio_num = pin_b;
    ch1.ctrl_gpio_num = pin_a;
    ch1.channel = PCNT_CHANNEL_1;
    ch1.pos_mode = PCNT_COUNT_INC;
    ch1.neg_mode = PCNT_COUNT_DEC;
    pcnt_unit_config(&ch1);

    pcnt_set_filter_value(unit, kFilterCycles);
    pcnt_filter_enable(unit);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
  }

  int32_t read_raw() override {
    int16_t value = 0;
    pcnt_get_counter_value(unit_, &value);
    return value;
  }

 private:
  pcnt_unit_t unit_ = PCNT_UNIT_0;
};

PcntCounter g_counters[4];
odocore::HardwareCounterEncoder g_hw[4] = {
    odocore::HardwareCounterEncoder(g_counters[0], kLimit),
    odocore::HardwareCounterEncoder(g_counters[1], kLimit),
    odocore::HardwareCounterEncoder(g_counters[2], kLimit),
    odocore::HardwareCounterEncoder(g_counters[3], kLimit),
};
InvertedEncoder g_wrapped[4];
odocore::Encoder* g_list[4];

}  // namespace

odocore::Encoder** encoders_begin(uint32_t now_us) {
  for (int i = 0; i < 4; ++i) {
    g_counters[i].begin(static_cast<pcnt_unit_t>(i), board::kEncoders[i].a, board::kEncoders[i].b);
    g_wrapped[i].bind(&g_hw[i], board::kEncoders[i].invert);
    g_list[i] = &g_wrapped[i];
    g_list[i]->sample(now_us);  // prime the extenders
  }
  return g_list;
}

bool encoders_are_hardware() { return true; }

#elif defined(ODO_ENCODER_ISR)

namespace {

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// Pin-change interrupts on both channels feed the software 4x decoder. At very
// high edge rates the ISR can fall behind; the decoder then sees both pins
// changed and counts an illegal transition, which shows up in DIAGNOSTICS.
class IsrEncoder : public odocore::Encoder {
 public:
  void begin(uint8_t pin_a, uint8_t pin_b, uint32_t now_us) {
    a_ = pin_a;
    b_ = pin_b;
    pinMode(a_, INPUT_PULLUP);
    pinMode(b_, INPUT_PULLUP);
    core_.begin(digitalRead(a_) == HIGH, digitalRead(b_) == HIGH, now_us);
    attachInterruptArg(a_, &IsrEncoder::on_change, this, CHANGE);
    attachInterruptArg(b_, &IsrEncoder::on_change, this, CHANGE);
  }

  odocore::EncoderSnapshot sample(uint32_t now_us) override {
    portENTER_CRITICAL(&g_mux);
    const odocore::EncoderSnapshot s = core_.sample(now_us);
    portEXIT_CRITICAL(&g_mux);
    return s;
  }

  static void IRAM_ATTR on_change(void* arg) {
    IsrEncoder* self = static_cast<IsrEncoder*>(arg);
    const bool a = digitalRead(self->a_) == HIGH;
    const bool b = digitalRead(self->b_) == HIGH;
    const uint32_t now = micros();
    portENTER_CRITICAL_ISR(&g_mux);
    self->core_.on_pins(a, b, now);
    portEXIT_CRITICAL_ISR(&g_mux);
  }

 private:
  odocore::SoftwareQuadratureEncoder core_;
  uint8_t a_ = 0;
  uint8_t b_ = 0;
};

IsrEncoder g_isr[4];
InvertedEncoder g_wrapped[4];
odocore::Encoder* g_list[4];

}  // namespace

odocore::Encoder** encoders_begin(uint32_t now_us) {
  for (int i = 0; i < 4; ++i) {
    g_isr[i].begin(board::kEncoders[i].a, board::kEncoders[i].b, now_us);
    g_wrapped[i].bind(&g_isr[i], board::kEncoders[i].invert);
    g_list[i] = &g_wrapped[i];
  }
  return g_list;
}

bool encoders_are_hardware() { return false; }

#else
#error "ESP32 builds need -D ODO_ENCODER_PCNT or -D ODO_ENCODER_ISR"
#endif

#endif  // ARDUINO_ARCH_ESP32
