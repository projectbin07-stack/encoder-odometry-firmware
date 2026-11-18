#include <stdint.h>
#include <unity.h>

#include <odocore/quadrature.h>

using odocore::CounterExtender;
using odocore::EncoderSnapshot;
using odocore::HardwareCounter;
using odocore::HardwareCounterEncoder;
using odocore::QuadratureDecoder;
using odocore::SoftwareQuadratureEncoder;

void setUp() {}
void tearDown() {}

// One forward cycle with A leading B: (A,B) = 00 -> 10 -> 11 -> 01 -> 00.
static const bool kFwdA[] = {true, true, false, false};
static const bool kFwdB[] = {false, true, true, false};

static void test_forward_cycle_counts_four() {
  QuadratureDecoder d;
  d.reset(false, false);
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_EQUAL_INT8(1, d.update(kFwdA[i], kFwdB[i]));
  }
  TEST_ASSERT_EQUAL_INT32(4, d.count());
  TEST_ASSERT_EQUAL_INT8(1, d.direction());
  TEST_ASSERT_EQUAL_UINT32(0, d.illegal_transitions());
}

static void test_reverse_cycle_counts_minus_four() {
  QuadratureDecoder d;
  d.reset(false, false);
  // Reverse: 00 -> 01 -> 11 -> 10 -> 00.
  const bool a[] = {false, true, true, false};
  const bool b[] = {true, true, false, false};
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_EQUAL_INT8(-1, d.update(a[i], b[i]));
  }
  TEST_ASSERT_EQUAL_INT32(-4, d.count());
  TEST_ASSERT_EQUAL_INT8(-1, d.direction());
}

static void test_no_change_counts_nothing() {
  QuadratureDecoder d;
  d.reset(true, false);
  TEST_ASSERT_EQUAL_INT8(0, d.update(true, false));
  TEST_ASSERT_EQUAL_INT32(0, d.count());
  TEST_ASSERT_EQUAL_INT8(0, d.direction());
}

static void test_illegal_transitions_are_counted_not_guessed() {
  QuadratureDecoder d;
  d.reset(false, false);
  TEST_ASSERT_EQUAL_INT8(0, d.update(true, true));   // 00 -> 11
  TEST_ASSERT_EQUAL_INT8(0, d.update(false, false)); // 11 -> 00
  TEST_ASSERT_EQUAL_INT8(1, d.update(true, false));  // 00 -> 10 is legal again
  TEST_ASSERT_EQUAL_INT8(0, d.update(false, true));  // 10 -> 01
  TEST_ASSERT_EQUAL_UINT32(3, d.illegal_transitions());
  TEST_ASSERT_EQUAL_INT32(1, d.count());
}

static void test_full_transition_table() {
  // Every (from, to) pair: +1 for the forward sequence 0->2->3->1->0,
  // -1 for its reverse, illegal when both bits change, 0 when equal.
  const int8_t expected[4][4] = {
      {0, -1, 1, 0},
      {1, 0, 0, -1},
      {-1, 0, 0, 1},
      {0, 1, -1, 0},
  };
  const bool illegal[4][4] = {
      {false, false, false, true},
      {false, false, true, false},
      {false, true, false, false},
      {true, false, false, false},
  };
  for (uint8_t from = 0; from < 4; ++from) {
    for (uint8_t to = 0; to < 4; ++to) {
      bool bad = false;
      TEST_ASSERT_EQUAL_INT8(expected[from][to], QuadratureDecoder::transition(from, to, &bad));
      TEST_ASSERT_EQUAL(illegal[from][to], bad);
    }
  }
}

static void test_count_wraps_modulo_2_32() {
  QuadratureDecoder d;
  d.reset(false, false);
  d.set_count(INT32_MAX);
  d.update(true, false);
  TEST_ASSERT_EQUAL_INT32(INT32_MIN, d.count());
  TEST_ASSERT_EQUAL_INT32(1, odocore::count_delta(d.count(), INT32_MAX));
}

static void test_software_encoder_timestamps_counted_edges_only() {
  SoftwareQuadratureEncoder enc;
  enc.begin(false, false, 100);
  enc.on_pins(true, false, 250);   // counted edge
  enc.on_pins(true, false, 300);   // no change
  enc.on_pins(false, true, 400);   // illegal 10 -> 01
  EncoderSnapshot s = enc.sample(1000);
  TEST_ASSERT_EQUAL_INT32(1, s.count);
  TEST_ASSERT_EQUAL_UINT32(250, s.last_edge_us);
  TEST_ASSERT_EQUAL_UINT32(1, s.illegal_transitions);
  TEST_ASSERT_TRUE(s.edge_time_exact);
}

static void test_extender_16_bit_timer() {
  CounterExtender ext(65536);
  ext.reset(65530);
  TEST_ASSERT_EQUAL_INT32(11, ext.update(5));       // 65530 -> 65535 -> 0 -> 5
  TEST_ASSERT_EQUAL_INT32(11, ext.count());
  TEST_ASSERT_EQUAL_INT32(-11, ext.update(65530));  // and back
  TEST_ASSERT_EQUAL_INT32(0, ext.count());
  TEST_ASSERT_EQUAL_INT32(32767, ext.update(32761)); // largest forward step
  TEST_ASSERT_EQUAL_INT32(-32768, ext.update(65529)); // largest backward step
}

// Behaves like an ESP32 PCNT unit with limits -L/+L: the counter resets to 0
// whenever it reaches either limit.
class FakePcnt : public HardwareCounter {
 public:
  explicit FakePcnt(int32_t limit) : limit_(limit) {}
  void move(int32_t steps) {
    const int32_t dir = steps > 0 ? 1 : -1;
    for (int32_t i = 0; i != steps; i += dir) {
      raw_ += dir;
      if (raw_ >= limit_ || raw_ <= -limit_) {
        raw_ = 0;
      }
    }
  }
  int32_t read_raw() override { return raw_; }
  int32_t raw() const { return raw_; }

 private:
  int32_t limit_;
  int32_t raw_ = 0;
};

static void test_extender_follows_pcnt_resets() {
  FakePcnt pcnt(30000);
  HardwareCounterEncoder enc(pcnt, 30000);
  enc.sample(0);  // primes at raw 0
  int32_t truth = 0;
  // 1500 counts per sample forward through several resets, then back.
  for (int i = 1; i <= 50; ++i) {
    pcnt.move(1500);
    truth += 1500;
    TEST_ASSERT_EQUAL_INT32(truth, enc.sample(static_cast<uint32_t>(i) * 10000u).count);
  }
  for (int i = 0; i < 70; ++i) {
    pcnt.move(-1300);
    truth -= 1300;
    TEST_ASSERT_EQUAL_INT32(truth, enc.sample(600000u + static_cast<uint32_t>(i) * 10000u).count);
  }
  TEST_ASSERT_TRUE(truth < 0);
}

class FakeTimer : public HardwareCounter {
 public:
  int32_t value = 0;
  int32_t read_raw() override { return value & 0xFFFF; }
};

static void test_hardware_encoder_reports_sample_time_of_change() {
  FakeTimer timer;
  timer.value = 40000;
  HardwareCounterEncoder enc(timer, 65536);
  EncoderSnapshot s = enc.sample(1000);
  TEST_ASSERT_EQUAL_INT32(0, s.count);
  TEST_ASSERT_EQUAL_UINT32(1000, s.last_edge_us);
  TEST_ASSERT_FALSE(s.edge_time_exact);
  s = enc.sample(2000);  // no movement: edge time unchanged
  TEST_ASSERT_EQUAL_UINT32(1000, s.last_edge_us);
  timer.value = 40000 + 30000;  // wraps the 16-bit register
  s = enc.sample(3000);
  TEST_ASSERT_EQUAL_INT32(30000, s.count);
  TEST_ASSERT_EQUAL_UINT32(3000, s.last_edge_us);
  TEST_ASSERT_EQUAL_UINT32(0, s.illegal_transitions);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_forward_cycle_counts_four);
  RUN_TEST(test_reverse_cycle_counts_minus_four);
  RUN_TEST(test_no_change_counts_nothing);
  RUN_TEST(test_illegal_transitions_are_counted_not_guessed);
  RUN_TEST(test_full_transition_table);
  RUN_TEST(test_count_wraps_modulo_2_32);
  RUN_TEST(test_software_encoder_timestamps_counted_edges_only);
  RUN_TEST(test_extender_16_bit_timer);
  RUN_TEST(test_extender_follows_pcnt_resets);
  RUN_TEST(test_hardware_encoder_reports_sample_time_of_change);
  return UNITY_END();
}
