#include <math.h>
#include <stdint.h>
#include <unity.h>

#include <odocore/velocity.h>

using odocore::EncoderSnapshot;
using odocore::VelocityConfig;
using odocore::VelocityEstimator;

void setUp() {}
void tearDown() {}

// Ideal encoder at constant speed `cps`: edge k happens at t0 + k/cps seconds.
// `t_us` is the time since t0; counts and times wrap like the firmware's.
static EncoderSnapshot constant_speed(double cps, uint32_t t_us, int32_t count0 = 0,
                                      uint32_t t0 = 0) {
  const double n = floor(cps * static_cast<double>(t_us) * 1e-6 + 1e-9);
  EncoderSnapshot s;
  s.count = static_cast<int32_t>(static_cast<uint32_t>(count0) + static_cast<uint32_t>(n));
  s.last_edge_us = t0 + (n > 0 ? static_cast<uint32_t>(llround(n / cps * 1e6)) : 0u);
  s.edge_time_exact = true;
  return s;
}

static void test_low_speed_period_exact_fixed_quantised() {
  // 50 counts/s at a 100 Hz loop: fixed-period can only read 0 or 100
  // counts/s, the period (1/T) estimate is exactly 50 and the blend uses it.
  VelocityEstimator est;
  bool saw_zero = false;
  bool saw_hundred = false;
  for (uint32_t k = 0; k <= 100; ++k) {
    const uint32_t t = k * 10000u;
    est.update(constant_speed(50.0, t), t);
    if (k > 20) {
      TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f, est.period_cps());
      TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f, est.blended_cps());
      TEST_ASSERT_EQUAL_FLOAT(0.0f, est.blend_weight());
      saw_zero |= est.fixed_cps() == 0.0f;
      saw_hundred |= fabsf(est.fixed_cps() - 100.0f) < 1e-3f;
    }
  }
  TEST_ASSERT_TRUE(saw_zero);
  TEST_ASSERT_TRUE(saw_hundred);
}

static void test_very_low_speed_two_counts_per_second() {
  // 2 counts/s: one edge every 500 ms, far slower than the 250 ms zero
  // timeout. Between edges the estimate is bounded by 1/(time since edge) and
  // drops to zero after the timeout; it must never exceed the true speed by
  // more than that bound and never change sign.
  VelocityConfig cfg;
  cfg.zero_timeout_us = 1000000u;  // allow slow speeds to be measured
  VelocityEstimator est(cfg);
  for (uint32_t k = 0; k <= 300; ++k) {
    const uint32_t t = k * 10000u;
    est.update(constant_speed(2.0, t), t);
    TEST_ASSERT_TRUE(est.blended_cps() >= 0.0f);
    TEST_ASSERT_TRUE(est.blended_cps() <= 2.0f + 1e-3f);
  }
  // At t = 3.0 s an edge has just been seen (edge 6 at 3.0 s).
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, est.period_cps());
}

static void test_high_speed_uses_fixed_period() {
  VelocityEstimator est;
  for (uint32_t k = 0; k < 20; ++k) {
    const uint32_t t = k * 10000u;
    est.update(constant_speed(100000.0, t), t);
  }
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 100000.0f, est.fixed_cps());
  TEST_ASSERT_EQUAL_FLOAT(1.0f, est.blend_weight());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 100000.0f, est.blended_cps());
}

static void test_very_high_speed_two_million_counts_per_second() {
  // 2e6 counts/s (e.g. a 2000-count encoder at 60000 rpm) still fits easily
  // in the 32-bit delta of one period.
  VelocityEstimator est;
  for (uint32_t k = 0; k < 5; ++k) {
    const uint32_t t = k * 10000u;
    est.update(constant_speed(2.0e6, t), t);
  }
  TEST_ASSERT_FLOAT_WITHIN(10.0f, 2.0e6f, est.blended_cps());
}

static void test_reverse_direction_is_negative() {
  VelocityEstimator est;
  for (uint32_t k = 0; k < 20; ++k) {
    const uint32_t t = k * 10000u;
    EncoderSnapshot s = constant_speed(5000.0, t);
    s.count = -s.count;
    est.update(s, t);
  }
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -5000.0f, est.blended_cps());
}

static void test_count_and_clock_wraparound() {
  // Start 1 ms before micros() wraps and 500 counts below INT32_MAX, at
  // 20000 counts/s (200 per period); both wraps happen in the first periods.
  const uint32_t t0 = 0xFFFFFFFFu - 1000u;
  const int32_t c0 = INT32_MAX - 500;
  VelocityEstimator est;
  for (uint32_t k = 0; k < 10; ++k) {
    const uint32_t dt = k * 10000u;
    est.update(constant_speed(20000.0, dt, c0, t0), t0 + dt);
    if (k >= 2) {
      TEST_ASSERT_FLOAT_WITHIN(0.5f, 20000.0f, est.fixed_cps());
      TEST_ASSERT_FLOAT_WITHIN(20.0f, 20000.0f, est.period_cps());
    }
  }
}

static void test_stale_period_is_bounded_then_zeroed() {
  VelocityEstimator est;  // zero timeout 250 ms
  for (uint32_t k = 0; k < 30; ++k) {
    const uint32_t t = k * 10000u;
    est.update(constant_speed(50.0, t), t);
  }
  const EncoderSnapshot last = constant_speed(50.0, 290000u);
  // 100 ms after the last edge the speed is at most 1e6 / 100e3 = 10 counts/s.
  est.update(last, last.last_edge_us + 100000u);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, est.period_cps());
  est.update(last, last.last_edge_us + 300000u);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, est.period_cps());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, est.blended_cps());
}

static void test_dither_on_one_edge_reads_zero() {
  VelocityEstimator est;
  EncoderSnapshot s;
  s.count = 10;
  s.last_edge_us = 0;
  est.update(s, 0);
  s.count = 11;
  s.last_edge_us = 5000;
  est.update(s, 10000);
  s.count = 10;  // back across the same edge
  s.last_edge_us = 15000;
  est.update(s, 20000);
  s.count = 11;
  s.last_edge_us = 25000;
  est.update(s, 30000);
  s.count = 10;
  s.last_edge_us = 35000;
  est.update(s, 40000);
  // Back and forth across one edge: each edge-to-edge reading is one count in
  // 10 ms, alternating in sign, so the magnitude is 100 counts/s.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, fabsf(est.period_cps()));
  est.update(s, 45000);
  // A new edge that leaves the count where the previous edge left it means
  // the shaft is dithering on an edge: the period estimate reads zero.
  s.count = 10;
  s.last_edge_us = 47000;
  est.update(s, 50000);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, est.period_cps());
}

static void test_blend_weight_is_linear_between_thresholds() {
  VelocityConfig cfg;
  cfg.low_cps = 200.0f;
  cfg.high_cps = 800.0f;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, VelocityEstimator::weight_for(150.0f, cfg));
  TEST_ASSERT_EQUAL_FLOAT(0.5f, VelocityEstimator::weight_for(500.0f, cfg));
  TEST_ASSERT_EQUAL_FLOAT(0.5f, VelocityEstimator::weight_for(-500.0f, cfg));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, VelocityEstimator::weight_for(900.0f, cfg));
  cfg.high_cps = cfg.low_cps;  // degenerate: hard switch
  TEST_ASSERT_EQUAL_FLOAT(0.0f, VelocityEstimator::weight_for(199.0f, cfg));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, VelocityEstimator::weight_for(200.0f, cfg));
}

static void test_hardware_counter_timing_at_low_speed() {
  // A hardware counter only tells us the sample time at which the count
  // changed. At 50 counts/s and 100 Hz a change is seen every second
  // sample, so the period estimate is 1 count / 20 ms = 50 counts/s.
  VelocityEstimator est;
  for (uint32_t k = 0; k <= 50; ++k) {
    const uint32_t t = k * 10000u;
    EncoderSnapshot s;
    s.count = static_cast<int32_t>(k / 2);
    s.last_edge_us = (k / 2) * 20000u;
    s.edge_time_exact = false;
    est.update(s, t);
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f, est.period_cps());
}

static void test_reset_reprimes() {
  VelocityEstimator est;
  for (uint32_t k = 0; k < 10; ++k) {
    est.update(constant_speed(5000.0, k * 10000u), k * 10000u);
  }
  est.reset();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, est.blended_cps());
  // The first update after reset only primes, even after a large jump.
  EncoderSnapshot s;
  s.count = 1000000;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, est.update(s, 200000u));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_low_speed_period_exact_fixed_quantised);
  RUN_TEST(test_very_low_speed_two_counts_per_second);
  RUN_TEST(test_high_speed_uses_fixed_period);
  RUN_TEST(test_very_high_speed_two_million_counts_per_second);
  RUN_TEST(test_reverse_direction_is_negative);
  RUN_TEST(test_count_and_clock_wraparound);
  RUN_TEST(test_stale_period_is_bounded_then_zeroed);
  RUN_TEST(test_dither_on_one_edge_reads_zero);
  RUN_TEST(test_blend_weight_is_linear_between_thresholds);
  RUN_TEST(test_hardware_counter_timing_at_low_speed);
  RUN_TEST(test_reset_reprimes);
  return UNITY_END();
}
