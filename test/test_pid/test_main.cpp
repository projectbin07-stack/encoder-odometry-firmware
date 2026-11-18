#include <math.h>
#include <stdint.h>
#include <unity.h>

#include <odocore/pid.h>
#include <odocore/watchdog.h>

using odocore::CommandWatchdog;
using odocore::PidConfig;
using odocore::VelocityPid;

void setUp() {}
void tearDown() {}

static PidConfig plain(float kp, float ki, float kd = 0.0f) {
  PidConfig c;
  c.kp = kp;
  c.ki = ki;
  c.kd = kd;
  c.kff = 0.0f;
  c.ks = 0.0f;
  c.output_limit = 1.0f;
  c.slew_rate = 0.0f;
  c.d_cutoff_hz = 0.0f;
  return c;
}

static void test_feedforward_terms() {
  PidConfig c = plain(0.0f, 0.0f);
  c.kff = 0.03f;
  c.ks = 0.05f;
  VelocityPid pid(c);
  // u = 0.03 * 10 + 0.05 = 0.35, sign follows the target, 0 at rest.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.35f, pid.update(10.0f, 0.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.35f, pid.update(-10.0f, 0.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.update(0.0f, 0.0f, 0.01f));
}

static void test_pi_by_hand() {
  VelocityPid pid(plain(0.1f, 2.0f));
  // e = 1: P = 0.1, I = 2 * 1 * 0.01 = 0.02 -> 0.12; next step I = 0.04 -> 0.14.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.12f, pid.update(1.0f, 0.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.14f, pid.update(1.0f, 0.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.04f, pid.integral());
  // Error zero: output is the integral alone.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.04f, pid.update(1.0f, 1.0f, 0.01f));
}

static void test_derivative_on_measurement_no_setpoint_kick() {
  VelocityPid pid(plain(0.0f, 0.0f, 0.01f));
  pid.update(0.0f, 0.0f, 0.01f);
  // Set-point step with constant measurement: derivative term stays 0.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.update(10.0f, 0.0f, 0.01f));
  // Measurement rises by 1 in 10 ms: d = -kd * 100 = -1 (unfiltered).
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, pid.update(10.0f, 1.0f, 0.01f));
}

static void test_derivative_filter_first_order() {
  PidConfig c = plain(0.0f, 0.0f, 0.01f);
  c.d_cutoff_hz = 10.0f;
  VelocityPid pid(c);
  pid.update(0.0f, 0.0f, 0.01f);
  // tau = 1/(2*pi*10) = 0.0159155; alpha = 0.01/(0.01+tau) = 0.385870
  // raw derivative -100 -> filtered -38.587 -> term -0.38587
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.38587f, pid.update(0.0f, 1.0f, 0.01f));
}

static void test_anti_windup_conditional_integration() {
  VelocityPid pid(plain(0.1f, 1.0f));
  for (int i = 0; i < 500; ++i) {
    pid.update(100.0f, 0.0f, 0.01f);  // unreachable: P alone is 10
  }
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.output());
  TEST_ASSERT_TRUE(pid.limited());
  // The integrator did not wind up while clipped...
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral());
  // ...so the output leaves saturation as soon as the target is met.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, pid.update(5.0f, 5.0f, 0.01f));
}

static void test_integrator_may_unwind_while_saturated() {
  VelocityPid pid(plain(0.0f, 10.0f));
  for (int i = 0; i < 20; ++i) {
    pid.update(10.0f, 0.0f, 0.01f);  // integral reaches the clamp at 1.0
  }
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.integral());
  // Negative error while at +limit must still be integrated (pulls it back).
  pid.update(0.0f, 1.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.9f, pid.integral());
}

static void test_slew_limit_and_negative_saturation() {
  PidConfig c = plain(1.0f, 0.0f);
  c.slew_rate = 2.0f;  // 0.02 duty per 10 ms
  VelocityPid pid(c);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, pid.update(1.0f, 0.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.04f, pid.update(1.0f, 0.0f, 0.01f));
  TEST_ASSERT_TRUE(pid.limited());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, pid.update(-5.0f, 0.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, odocore::slew_limit(0.0f, 0.5f, 0.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.1f, odocore::slew_limit(0.0f, -0.5f, 10.0f, 0.01f));

  VelocityPid neg(plain(1.0f, 0.0f));
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, neg.update(-5.0f, 0.0f, 0.01f));
}

static void test_output_limit_below_one() {
  PidConfig c = plain(1.0f, 0.0f);
  c.output_limit = 0.6f;
  VelocityPid pid(c);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, pid.update(5.0f, 0.0f, 0.01f));
}

static void test_bad_inputs_hold_output() {
  VelocityPid pid(plain(0.1f, 0.0f));
  const float out = pid.update(1.0f, 0.0f, 0.01f);
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(1.0f, 0.0f, 0.0f));
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(1.0f, 0.0f, -0.01f));
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(NAN, 0.0f, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(out, pid.update(1.0f, INFINITY, 0.01f));
  pid.reset();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.output());
}

static void test_watchdog_starts_expired_without_trip() {
  CommandWatchdog wd(100000u);
  TEST_ASSERT_FALSE(wd.update(0));
  TEST_ASSERT_TRUE(wd.expired());
  TEST_ASSERT_EQUAL_UINT32(0, wd.trips());
}

static void test_watchdog_trips_once_and_needs_feed() {
  CommandWatchdog wd(100000u);
  wd.feed(1000);
  TEST_ASSERT_TRUE(wd.update(101000));   // exactly the timeout: still fresh
  TEST_ASSERT_FALSE(wd.update(101001));
  TEST_ASSERT_FALSE(wd.update(900000));
  TEST_ASSERT_EQUAL_UINT32(1, wd.trips());
  wd.feed(950000);
  TEST_ASSERT_TRUE(wd.update(950001));
  TEST_ASSERT_FALSE(wd.update(2000000));
  TEST_ASSERT_EQUAL_UINT32(2, wd.trips());
}

static void test_watchdog_across_clock_wrap() {
  CommandWatchdog wd(100000u);
  wd.feed(0xFFFFFF00u);
  TEST_ASSERT_TRUE(wd.update(0x00000100u));  // 512 us later, after the wrap
  TEST_ASSERT_FALSE(wd.update(0x00020000u));
  // Stays expired even when the clock comes round to "near" the old feed.
  TEST_ASSERT_FALSE(wd.update(0xFFFFFF10u));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_feedforward_terms);
  RUN_TEST(test_pi_by_hand);
  RUN_TEST(test_derivative_on_measurement_no_setpoint_kick);
  RUN_TEST(test_derivative_filter_first_order);
  RUN_TEST(test_anti_windup_conditional_integration);
  RUN_TEST(test_integrator_may_unwind_while_saturated);
  RUN_TEST(test_slew_limit_and_negative_saturation);
  RUN_TEST(test_output_limit_below_one);
  RUN_TEST(test_bad_inputs_hold_output);
  RUN_TEST(test_watchdog_starts_expired_without_trip);
  RUN_TEST(test_watchdog_trips_once_and_needs_feed);
  RUN_TEST(test_watchdog_across_clock_wrap);
  return UNITY_END();
}
