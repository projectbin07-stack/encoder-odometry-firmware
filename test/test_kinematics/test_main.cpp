// Kinematics and odometry against hand-computed values.
//
// Geometry: r = 0.05 m, lx = 0.15 m, ly = 0.17 m, 45 degree rollers, so
// tan(alpha) = 1 and k = ly + lx = 0.32 m. host/tests/test_kinematics.py uses
// the same numbers for the Python mirror.
#include <math.h>
#include <unity.h>

#include <odocore/kinematics.h>
#include <odocore/odometry.h>

using namespace odocore;

void setUp() {}
void tearDown() {}

static Geometry mecanum() {
  Geometry g;
  g.drive = DriveType::kMecanum;
  g.wheel_radius = 0.05f;
  g.half_length = 0.15f;
  g.half_width = 0.17f;
  g.roller_angle_deg = 45.0f;
  return g;
}

static Twist twist(float vx, float vy, float wz) {
  Twist t;
  t.vx = vx;
  t.vy = vy;
  t.wz = wz;
  return t;
}

static void assert_wheels(float fl, float fr, float rl, float rr, const WheelSpeeds& w) {
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, fl, w.w[kFL]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, fr, w.w[kFR]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, rl, w.w[kRL]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, rr, w.w[kRR]);
}

static void test_mecanum_inverse_hand_values() {
  const Geometry g = mecanum();
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.32f, g.rotation_arm());
  // vx = 0.5 -> 0.5/0.05 = 10 rad/s on every wheel.
  assert_wheels(10.0f, 10.0f, 10.0f, 10.0f, inverse_kinematics(g, twist(0.5f, 0.0f, 0.0f)));
  // vy = 0.2 -> 4 rad/s with signs (-, +, +, -).
  assert_wheels(-4.0f, 4.0f, 4.0f, -4.0f, inverse_kinematics(g, twist(0.0f, 0.2f, 0.0f)));
  // wz = 1 -> k/r = 6.4 rad/s with signs (-, +, -, +).
  assert_wheels(-6.4f, 6.4f, -6.4f, 6.4f, inverse_kinematics(g, twist(0.0f, 0.0f, 1.0f)));
  // All three: FL 10-4-6.4, FR 10+4+6.4, RL 10+4-6.4, RR 10-4+6.4.
  assert_wheels(-0.4f, 20.4f, 7.6f, 12.4f, inverse_kinematics(g, twist(0.5f, 0.2f, 1.0f)));
}

static void test_mecanum_forward_roundtrip_and_least_squares() {
  const Geometry g = mecanum();
  WheelSpeeds w;
  w.w[kFL] = -0.4f;
  w.w[kFR] = 20.4f;
  w.w[kRL] = 7.6f;
  w.w[kRR] = 12.4f;
  Twist t = forward_kinematics(g, w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, t.vx);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, t.vy);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, t.wz);
  // FL slipping 2 rad/s fast: vx = r/4*42 = 0.525, vy = r/4*(-2) = -0.025,
  // wz = r/(4k)*(-2) = -0.078125.
  w.w[kFL] = 12.0f;
  w.w[kFR] = 10.0f;
  w.w[kRL] = 10.0f;
  w.w[kRR] = 10.0f;
  t = forward_kinematics(g, w);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.525f, t.vx);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.025f, t.vy);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.078125f, t.wz);
}

static void test_mecanum_matches_mecanum_kinematics_jacobian() {
  // mecanum-kinematics builds each row as
  //   r*w_i = vx + s_i*t*vy + (-y_i + s_i*t*x_i)*wz
  // with roller signs (-1, 1, 1, -1) and wheel positions (lx,ly), (lx,-ly),
  // (-lx,ly), (-lx,-ly). Rebuild it for 30 degree rollers and compare.
  Geometry g = mecanum();
  g.wheel_radius = 0.04f;
  g.half_length = 0.12f;
  g.half_width = 0.2f;
  g.roller_angle_deg = 30.0f;
  const double t = tan(30.0 * 3.14159265358979323846 / 180.0);
  const double s[4] = {-1, 1, 1, -1};
  const double x[4] = {0.12, 0.12, -0.12, -0.12};
  const double y[4] = {0.2, -0.2, 0.2, -0.2};
  const double vx = 0.3, vy = -0.1, wz = 0.7;
  const WheelSpeeds w = inverse_kinematics(g, twist(0.3f, -0.1f, 0.7f));
  for (int i = 0; i < 4; ++i) {
    const double expected = (vx + s[i] * t * vy + (-y[i] + s[i] * t * x[i]) * wz) / 0.04;
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, static_cast<float>(expected), w.w[i]);
  }
  const Twist back = forward_kinematics(g, w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.3f, back.vx);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.1f, back.vy);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.7f, back.wz);
}

static void test_differential_drive() {
  Geometry g = mecanum();
  g.drive = DriveType::kDifferential;
  // left = (0.5 - 1*0.17)/0.05 = 6.6, right = (0.5 + 0.17)/0.05 = 13.4; vy ignored.
  const WheelSpeeds w = inverse_kinematics(g, twist(0.5f, 0.3f, 1.0f));
  assert_wheels(6.6f, 13.4f, 6.6f, 13.4f, w);
  const Twist t = forward_kinematics(g, w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, t.vx);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, t.vy);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, t.wz);
}

static void test_saturation_uniform() {
  WheelSpeeds w;
  w.w[0] = 30.0f;
  w.w[1] = -15.0f;
  w.w[2] = 0.0f;
  w.w[3] = 10.0f;
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f / 3.0f, saturate_wheel_speeds(w, 20.0f));
  assert_wheels(20.0f, -10.0f, 0.0f, 20.0f / 3.0f, w);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, saturate_wheel_speeds(w, 100.0f));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, saturate_wheel_speeds(w, 0.0f));  // disabled
}

static void test_geometry_validation() {
  Geometry g = mecanum();
  TEST_ASSERT_TRUE(g.valid());
  g.roller_angle_deg = 90.0f;
  TEST_ASSERT_FALSE(g.valid());
  g.drive = DriveType::kDifferential;  // roller angle irrelevant
  TEST_ASSERT_TRUE(g.valid());
  g.wheel_radius = 0.0f;
  TEST_ASSERT_FALSE(g.valid());
}

static void test_wrap_angle() {
  const double pi = 3.14159265358979323846;
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, pi, wrap_angle(3.0 * pi));
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, pi, wrap_angle(-pi));
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, -0.5, wrap_angle(-0.5 + 4.0 * pi));
}

static void test_se2_quarter_circle_and_full_circle() {
  const double pi = 3.14159265358979323846;
  // Unit radius quarter arc: forward pi/2 while turning pi/2 ends at (1, 1).
  const Pose2D q = se2_exp(pi / 2.0, 0.0, pi / 2.0);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.0, q.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.0, q.y);
  // 100 equal arcs around a full circle return to the start exactly.
  Odometry odo;
  Twist step;
  step.vx = static_cast<float>(2.0 * pi / 100.0);
  step.wz = static_cast<float>(2.0 * pi / 100.0);
  for (int i = 0; i < 100; ++i) {
    odo.integrate(step);
  }
  TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, odo.pose().x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-5, 0.0, odo.pose().y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-5, 2.0 * pi, odo.pose().theta);
}

static void test_small_angle_branch_matches_exact() {
  const Pose2D a = se2_exp(0.1, 0.05, 5e-7);
  // sin(t)/t ~ 1, (1-cos t)/t ~ t/2: x = 0.1 - 0.05*2.5e-7, y = 0.1*2.5e-7 + 0.05.
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.1 - 0.05 * 2.5e-7, a.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.05 + 0.1 * 2.5e-7, a.y);
}

static void test_odometry_from_wheel_increments_and_set_pose() {
  const Geometry g = mecanum();
  Odometry odo;
  // Spin in place: increments (-a, a, -a, a) give dtheta = r*a/k.
  WheelSpeeds inc;
  const float a = 0.64f;
  inc.w[kFL] = -a;
  inc.w[kFR] = a;
  inc.w[kRL] = -a;
  inc.w[kRR] = a;
  odo.integrate_wheels(g, inc);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.05 * 0.64 / 0.32, odo.pose().theta);  // 0.1 rad
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, odo.pose().x);
  // Now drive 1 rad of wheel rotation straight: 0.05 m along the heading.
  for (float& w : inc.w) {
    w = 1.0f;
  }
  odo.integrate_wheels(g, inc);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.05 * cos(0.1), odo.pose().x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.05 * sin(0.1), odo.pose().y);
  Pose2D p;
  p.x = 1.0;
  p.y = -2.0;
  p.theta = 0.5;
  odo.set_pose(p);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, -2.0, odo.pose().y);
  odo.reset();
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, odo.pose().theta);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_mecanum_inverse_hand_values);
  RUN_TEST(test_mecanum_forward_roundtrip_and_least_squares);
  RUN_TEST(test_mecanum_matches_mecanum_kinematics_jacobian);
  RUN_TEST(test_differential_drive);
  RUN_TEST(test_saturation_uniform);
  RUN_TEST(test_geometry_validation);
  RUN_TEST(test_wrap_angle);
  RUN_TEST(test_se2_quarter_circle_and_full_circle);
  RUN_TEST(test_small_angle_branch_matches_exact);
  RUN_TEST(test_odometry_from_wheel_increments_and_set_pose);
  return UNITY_END();
}
