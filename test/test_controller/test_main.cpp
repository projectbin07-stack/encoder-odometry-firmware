// DriveController closed loop against a simulated plant, parameters, the
// protocol node, and the H-bridge output tables.
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#include <odocore/drive_controller.h>
#include <odocore/motor_output.h>
#include <odocore/node.h>
#include <odocore/params.h>

using namespace odocore;

void setUp() {}
void tearDown() {}

// First-order motor with a dead band, driving ideal quadrature encoders with
// exact edge timestamps. Same model as host/src/odobridge/firmware_model.py.
class Plant {
 public:
  explicit Plant(const DriveConfig& cfg) : rad_per_tick_(cfg.radians_per_tick()) {}

  void advance(const float duty[4], uint32_t dt_us) {
    const int sub = 10;
    const double h = dt_us * 1e-6 / sub;
    for (int k = 0; k < sub; ++k) {
      for (int i = 0; i < 4; ++i) {
        double target = 0.0;
        const double mag = fabs(duty[i]) - 0.06;
        if (mag > 0.0) {
          target = (duty[i] > 0 ? 1.0 : -1.0) * mag / 0.034;
        }
        const double before = speed_[i];
        speed_[i] += (1.0 - exp(-h / 0.08)) * (target - speed_[i]);
        angle_[i] += 0.5 * (before + speed_[i]) * h;
        const int64_t c = static_cast<int64_t>(floor(angle_[i] / rad_per_tick_));
        if (c != count_[i]) {
          count_[i] = c;
          edge_us_[i] = now_us_ + static_cast<uint32_t>((k + 1) * h * 1e6);
        }
      }
    }
    now_us_ += dt_us;
  }

  void snapshots(EncoderSnapshot out[4]) const {
    for (int i = 0; i < 4; ++i) {
      out[i].count = static_cast<int32_t>(count_[i]);
      out[i].last_edge_us = edge_us_[i];
      out[i].illegal_transitions = 0;
      out[i].edge_time_exact = true;
    }
  }

  uint32_t now() const { return now_us_; }
  double angle(int i) const { return angle_[i]; }
  double speed(int i) const { return speed_[i]; }

 private:
  double rad_per_tick_;
  double speed_[4] = {0, 0, 0, 0};
  double angle_[4] = {0, 0, 0, 0};
  int64_t count_[4] = {0, 0, 0, 0};
  uint32_t edge_us_[4] = {0, 0, 0, 0};
  uint32_t now_us_ = 0;
};

static Twist twist(float vx, float vy, float wz) {
  Twist t;
  t.vx = vx;
  t.vy = vy;
  t.wz = wz;
  return t;
}

static void run(DriveController& c, Plant& p, int periods, float duty[4]) {
  for (int k = 0; k < periods; ++k) {
    EncoderSnapshot s[4];
    p.snapshots(s);
    c.step(s, p.now(), duty);
    p.advance(duty, 10000u);
  }
}

static void test_closed_loop_reaches_wheel_targets() {
  DriveController c;
  Plant p(c.config());
  float duty[4] = {0, 0, 0, 0};
  // The default feed-forward (0.03) is deliberately not matched to the plant
  // (0.034 duty per rad/s plus a 0.06 dead band); the integrator closes the gap.
  for (int k = 0; k < 15; ++k) {  // 1.5 s, command refreshed every 100 ms
    TEST_ASSERT_TRUE(c.command(twist(0.3f, 0.1f, 0.5f), p.now()));
    run(c, p, 10, duty);
  }
  // Targets from inverse kinematics: (0.3 -/+ 0.1 -/+ 0.32*0.5)/0.05.
  const float expected[4] = {0.8f, 11.2f, 4.8f, 7.2f};
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected[i], c.wheel(i).target);
    TEST_ASSERT_FLOAT_WITHIN(0.03f * fabsf(expected[i]) + 0.05f, expected[i], c.wheel(i).velocity);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.3f, c.measured_twist().vx);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f, c.measured_twist().vy);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, c.measured_twist().wz);
  TEST_ASSERT_FALSE(c.watchdog_expired());
}

static void test_watchdog_stops_motors_and_counts_trip() {
  DriveController c;
  Plant p(c.config());
  float duty[4];
  c.command(twist(0.3f, 0.0f, 0.0f), p.now());
  run(c, p, 30, duty);
  TEST_ASSERT_TRUE(duty[0] > 0.1f);
  run(c, p, 25, duty);  // no new command for > 500 ms
  TEST_ASSERT_TRUE(c.watchdog_expired());
  TEST_ASSERT_EQUAL_UINT32(1, c.watchdog_trips());
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty[i]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.wheel(i).target);
  }
  // A fresh command resumes control.
  c.command(twist(0.1f, 0.0f, 0.0f), p.now());
  run(c, p, 5, duty);
  TEST_ASSERT_FALSE(c.watchdog_expired());
  TEST_ASSERT_TRUE(duty[0] > 0.0f);
}

static void test_saturation_scales_command() {
  DriveConfig cfg;
  cfg.max_wheel_speed = 5.0f;
  DriveController c(cfg);
  c.command(twist(0.5f, 0.0f, 0.0f), 0);  // needs 10 rad/s per wheel
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, c.saturation_scale());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, c.active_command().vx);
}

static void test_non_finite_command_rejected() {
  DriveController c;
  TEST_ASSERT_FALSE(c.command(twist(NAN, 0.0f, 0.0f), 0));
  TEST_ASSERT_FALSE(c.command(twist(0.0f, INFINITY, 0.0f), 0));
  TEST_ASSERT_TRUE(c.watchdog_expired());
}

static void test_differential_ignores_vy() {
  DriveConfig cfg;
  cfg.geometry.drive = DriveType::kDifferential;
  DriveController c(cfg);
  c.command(twist(0.2f, 0.5f, 0.0f), 0);
  TEST_ASSERT_TRUE(c.lateral_ignored());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.active_command().vy);
}

static void test_odometry_follows_encoder_counts() {
  DriveController c;
  Plant p(c.config());
  float duty[4];
  for (int k = 0; k < 15; ++k) {
    c.command(twist(0.25f, 0.0f, 0.0f), p.now());
    run(c, p, 10, duty);
  }
  // Compare at the same instant: one more sample after the last advance.
  EncoderSnapshot s[4];
  p.snapshots(s);
  c.step(s, p.now(), duty);
  // Straight drive: x = r * mean wheel angle (to one count), y = theta = 0.
  double mean = 0.0;
  for (int i = 0; i < 4; ++i) {
    mean += p.angle(i) / 4.0;
  }
  TEST_ASSERT_DOUBLE_WITHIN(1e-4, 0.05 * mean, c.pose().x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, c.pose().y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, c.pose().theta);
  TEST_ASSERT_TRUE(c.pose().x > 0.3);
}

static void test_params_get_set() {
  DriveConfig cfg;
  float v = 0.0f;
  TEST_ASSERT_EQUAL(ParamStatus::kOk, get_param(cfg, 1, &v));
  TEST_ASSERT_EQUAL_FLOAT(cfg.pid.kp, v);
  TEST_ASSERT_EQUAL(ParamStatus::kOk, set_param(cfg, 9, 12.5f));
  TEST_ASSERT_EQUAL_FLOAT(12.5f, cfg.max_wheel_speed);
  TEST_ASSERT_EQUAL(ParamStatus::kOutOfRange, set_param(cfg, 14, 95.0f));
  TEST_ASSERT_EQUAL(ParamStatus::kOutOfRange, set_param(cfg, 1, NAN));
  TEST_ASSERT_EQUAL(ParamStatus::kReadOnly, set_param(cfg, 21, 50.0f));
  TEST_ASSERT_EQUAL(ParamStatus::kUnknownId, set_param(cfg, 99, 1.0f));
  TEST_ASSERT_EQUAL(ParamStatus::kUnknownId, get_param(cfg, 0, &v));
  // Integer parameters are rounded.
  TEST_ASSERT_EQUAL(ParamStatus::kOk, set_param(cfg, 10, 249.6f));
  TEST_ASSERT_EQUAL_UINT32(250, cfg.watchdog_timeout_ms);
  TEST_ASSERT_EQUAL(ParamStatus::kOk, set_param(cfg, 16, 1.0f));
  TEST_ASSERT_TRUE(cfg.geometry.drive == DriveType::kDifferential);
  TEST_ASSERT_EQUAL(ParamStatus::kOk, set_param(cfg, 20, 100.0f));
  TEST_ASSERT_EQUAL_UINT32(100000, cfg.velocity.zero_timeout_us);
  TEST_ASSERT_EQUAL(ParamStatus::kOk, get_param(cfg, 22, &v));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, v);
  // Every table entry can be read, and ids are unique.
  for (int i = 0; i < param_count(); ++i) {
    TEST_ASSERT_EQUAL(ParamStatus::kOk, get_param(cfg, param_at(i).id, &v));
    for (int j = i + 1; j < param_count(); ++j) {
      TEST_ASSERT_NOT_EQUAL(param_at(i).id, param_at(j).id);
    }
  }
  TEST_ASSERT_EQUAL_INT(22, param_count());
}

class BufferSink : public FrameSink {
 public:
  void write_frame(const uint8_t* data, size_t length) override {
    for (size_t i = 0; i < length && size < sizeof(buffer); ++i) {
      buffer[size++] = data[i];
    }
  }
  // Decode everything written so far.
  int decode(Message* out, int max) {
    FrameReceiver rx;
    int n = 0;
    for (size_t i = 0; i < size && n < max; ++i) {
      if (rx.push(buffer[i], &out[n])) {
        ++n;
      }
    }
    return n;
  }
  uint8_t buffer[4096];
  size_t size = 0;
};

static void feed(FirmwareNode& node, const Message& m) {
  uint8_t frame[kMaxFrameSize];
  const size_t n = encode_frame(m, frame, sizeof(frame));
  for (size_t i = 0; i < n; ++i) {
    node.receive_byte(frame[i], 0);
  }
}

static void test_node_param_and_command_handling() {
  DriveController c;
  BufferSink sink;
  FirmwareNode node(c, sink);
  Message set;
  set.type = MsgType::kParamSet;
  set.param_set.id = 10;  // watchdog_ms
  set.param_set.value = 100.0f;
  feed(node, set);
  Message cmd;
  cmd.type = MsgType::kVelocityCommand;
  cmd.velocity_command.vx = 0.1f;
  feed(node, cmd);
  TEST_ASSERT_FALSE(c.watchdog_expired());
  Message out[8];
  TEST_ASSERT_EQUAL_INT(1, sink.decode(out, 8));
  TEST_ASSERT_TRUE(out[0].type == MsgType::kParamValue);
  TEST_ASSERT_EQUAL_UINT8(10, out[0].param_value.id);
  TEST_ASSERT_EQUAL_UINT8(0, out[0].param_value.status);
  TEST_ASSERT_EQUAL_FLOAT(100.0f, out[0].param_value.value);
  TEST_ASSERT_EQUAL_UINT32(100, c.config().watchdog_timeout_ms);

  Message pose;
  pose.type = MsgType::kSetPose;
  pose.set_pose.x = 1.0f;
  pose.set_pose.theta = 4.0f;  // reported wrapped: 4 - 2*pi
  feed(node, pose);
  TEST_ASSERT_EQUAL_INT(2, sink.decode(out, 8));
  TEST_ASSERT_TRUE(out[1].type == MsgType::kOdometry);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, out[1].odometry.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f - 6.2831853f, out[1].odometry.theta);
  TEST_ASSERT_EQUAL_UINT8(1, out[1].seq);  // firmware's own sequence counter

  // Unknown id: reply carries the status.
  Message get;
  get.type = MsgType::kParamGet;
  get.param_get.id = 200;
  feed(node, get);
  TEST_ASSERT_EQUAL_INT(3, sink.decode(out, 8));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(ParamStatus::kUnknownId), out[2].param_value.status);
}

static void test_node_telemetry_schedule_and_diagnostics() {
  DriveController c;  // telemetry divider 5
  BufferSink sink;
  FirmwareNode node(c, sink);
  node.set_hardware_counters(true);
  for (uint32_t k = 0; k < 10; ++k) {
    node.after_step(k * 10000u);
  }
  // Periods 0 and 5 send WHEEL_STATE + ODOMETRY; period 0 also DIAGNOSTICS.
  Message out[16];
  TEST_ASSERT_EQUAL_INT(5, sink.decode(out, 16));
  TEST_ASSERT_TRUE(out[0].type == MsgType::kWheelState);
  TEST_ASSERT_TRUE(out[1].type == MsgType::kOdometry);
  TEST_ASSERT_TRUE(out[2].type == MsgType::kDiagnostics);
  TEST_ASSERT_TRUE(out[3].type == MsgType::kWheelState);
  TEST_ASSERT_EQUAL_UINT32(50000, out[3].wheel_state.t_us);
  const uint8_t flags = out[2].diagnostics.flags;
  TEST_ASSERT_TRUE(flags & kDiagWatchdogExpired);  // no command yet
  TEST_ASSERT_TRUE(flags & kDiagHardwareCounters);
  TEST_ASSERT_EQUAL_UINT32(5, node.frames_sent());

  // Garbage on the input is counted and reported.
  node.receive_byte(0x05, 0);
  node.receive_byte(0x00, 0);
  TEST_ASSERT_EQUAL_UINT32(1, node.make_diagnostics(0).diagnostics.rx_frame_errors);
}

static void test_loop_monitor() {
  LoopMonitor mon(10000u);
  mon.record(0, 4000, 0);
  mon.record(10000, 22000, 0);   // ran longer than the period
  mon.record(30000, 31000, 2);   // two timer ticks were missed
  TEST_ASSERT_EQUAL_UINT32(3, mon.overruns());
  TEST_ASSERT_EQUAL_UINT32(12000, mon.max_loop_us());
  TEST_ASSERT_EQUAL_UINT32(1000, mon.last_loop_us());
  mon.record(0xFFFFFF00u, 0x00000100u, 0);  // micros() wrapped mid-loop
  TEST_ASSERT_EQUAL_UINT32(512, mon.last_loop_us());
}

static void test_l298n_truth_table() {
  L298nOutput o = l298n_output(0.5f, 1023, DecayMode::kCoast);
  TEST_ASSERT_TRUE(o.in1);
  TEST_ASSERT_FALSE(o.in2);
  TEST_ASSERT_EQUAL_UINT32(512, o.enable_pwm);  // round(511.5)
  o = l298n_output(-1.5f, 1023, DecayMode::kCoast);
  TEST_ASSERT_FALSE(o.in1);
  TEST_ASSERT_TRUE(o.in2);
  TEST_ASSERT_EQUAL_UINT32(1023, o.enable_pwm);
  o = l298n_output(0.0f, 1023, DecayMode::kCoast);
  TEST_ASSERT_EQUAL_UINT32(0, o.enable_pwm);
  o = l298n_output(0.0f, 1023, DecayMode::kBrake);
  TEST_ASSERT_FALSE(o.in1);
  TEST_ASSERT_FALSE(o.in2);
  TEST_ASSERT_EQUAL_UINT32(1023, o.enable_pwm);
  o = l298n_output(NAN, 1023, DecayMode::kCoast);
  TEST_ASSERT_EQUAL_UINT32(0, o.enable_pwm);
}

static void test_drv_truth_table() {
  DrvOutput o = drv_output(0.25f, 1000, DecayMode::kCoast);
  TEST_ASSERT_EQUAL_UINT32(250, o.in1_pwm);
  TEST_ASSERT_EQUAL_UINT32(0, o.in2_pwm);
  o = drv_output(-0.25f, 1000, DecayMode::kCoast);
  TEST_ASSERT_EQUAL_UINT32(0, o.in1_pwm);
  TEST_ASSERT_EQUAL_UINT32(250, o.in2_pwm);
  o = drv_output(0.25f, 1000, DecayMode::kBrake);
  TEST_ASSERT_EQUAL_UINT32(1000, o.in1_pwm);
  TEST_ASSERT_EQUAL_UINT32(750, o.in2_pwm);
  o = drv_output(-0.25f, 1000, DecayMode::kBrake);
  TEST_ASSERT_EQUAL_UINT32(750, o.in1_pwm);
  TEST_ASSERT_EQUAL_UINT32(1000, o.in2_pwm);
  o = drv_output(0.0f, 1000, DecayMode::kBrake);
  TEST_ASSERT_EQUAL_UINT32(1000, o.in1_pwm);
  TEST_ASSERT_EQUAL_UINT32(1000, o.in2_pwm);
  o = drv_output(0.0f, 1000, DecayMode::kCoast);
  TEST_ASSERT_EQUAL_UINT32(0, o.in1_pwm);
  TEST_ASSERT_EQUAL_UINT32(0, o.in2_pwm);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_closed_loop_reaches_wheel_targets);
  RUN_TEST(test_watchdog_stops_motors_and_counts_trip);
  RUN_TEST(test_saturation_scales_command);
  RUN_TEST(test_non_finite_command_rejected);
  RUN_TEST(test_differential_ignores_vy);
  RUN_TEST(test_odometry_follows_encoder_counts);
  RUN_TEST(test_params_get_set);
  RUN_TEST(test_node_param_and_command_handling);
  RUN_TEST(test_node_telemetry_schedule_and_diagnostics);
  RUN_TEST(test_loop_monitor);
  RUN_TEST(test_l298n_truth_table);
  RUN_TEST(test_drv_truth_table);
  return UNITY_END();
}
