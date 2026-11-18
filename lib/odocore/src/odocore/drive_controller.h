// The whole per-period control computation, free of hardware:
//
//   encoder snapshots -> wheel speeds (rad/s) -> odometry
//   velocity command -> inverse kinematics -> saturated wheel targets
//   targets + speeds -> per-wheel PI(D) -> duty cycles
//
// The firmware samples the encoders, calls `step`, and writes the duties to
// the motor drivers. Everything else lives here and is unit-tested natively.
#pragma once

#include <stdint.h>

#include "odocore/kinematics.h"
#include "odocore/odometry.h"
#include "odocore/pid.h"
#include "odocore/quadrature.h"
#include "odocore/velocity.h"
#include "odocore/watchdog.h"

namespace odocore {

struct DriveConfig {
  Geometry geometry;
  float ticks_per_wheel_rev = 2000.0f * 19.0f;  ///< encoder counts per wheel revolution
  float max_wheel_speed = 25.0f;                ///< rad/s, uniform saturation limit
  PidConfig pid;
  VelocityConfig velocity;
  uint32_t watchdog_timeout_ms = 500;
  uint32_t control_rate_hz = 100;   ///< fixed by the firmware build, read-only at runtime
  uint32_t telemetry_divider = 5;   ///< stream wheel state + odometry every N periods

  float radians_per_tick() const;
};

struct WheelStatus {
  int32_t count = 0;
  float velocity = 0.0f;  ///< rad/s, measured
  float target = 0.0f;    ///< rad/s
  float duty = 0.0f;      ///< -1..1
  uint32_t illegal_transitions = 0;
};

class DriveController {
 public:
  explicit DriveController(const DriveConfig& config = DriveConfig());

  /// Mutable access for parameter updates; call `apply_config` afterwards.
  DriveConfig& config() { return config_; }
  const DriveConfig& config() const { return config_; }
  /// Push the configuration into the controllers, estimators and watchdog.
  void apply_config();

  /// New velocity command. Returns false (and ignores it, without feeding
  /// the watchdog) if any component is not finite.
  bool command(const Twist& twist, uint32_t now_us);

  /// One control period. `snapshots` and `duty` are in FL FR RL RR order.
  void step(const EncoderSnapshot snapshots[kWheelCount], uint32_t now_us,
            float duty[kWheelCount]);

  const WheelStatus& wheel(int index) const { return wheels_[index]; }
  const Pose2D& pose() const { return odometry_.pose(); }
  void set_pose(const Pose2D& pose) { odometry_.set_pose(pose); }
  /// Body twist from the measured wheel speeds.
  const Twist& measured_twist() const { return measured_; }
  /// The command actually being tracked (after saturation scaling).
  const Twist& active_command() const { return active_; }
  float saturation_scale() const { return scale_; }
  bool watchdog_expired() const { return watchdog_.expired(); }
  uint32_t watchdog_trips() const { return watchdog_.trips(); }
  /// True if the last command asked for vy on a differential drive.
  bool lateral_ignored() const { return lateral_ignored_; }

 private:
  void stop_all(float duty[kWheelCount]);

  DriveConfig config_;
  VelocityEstimator estimators_[kWheelCount];
  VelocityPid pids_[kWheelCount];
  WheelStatus wheels_[kWheelCount];
  CommandWatchdog watchdog_;
  Odometry odometry_;
  WheelSpeeds targets_;
  Twist measured_;
  Twist active_;
  float scale_ = 1.0f;
  bool lateral_ignored_ = false;
  bool primed_ = false;
  uint32_t prev_us_ = 0;
  int32_t prev_count_[kWheelCount] = {0, 0, 0, 0};
};

}  // namespace odocore
