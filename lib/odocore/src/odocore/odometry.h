// Dead-reckoning on SE(2) with exact arc integration.
//
// Each control period's body displacement (dx, dy, dtheta) comes from forward
// kinematics applied to the wheel angle increments. It is composed onto the
// pose with the SE(2) exponential map, which is exact when the body twist is
// constant over the period: a robot driving a perfect circle is integrated
// without the chord error of an Euler step.
//
// The pose is accumulated in double. On the ESP32 that is software-emulated,
// but it runs once per control period and keeps sub-millimetre resolution far
// from the origin; the wheel-level maths stays in float.
#pragma once

#include "odocore/kinematics.h"

namespace odocore {

struct Pose2D {
  double x = 0.0;      ///< m
  double y = 0.0;      ///< m
  double theta = 0.0;  ///< rad, not wrapped
};

/// Wrap an angle to (-pi, pi].
double wrap_angle(double angle);

/// Exponential map: pose reached from the origin by holding the body-frame
/// displacement rate constant for unit time.
Pose2D se2_exp(double dx, double dy, double dtheta);

/// `a * b`: `b` expressed in the frame `a` is expressed in.
Pose2D compose(const Pose2D& a, const Pose2D& b);

class Odometry {
 public:
  void reset() { pose_ = Pose2D(); }
  void set_pose(const Pose2D& pose) { pose_ = pose; }
  const Pose2D& pose() const { return pose_; }

  /// Advance by a body-frame displacement (vx = dx in m, vy = dy in m, wz =
  /// dtheta in rad).
  void integrate(const Twist& displacement);

  /// Advance from wheel angle increments (rad, FL FR RL RR).
  void integrate_wheels(const Geometry& geometry, const WheelSpeeds& angle_increments);

 private:
  Pose2D pose_;
};

}  // namespace odocore
