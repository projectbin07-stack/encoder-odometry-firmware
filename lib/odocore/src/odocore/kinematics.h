// Mecanum and differential-drive kinematics.
//
// Conventions match the `mecanum-kinematics` Python library so that the two
// projects interoperate:
//
//  * Body frame per ROS REP-103: x forward, y left, yaw counter-clockwise.
//  * Wheel arrays are always in canonical order FL, FR, RL, RR.
//  * A positive wheel angular velocity drives that wheel's side forward.
//  * Mecanum rollers form an "X" seen from above. With t = tan(roller angle)
//    and k = half_width + half_length * t (= lx + ly for 45 degree rollers):
//
//      r * w_FL = vx - t*vy - k*wz        r * w_FR = vx + t*vy + k*wz
//      r * w_RL = vx + t*vy - k*wz        r * w_RR = vx - t*vy + k*wz
//
//    The four columns of that 4x3 Jacobian are mutually orthogonal, so the
//    least-squares forward solution has the closed form
//
//      vx = r/4      * ( w_FL + w_FR + w_RL + w_RR)
//      vy = r/(4t)   * (-w_FL + w_FR + w_RL - w_RR)
//      wz = r/(4k)   * (-w_FL + w_FR - w_RL + w_RR)
//
//  * Differential drive uses the same four-slot layout (a 4WD skid-steer or a
//    2WD robot wired to the FL/FR slots with RL/RR left unused):
//    left = (vx - wz*half_width)/r, right = (vx + wz*half_width)/r; vy is
//    not achievable and is ignored. Forward kinematics averages each side.
//
// The same forward functions turn wheel angle increments (rad) into body
// displacements (m, m, rad), which is what odometry integrates.
#pragma once

#include <stdint.h>

namespace odocore {

enum WheelIndex : uint8_t { kFL = 0, kFR = 1, kRL = 2, kRR = 3, kWheelCount = 4 };

enum class DriveType : uint8_t { kMecanum = 0, kDifferential = 1 };

struct Twist {
  float vx = 0.0f;  ///< m/s (or m for a displacement)
  float vy = 0.0f;  ///< m/s
  float wz = 0.0f;  ///< rad/s
};

struct WheelSpeeds {
  float w[kWheelCount] = {0.0f, 0.0f, 0.0f, 0.0f};  ///< rad/s (or rad), FL FR RL RR
};

struct Geometry {
  DriveType drive = DriveType::kMecanum;
  float wheel_radius = 0.05f;      ///< m
  float half_length = 0.15f;       ///< m, lx: centre to wheel axle along x
  float half_width = 0.17f;        ///< m, ly: centre to wheel centre along y
  float roller_angle_deg = 45.0f;  ///< mecanum only, strictly between 0 and 90

  /// True if all lengths are positive and the roller angle is in (0, 90).
  bool valid() const;
  /// k = half_width + half_length * tan(roller angle), the yaw lever arm.
  float rotation_arm() const;
};

WheelSpeeds inverse_kinematics(const Geometry& geometry, const Twist& twist);
Twist forward_kinematics(const Geometry& geometry, const WheelSpeeds& wheels);

/// Scale all wheels uniformly so none exceeds `max_speed` (direction-preserving,
/// as in mecanum-kinematics). Returns the scale in (0, 1]; a non-positive
/// limit disables saturation.
float saturate_wheel_speeds(WheelSpeeds& wheels, float max_speed);

}  // namespace odocore
