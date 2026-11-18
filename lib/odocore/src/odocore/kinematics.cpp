#include "odocore/kinematics.h"

#include <cmath>

namespace odocore {

namespace {
constexpr float kDegToRad = 0.017453292519943295f;
}  // namespace

bool Geometry::valid() const {
  if (!(wheel_radius > 0.0f) || !(half_width > 0.0f)) {
    return false;
  }
  if (drive == DriveType::kDifferential) {
    return true;
  }
  return half_length > 0.0f && roller_angle_deg > 0.0f && roller_angle_deg < 90.0f;
}

float Geometry::rotation_arm() const {
  return half_width + half_length * std::tan(roller_angle_deg * kDegToRad);
}

WheelSpeeds inverse_kinematics(const Geometry& g, const Twist& v) {
  WheelSpeeds out;
  const float inv_r = 1.0f / g.wheel_radius;
  if (g.drive == DriveType::kDifferential) {
    const float left = (v.vx - v.wz * g.half_width) * inv_r;
    const float right = (v.vx + v.wz * g.half_width) * inv_r;
    out.w[kFL] = left;
    out.w[kRL] = left;
    out.w[kFR] = right;
    out.w[kRR] = right;
    return out;
  }
  const float t = std::tan(g.roller_angle_deg * kDegToRad);
  const float k = g.rotation_arm();
  out.w[kFL] = (v.vx - t * v.vy - k * v.wz) * inv_r;
  out.w[kFR] = (v.vx + t * v.vy + k * v.wz) * inv_r;
  out.w[kRL] = (v.vx + t * v.vy - k * v.wz) * inv_r;
  out.w[kRR] = (v.vx - t * v.vy + k * v.wz) * inv_r;
  return out;
}

Twist forward_kinematics(const Geometry& g, const WheelSpeeds& s) {
  Twist out;
  const float r = g.wheel_radius;
  const float* w = s.w;
  if (g.drive == DriveType::kDifferential) {
    const float left = 0.5f * (w[kFL] + w[kRL]);
    const float right = 0.5f * (w[kFR] + w[kRR]);
    out.vx = 0.5f * r * (left + right);
    out.vy = 0.0f;
    out.wz = r * (right - left) / (2.0f * g.half_width);
    return out;
  }
  const float t = std::tan(g.roller_angle_deg * kDegToRad);
  const float k = g.rotation_arm();
  out.vx = 0.25f * r * (w[kFL] + w[kFR] + w[kRL] + w[kRR]);
  out.vy = 0.25f * r / t * (-w[kFL] + w[kFR] + w[kRL] - w[kRR]);
  out.wz = 0.25f * r / k * (-w[kFL] + w[kFR] - w[kRL] + w[kRR]);
  return out;
}

float saturate_wheel_speeds(WheelSpeeds& wheels, float max_speed) {
  if (!(max_speed > 0.0f) || std::isinf(max_speed)) {
    return 1.0f;
  }
  float peak = 0.0f;
  for (float w : wheels.w) {
    const float a = std::fabs(w);
    if (a > peak) {
      peak = a;
    }
  }
  if (peak <= max_speed) {
    return 1.0f;
  }
  const float scale = max_speed / peak;
  for (float& w : wheels.w) {
    w *= scale;
  }
  return scale;
}

}  // namespace odocore
