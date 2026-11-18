#include "odocore/odometry.h"

#include <cmath>

namespace odocore {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kSmallAngle = 1e-6;
}  // namespace

double wrap_angle(double angle) {
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped <= 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

Pose2D se2_exp(double dx, double dy, double dtheta) {
  double a = 0.0;
  double b = 0.0;
  if (std::fabs(dtheta) < kSmallAngle) {
    // Taylor series of sin(t)/t and (1 - cos(t))/t.
    a = 1.0 - dtheta * dtheta / 6.0;
    b = dtheta / 2.0 - dtheta * dtheta * dtheta / 24.0;
  } else {
    a = std::sin(dtheta) / dtheta;
    b = (1.0 - std::cos(dtheta)) / dtheta;
  }
  Pose2D out;
  out.x = a * dx - b * dy;
  out.y = b * dx + a * dy;
  out.theta = dtheta;
  return out;
}

Pose2D compose(const Pose2D& p, const Pose2D& q) {
  const double c = std::cos(p.theta);
  const double s = std::sin(p.theta);
  Pose2D out;
  out.x = p.x + c * q.x - s * q.y;
  out.y = p.y + s * q.x + c * q.y;
  out.theta = p.theta + q.theta;
  return out;
}

void Odometry::integrate(const Twist& d) {
  pose_ = compose(pose_, se2_exp(d.vx, d.vy, d.wz));
}

void Odometry::integrate_wheels(const Geometry& geometry, const WheelSpeeds& increments) {
  integrate(forward_kinematics(geometry, increments));
}

}  // namespace odocore
