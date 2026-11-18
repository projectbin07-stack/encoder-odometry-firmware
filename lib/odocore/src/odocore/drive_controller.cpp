#include "odocore/drive_controller.h"

#include <cmath>

namespace odocore {

namespace {
constexpr float kTwoPi = 6.28318530717958647692f;
}  // namespace

float DriveConfig::radians_per_tick() const { return kTwoPi / ticks_per_wheel_rev; }

DriveController::DriveController(const DriveConfig& config) : config_(config) { apply_config(); }

void DriveController::apply_config() {
  for (int i = 0; i < kWheelCount; ++i) {
    estimators_[i].configure(config_.velocity);
    pids_[i].configure(config_.pid);
  }
  watchdog_.set_timeout_us(config_.watchdog_timeout_ms * 1000u);
}

bool DriveController::command(const Twist& twist, uint32_t now_us) {
  if (!std::isfinite(twist.vx) || !std::isfinite(twist.vy) || !std::isfinite(twist.wz)) {
    return false;
  }
  lateral_ignored_ = config_.geometry.drive == DriveType::kDifferential && twist.vy != 0.0f;
  WheelSpeeds wheels = inverse_kinematics(config_.geometry, twist);
  scale_ = saturate_wheel_speeds(wheels, config_.max_wheel_speed);
  targets_ = wheels;
  active_.vx = twist.vx * scale_;
  active_.vy = config_.geometry.drive == DriveType::kDifferential ? 0.0f : twist.vy * scale_;
  active_.wz = twist.wz * scale_;
  watchdog_.feed(now_us);
  return true;
}

void DriveController::stop_all(float duty[kWheelCount]) {
  for (int i = 0; i < kWheelCount; ++i) {
    pids_[i].reset();
    targets_.w[i] = 0.0f;
    wheels_[i].target = 0.0f;
    wheels_[i].duty = 0.0f;
    duty[i] = 0.0f;
  }
  active_ = Twist();
  scale_ = 1.0f;
}

void DriveController::step(const EncoderSnapshot snapshots[kWheelCount], uint32_t now_us,
                           float duty[kWheelCount]) {
  const float rad_per_tick = config_.radians_per_tick();
  if (!primed_) {
    for (int i = 0; i < kWheelCount; ++i) {
      estimators_[i].update(snapshots[i], now_us);
      prev_count_[i] = snapshots[i].count;
      wheels_[i].count = snapshots[i].count;
      wheels_[i].illegal_transitions = snapshots[i].illegal_transitions;
    }
    prev_us_ = now_us;
    primed_ = true;
    // No speed estimate yet: hold the motors off for this one period.
    for (int i = 0; i < kWheelCount; ++i) {
      duty[i] = 0.0f;
      wheels_[i].duty = 0.0f;
    }
    return;
  }

  const float dt = static_cast<float>(elapsed_us(now_us, prev_us_)) * 1.0e-6f;
  prev_us_ = now_us;

  WheelSpeeds speeds;
  WheelSpeeds increments;
  for (int i = 0; i < kWheelCount; ++i) {
    const EncoderSnapshot& s = snapshots[i];
    speeds.w[i] = estimators_[i].update(s, now_us) * rad_per_tick;
    increments.w[i] = static_cast<float>(count_delta(s.count, prev_count_[i])) * rad_per_tick;
    prev_count_[i] = s.count;
    wheels_[i].count = s.count;
    wheels_[i].velocity = speeds.w[i];
    wheels_[i].illegal_transitions = s.illegal_transitions;
  }
  odometry_.integrate_wheels(config_.geometry, increments);
  measured_ = forward_kinematics(config_.geometry, speeds);

  if (!watchdog_.update(now_us)) {
    stop_all(duty);
    return;
  }
  for (int i = 0; i < kWheelCount; ++i) {
    duty[i] = pids_[i].update(targets_.w[i], speeds.w[i], dt);
    wheels_[i].target = targets_.w[i];
    wheels_[i].duty = duty[i];
  }
}

}  // namespace odocore
