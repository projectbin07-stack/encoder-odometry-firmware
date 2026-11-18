#include "odocore/params.h"

#include <cmath>

#include "odocore/protocol.h"

namespace odocore {

namespace {

// Keep in sync with host/src/odobridge/params.py and docs/protocol.md.
constexpr ParamInfo kParams[] = {
    {1, "kp", 0.0f, 10.0f, false, false},
    {2, "ki", 0.0f, 100.0f, false, false},
    {3, "kd", 0.0f, 1.0f, false, false},
    {4, "kff", 0.0f, 1.0f, false, false},
    {5, "ks", 0.0f, 1.0f, false, false},
    {6, "output_limit", 0.0f, 1.0f, false, false},
    {7, "slew_rate", 0.0f, 1000.0f, false, false},
    {8, "d_cutoff_hz", 0.0f, 1000.0f, false, false},
    {9, "max_wheel_speed", 0.1f, 1000.0f, false, false},
    {10, "watchdog_ms", 20.0f, 10000.0f, false, true},
    {11, "wheel_radius", 0.001f, 1.0f, false, false},
    {12, "half_length", 0.001f, 5.0f, false, false},
    {13, "half_width", 0.001f, 5.0f, false, false},
    {14, "roller_angle_deg", 1.0f, 89.0f, false, false},
    {15, "ticks_per_wheel_rev", 1.0f, 10000000.0f, false, false},
    {16, "drive_type", 0.0f, 1.0f, false, true},
    {17, "telemetry_divider", 1.0f, 1000.0f, false, true},
    {18, "estimator_low_cps", 0.0f, 1000000.0f, false, false},
    {19, "estimator_high_cps", 0.0f, 1000000.0f, false, false},
    {20, "zero_timeout_ms", 1.0f, 10000.0f, false, true},
    {21, "control_rate_hz", 1.0f, 10000.0f, true, true},
    {22, "protocol_version", 0.0f, 255.0f, true, true},
};

constexpr int kParamCount = static_cast<int>(sizeof(kParams) / sizeof(kParams[0]));

}  // namespace

const ParamInfo* find_param(uint8_t id) {
  for (const ParamInfo& p : kParams) {
    if (p.id == id) {
      return &p;
    }
  }
  return nullptr;
}

int param_count() { return kParamCount; }

const ParamInfo& param_at(int index) { return kParams[index]; }

ParamStatus get_param(const DriveConfig& c, uint8_t id, float* value) {
  if (find_param(id) == nullptr) {
    return ParamStatus::kUnknownId;
  }
  float v = 0.0f;
  switch (static_cast<ParamId>(id)) {
    case ParamId::kKp: v = c.pid.kp; break;
    case ParamId::kKi: v = c.pid.ki; break;
    case ParamId::kKd: v = c.pid.kd; break;
    case ParamId::kKff: v = c.pid.kff; break;
    case ParamId::kKs: v = c.pid.ks; break;
    case ParamId::kOutputLimit: v = c.pid.output_limit; break;
    case ParamId::kSlewRate: v = c.pid.slew_rate; break;
    case ParamId::kDCutoffHz: v = c.pid.d_cutoff_hz; break;
    case ParamId::kMaxWheelSpeed: v = c.max_wheel_speed; break;
    case ParamId::kWatchdogMs: v = static_cast<float>(c.watchdog_timeout_ms); break;
    case ParamId::kWheelRadius: v = c.geometry.wheel_radius; break;
    case ParamId::kHalfLength: v = c.geometry.half_length; break;
    case ParamId::kHalfWidth: v = c.geometry.half_width; break;
    case ParamId::kRollerAngleDeg: v = c.geometry.roller_angle_deg; break;
    case ParamId::kTicksPerWheelRev: v = c.ticks_per_wheel_rev; break;
    case ParamId::kDriveType: v = static_cast<float>(static_cast<uint8_t>(c.geometry.drive)); break;
    case ParamId::kTelemetryDivider: v = static_cast<float>(c.telemetry_divider); break;
    case ParamId::kEstimatorLowCps: v = c.velocity.low_cps; break;
    case ParamId::kEstimatorHighCps: v = c.velocity.high_cps; break;
    case ParamId::kZeroTimeoutMs: v = static_cast<float>(c.velocity.zero_timeout_us / 1000u); break;
    case ParamId::kControlRateHz: v = static_cast<float>(c.control_rate_hz); break;
    case ParamId::kProtocolVersion: v = static_cast<float>(kProtocolVersion); break;
  }
  *value = v;
  return ParamStatus::kOk;
}

ParamStatus set_param(DriveConfig& c, uint8_t id, float value) {
  const ParamInfo* info = find_param(id);
  if (info == nullptr) {
    return ParamStatus::kUnknownId;
  }
  if (info->read_only) {
    return ParamStatus::kReadOnly;
  }
  if (!std::isfinite(value) || value < info->min || value > info->max) {
    return ParamStatus::kOutOfRange;
  }
  const float v = info->integer ? std::round(value) : value;
  const uint32_t u = static_cast<uint32_t>(v < 0.0f ? 0.0f : v);
  switch (static_cast<ParamId>(id)) {
    case ParamId::kKp: c.pid.kp = v; break;
    case ParamId::kKi: c.pid.ki = v; break;
    case ParamId::kKd: c.pid.kd = v; break;
    case ParamId::kKff: c.pid.kff = v; break;
    case ParamId::kKs: c.pid.ks = v; break;
    case ParamId::kOutputLimit: c.pid.output_limit = v; break;
    case ParamId::kSlewRate: c.pid.slew_rate = v; break;
    case ParamId::kDCutoffHz: c.pid.d_cutoff_hz = v; break;
    case ParamId::kMaxWheelSpeed: c.max_wheel_speed = v; break;
    case ParamId::kWatchdogMs: c.watchdog_timeout_ms = u; break;
    case ParamId::kWheelRadius: c.geometry.wheel_radius = v; break;
    case ParamId::kHalfLength: c.geometry.half_length = v; break;
    case ParamId::kHalfWidth: c.geometry.half_width = v; break;
    case ParamId::kRollerAngleDeg: c.geometry.roller_angle_deg = v; break;
    case ParamId::kTicksPerWheelRev: c.ticks_per_wheel_rev = v; break;
    case ParamId::kDriveType:
      c.geometry.drive = u == 0 ? DriveType::kMecanum : DriveType::kDifferential;
      break;
    case ParamId::kTelemetryDivider: c.telemetry_divider = u; break;
    case ParamId::kEstimatorLowCps: c.velocity.low_cps = v; break;
    case ParamId::kEstimatorHighCps: c.velocity.high_cps = v; break;
    case ParamId::kZeroTimeoutMs: c.velocity.zero_timeout_us = u * 1000u; break;
    case ParamId::kControlRateHz:
    case ParamId::kProtocolVersion:
      return ParamStatus::kReadOnly;
  }
  return ParamStatus::kOk;
}

}  // namespace odocore
