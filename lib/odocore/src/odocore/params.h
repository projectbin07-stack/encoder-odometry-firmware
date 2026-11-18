// Runtime parameters reachable through PARAM_GET / PARAM_SET.
//
// Each parameter has a one-byte id, a float value on the wire, a valid range
// and a read-only flag. Integer parameters are rounded on set. The table is
// mirrored in host/src/odobridge/params.py and documented in docs/protocol.md.
#pragma once

#include <stdint.h>

#include "odocore/drive_controller.h"

namespace odocore {

enum class ParamId : uint8_t {
  kKp = 1,
  kKi = 2,
  kKd = 3,
  kKff = 4,
  kKs = 5,
  kOutputLimit = 6,
  kSlewRate = 7,
  kDCutoffHz = 8,
  kMaxWheelSpeed = 9,
  kWatchdogMs = 10,
  kWheelRadius = 11,
  kHalfLength = 12,
  kHalfWidth = 13,
  kRollerAngleDeg = 14,
  kTicksPerWheelRev = 15,
  kDriveType = 16,
  kTelemetryDivider = 17,
  kEstimatorLowCps = 18,
  kEstimatorHighCps = 19,
  kZeroTimeoutMs = 20,
  kControlRateHz = 21,
  kProtocolVersion = 22,
};

enum class ParamStatus : uint8_t {
  kOk = 0,
  kUnknownId = 1,
  kOutOfRange = 2,
  kReadOnly = 3,
};

struct ParamInfo {
  uint8_t id;
  const char* name;
  float min;
  float max;
  bool read_only;
  bool integer;
};

/// Table entry for `id`, or nullptr.
const ParamInfo* find_param(uint8_t id);
/// Number of table entries and access by position (for listing).
int param_count();
const ParamInfo& param_at(int index);

ParamStatus get_param(const DriveConfig& config, uint8_t id, float* value);
/// Validates range, then writes the field. On anything but kOk the config is
/// unchanged. The caller must call DriveController::apply_config afterwards.
ParamStatus set_param(DriveConfig& config, uint8_t id, float value);

}  // namespace odocore
