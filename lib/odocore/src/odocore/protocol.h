// Binary serial protocol, version 1. Full byte layout in docs/protocol.md.
//
//   wire frame = COBS(packet) 0x00
//   packet     = version:u8 type:u8 seq:u8 length:u8 body[length] crc:u16
//
// All multi-byte fields are little-endian; floats are IEEE-754 binary32. The
// CRC (CRC-16/CCITT-FALSE) covers version..body and is sent low byte first.
// A packet whose version, type or body length does not match is rejected
// whole; there is no partial decoding.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace odocore {

constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kPacketHeaderSize = 4;
constexpr size_t kPacketCrcSize = 2;
constexpr size_t kMaxBodySize = 64;
constexpr size_t kMaxPacketSize = kPacketHeaderSize + kMaxBodySize + kPacketCrcSize;
/// COBS of the largest packet plus the 0x00 delimiter.
constexpr size_t kMaxFrameSize = kMaxPacketSize + kMaxPacketSize / 254u + 1u + 1u;

enum class MsgType : uint8_t {
  kVelocityCommand = 0x01,  ///< host -> firmware
  kWheelState = 0x02,       ///< firmware -> host
  kOdometry = 0x03,         ///< firmware -> host
  kDiagnostics = 0x04,      ///< firmware -> host
  kParamGet = 0x05,         ///< host -> firmware
  kParamSet = 0x06,         ///< host -> firmware
  kParamValue = 0x07,       ///< firmware -> host (reply to get and set)
  kSetPose = 0x08,          ///< host -> firmware
};

/// Body length of each message type, or -1 for an unknown type.
int body_size(uint8_t type);

struct VelocityCommandMsg {
  float vx;  ///< m/s
  float vy;  ///< m/s
  float wz;  ///< rad/s
};

struct WheelStateMsg {
  uint32_t t_us;
  int32_t count[4];   ///< 4x encoder counts, FL FR RL RR
  float velocity[4];  ///< measured wheel speed, rad/s
  float target[4];    ///< wheel speed set-point, rad/s
  int16_t duty[4];    ///< motor duty x 10000 (-10000..10000)
};

struct OdometryMsg {
  uint32_t t_us;
  float x;      ///< m
  float y;      ///< m
  float theta;  ///< rad, wrapped to (-pi, pi]
  float vx;     ///< measured body twist, m/s
  float vy;     ///< m/s
  float wz;     ///< rad/s
};

/// Bits of DiagnosticsMsg::flags.
enum DiagFlags : uint8_t {
  kDiagWatchdogExpired = 0x01,
  kDiagHardwareCounters = 0x02,
  kDiagSaturated = 0x04,
};

struct DiagnosticsMsg {
  uint32_t t_us;
  uint32_t illegal_transitions[4];
  uint32_t loop_overruns;
  uint32_t watchdog_trips;
  uint32_t max_loop_us;
  uint32_t rx_crc_errors;
  uint32_t rx_frame_errors;
  uint8_t flags;
};

struct ParamGetMsg {
  uint8_t id;
};

struct ParamSetMsg {
  uint8_t id;
  float value;
};

struct ParamValueMsg {
  uint8_t id;
  uint8_t status;  ///< ParamStatus
  float value;
};

struct SetPoseMsg {
  float x;
  float y;
  float theta;
};

struct Message {
  MsgType type = MsgType::kVelocityCommand;
  uint8_t seq = 0;
  union {
    VelocityCommandMsg velocity_command;
    WheelStateMsg wheel_state;
    OdometryMsg odometry;
    DiagnosticsMsg diagnostics;
    ParamGetMsg param_get;
    ParamSetMsg param_set;
    ParamValueMsg param_value;
    SetPoseMsg set_pose;
  };
  Message() : wheel_state() {}
};

enum class DecodeStatus : uint8_t {
  kOk = 0,
  kTooShort,
  kBadCrc,
  kBadVersion,
  kUnknownType,
  kBadLength,
};

/// Serialise a message into a packet (header + body + CRC). Returns the
/// packet size or 0 if `capacity` is too small.
size_t encode_packet(const Message& msg, uint8_t* out, size_t capacity);

/// Packet -> COBS -> trailing 0x00. Returns the frame size or 0.
size_t encode_frame(const Message& msg, uint8_t* out, size_t capacity);

/// Parse a packet (already COBS-decoded, delimiter removed).
DecodeStatus decode_packet(const uint8_t* packet, size_t length, Message* out);

/// Byte-at-a-time frame receiver with error counters.
class FrameReceiver {
 public:
  /// Feed one received byte. Returns true when a complete, valid message has
  /// been stored in `*out`.
  bool push(uint8_t byte, Message* out);
  void reset() { length_ = 0; overflow_ = false; }

  uint32_t frames_ok() const { return frames_ok_; }
  uint32_t crc_errors() const { return crc_errors_; }
  /// COBS errors, overlong frames, bad version / type / length.
  uint32_t frame_errors() const { return frame_errors_; }

 private:
  uint8_t buffer_[kMaxFrameSize] = {};
  size_t length_ = 0;
  bool overflow_ = false;
  uint32_t frames_ok_ = 0;
  uint32_t crc_errors_ = 0;
  uint32_t frame_errors_ = 0;
};

/// Duty in [-1, 1] -> int16 x 10000, rounded, clamped; NaN -> 0.
int16_t duty_to_wire(float duty);

}  // namespace odocore
