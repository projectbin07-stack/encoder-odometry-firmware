#include "odocore/protocol.h"

#include <string.h>

#include <cmath>

#include "odocore/framing.h"

namespace odocore {

namespace {

class Writer {
 public:
  Writer(uint8_t* buf, size_t cap) : buf_(buf), cap_(cap) {}
  void u8(uint8_t v) {
    if (len_ + 1 > cap_) {
      ok_ = false;
      return;
    }
    buf_[len_++] = v;
  }
  void u16(uint16_t v) {
    u8(static_cast<uint8_t>(v & 0xFFu));
    u8(static_cast<uint8_t>(v >> 8));
  }
  void u32(uint32_t v) {
    for (int i = 0; i < 4; ++i) {
      u8(static_cast<uint8_t>((v >> (8 * i)) & 0xFFu));
    }
  }
  void i16(int16_t v) { u16(static_cast<uint16_t>(v)); }
  void i32(int32_t v) { u32(static_cast<uint32_t>(v)); }
  void f32(float v) {
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(v), "float must be 32-bit");
    memcpy(&bits, &v, sizeof(bits));
    u32(bits);
  }
  bool ok() const { return ok_; }
  size_t size() const { return len_; }
  uint8_t* data() { return buf_; }

 private:
  uint8_t* buf_;
  size_t cap_;
  size_t len_ = 0;
  bool ok_ = true;
};

class Reader {
 public:
  explicit Reader(const uint8_t* buf) : buf_(buf) {}
  uint8_t u8() { return buf_[pos_++]; }
  uint16_t u16() {
    const uint16_t lo = u8();
    const uint16_t hi = u8();
    return static_cast<uint16_t>(lo | (hi << 8));
  }
  uint32_t u32() {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
      v |= static_cast<uint32_t>(u8()) << (8 * i);
    }
    return v;
  }
  int16_t i16() { return static_cast<int16_t>(u16()); }
  int32_t i32() { return static_cast<int32_t>(u32()); }
  float f32() {
    const uint32_t bits = u32();
    float v = 0.0f;
    memcpy(&v, &bits, sizeof(v));
    return v;
  }

 private:
  const uint8_t* buf_;
  size_t pos_ = 0;
};

void write_body(const Message& m, Writer& w) {
  switch (m.type) {
    case MsgType::kVelocityCommand:
      w.f32(m.velocity_command.vx);
      w.f32(m.velocity_command.vy);
      w.f32(m.velocity_command.wz);
      break;
    case MsgType::kWheelState:
      w.u32(m.wheel_state.t_us);
      for (int i = 0; i < 4; ++i) {
        w.i32(m.wheel_state.count[i]);
        w.f32(m.wheel_state.velocity[i]);
        w.f32(m.wheel_state.target[i]);
        w.i16(m.wheel_state.duty[i]);
      }
      break;
    case MsgType::kOdometry:
      w.u32(m.odometry.t_us);
      w.f32(m.odometry.x);
      w.f32(m.odometry.y);
      w.f32(m.odometry.theta);
      w.f32(m.odometry.vx);
      w.f32(m.odometry.vy);
      w.f32(m.odometry.wz);
      break;
    case MsgType::kDiagnostics:
      w.u32(m.diagnostics.t_us);
      for (int i = 0; i < 4; ++i) {
        w.u32(m.diagnostics.illegal_transitions[i]);
      }
      w.u32(m.diagnostics.loop_overruns);
      w.u32(m.diagnostics.watchdog_trips);
      w.u32(m.diagnostics.max_loop_us);
      w.u32(m.diagnostics.rx_crc_errors);
      w.u32(m.diagnostics.rx_frame_errors);
      w.u8(m.diagnostics.flags);
      break;
    case MsgType::kParamGet:
      w.u8(m.param_get.id);
      break;
    case MsgType::kParamSet:
      w.u8(m.param_set.id);
      w.f32(m.param_set.value);
      break;
    case MsgType::kParamValue:
      w.u8(m.param_value.id);
      w.u8(m.param_value.status);
      w.f32(m.param_value.value);
      break;
    case MsgType::kSetPose:
      w.f32(m.set_pose.x);
      w.f32(m.set_pose.y);
      w.f32(m.set_pose.theta);
      break;
  }
}

void read_body(Reader& r, Message* m) {
  switch (m->type) {
    case MsgType::kVelocityCommand:
      m->velocity_command.vx = r.f32();
      m->velocity_command.vy = r.f32();
      m->velocity_command.wz = r.f32();
      break;
    case MsgType::kWheelState:
      m->wheel_state.t_us = r.u32();
      for (int i = 0; i < 4; ++i) {
        m->wheel_state.count[i] = r.i32();
        m->wheel_state.velocity[i] = r.f32();
        m->wheel_state.target[i] = r.f32();
        m->wheel_state.duty[i] = r.i16();
      }
      break;
    case MsgType::kOdometry:
      m->odometry.t_us = r.u32();
      m->odometry.x = r.f32();
      m->odometry.y = r.f32();
      m->odometry.theta = r.f32();
      m->odometry.vx = r.f32();
      m->odometry.vy = r.f32();
      m->odometry.wz = r.f32();
      break;
    case MsgType::kDiagnostics:
      m->diagnostics.t_us = r.u32();
      for (int i = 0; i < 4; ++i) {
        m->diagnostics.illegal_transitions[i] = r.u32();
      }
      m->diagnostics.loop_overruns = r.u32();
      m->diagnostics.watchdog_trips = r.u32();
      m->diagnostics.max_loop_us = r.u32();
      m->diagnostics.rx_crc_errors = r.u32();
      m->diagnostics.rx_frame_errors = r.u32();
      m->diagnostics.flags = r.u8();
      break;
    case MsgType::kParamGet:
      m->param_get.id = r.u8();
      break;
    case MsgType::kParamSet:
      m->param_set.id = r.u8();
      m->param_set.value = r.f32();
      break;
    case MsgType::kParamValue:
      m->param_value.id = r.u8();
      m->param_value.status = r.u8();
      m->param_value.value = r.f32();
      break;
    case MsgType::kSetPose:
      m->set_pose.x = r.f32();
      m->set_pose.y = r.f32();
      m->set_pose.theta = r.f32();
      break;
  }
}

}  // namespace

int body_size(uint8_t type) {
  switch (static_cast<MsgType>(type)) {
    case MsgType::kVelocityCommand:
      return 12;
    case MsgType::kWheelState:
      return 4 + 4 * (4 + 4 + 4 + 2);
    case MsgType::kOdometry:
      return 4 + 6 * 4;
    case MsgType::kDiagnostics:
      return 4 + 4 * 4 + 5 * 4 + 1;
    case MsgType::kParamGet:
      return 1;
    case MsgType::kParamSet:
      return 5;
    case MsgType::kParamValue:
      return 6;
    case MsgType::kSetPose:
      return 12;
  }
  return -1;
}

size_t encode_packet(const Message& msg, uint8_t* out, size_t capacity) {
  const int size = body_size(static_cast<uint8_t>(msg.type));
  if (size < 0) {
    return 0;
  }
  Writer w(out, capacity);
  w.u8(kProtocolVersion);
  w.u8(static_cast<uint8_t>(msg.type));
  w.u8(msg.seq);
  w.u8(static_cast<uint8_t>(size));
  write_body(msg, w);
  if (!w.ok()) {
    return 0;
  }
  const uint16_t crc = crc16_ccitt(out, w.size());
  w.u16(crc);
  return w.ok() ? w.size() : 0;
}

size_t encode_frame(const Message& msg, uint8_t* out, size_t capacity) {
  uint8_t packet[kMaxPacketSize];
  const size_t n = encode_packet(msg, packet, sizeof(packet));
  if (n == 0 || capacity < 1) {
    return 0;
  }
  const size_t encoded = cobs_encode(packet, n, out, capacity - 1);
  if (encoded == 0) {
    return 0;
  }
  out[encoded] = 0x00;
  return encoded + 1;
}

DecodeStatus decode_packet(const uint8_t* p, size_t length, Message* out) {
  if (length < kPacketHeaderSize + kPacketCrcSize) {
    return DecodeStatus::kTooShort;
  }
  const size_t covered = length - kPacketCrcSize;
  const uint16_t expected = static_cast<uint16_t>(p[covered] | (p[covered + 1] << 8));
  if (crc16_ccitt(p, covered) != expected) {
    return DecodeStatus::kBadCrc;
  }
  if (p[0] != kProtocolVersion) {
    return DecodeStatus::kBadVersion;
  }
  const int size = body_size(p[1]);
  if (size < 0) {
    return DecodeStatus::kUnknownType;
  }
  if (p[3] != size || covered != kPacketHeaderSize + static_cast<size_t>(size)) {
    return DecodeStatus::kBadLength;
  }
  Message m;
  m.type = static_cast<MsgType>(p[1]);
  m.seq = p[2];
  Reader r(p + kPacketHeaderSize);
  read_body(r, &m);
  *out = m;
  return DecodeStatus::kOk;
}

bool FrameReceiver::push(uint8_t byte, Message* out) {
  if (byte != 0x00) {
    if (length_ < sizeof(buffer_)) {
      buffer_[length_++] = byte;
    } else {
      overflow_ = true;
    }
    return false;
  }
  // Delimiter: process what was collected.
  const size_t n = length_;
  const bool overflow = overflow_;
  reset();
  if (n == 0) {
    return false;  // back-to-back delimiters are harmless (used to resync)
  }
  if (overflow) {
    ++frame_errors_;
    return false;
  }
  uint8_t packet[kMaxFrameSize];
  size_t packet_len = 0;
  if (!cobs_decode(buffer_, n, packet, sizeof(packet), &packet_len)) {
    ++frame_errors_;
    return false;
  }
  const DecodeStatus status = decode_packet(packet, packet_len, out);
  if (status == DecodeStatus::kOk) {
    ++frames_ok_;
    return true;
  }
  if (status == DecodeStatus::kBadCrc) {
    ++crc_errors_;
  } else {
    ++frame_errors_;
  }
  return false;
}

int16_t duty_to_wire(float duty) {
  if (!std::isfinite(duty)) {
    return 0;
  }
  float scaled = duty * 10000.0f;
  if (scaled > 10000.0f) {
    scaled = 10000.0f;
  } else if (scaled < -10000.0f) {
    scaled = -10000.0f;
  }
  return static_cast<int16_t>(std::lround(scaled));
}

}  // namespace odocore
