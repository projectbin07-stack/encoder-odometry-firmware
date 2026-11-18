#include "odocore/node.h"

#include "odocore/params.h"

namespace odocore {

void LoopMonitor::record(uint32_t start_us, uint32_t end_us, uint32_t missed_ticks) {
  const uint32_t duration = end_us - start_us;
  last_loop_us_ = duration;
  if (duration > max_loop_us_) {
    max_loop_us_ = duration;
  }
  overruns_ += missed_ticks;
  if (duration > period_us_) {
    ++overruns_;
  }
}

void FirmwareNode::receive_byte(uint8_t byte, uint32_t now_us) {
  Message msg;
  if (receiver_.push(byte, &msg)) {
    handle(msg, now_us);
  }
}

void FirmwareNode::send(Message& msg) {
  uint8_t frame[kMaxFrameSize];
  msg.seq = tx_seq_++;
  const size_t n = encode_frame(msg, frame, sizeof(frame));
  if (n > 0) {
    sink_.write_frame(frame, n);
    ++frames_sent_;
  }
}

void FirmwareNode::handle(const Message& msg, uint32_t now_us) {
  switch (msg.type) {
    case MsgType::kVelocityCommand: {
      Twist t;
      t.vx = msg.velocity_command.vx;
      t.vy = msg.velocity_command.vy;
      t.wz = msg.velocity_command.wz;
      controller_.command(t, now_us);
      break;
    }
    case MsgType::kSetPose: {
      Pose2D p;
      p.x = msg.set_pose.x;
      p.y = msg.set_pose.y;
      p.theta = msg.set_pose.theta;
      controller_.set_pose(p);
      Message reply = make_odometry(now_us);
      send(reply);
      break;
    }
    case MsgType::kParamGet:
    case MsgType::kParamSet: {
      Message reply;
      reply.type = MsgType::kParamValue;
      ParamStatus status = ParamStatus::kOk;
      uint8_t id = 0;
      if (msg.type == MsgType::kParamSet) {
        id = msg.param_set.id;
        status = set_param(controller_.config(), id, msg.param_set.value);
        if (status == ParamStatus::kOk) {
          controller_.apply_config();
        }
      } else {
        id = msg.param_get.id;
      }
      float value = 0.0f;
      const ParamStatus read_status = get_param(controller_.config(), id, &value);
      if (status == ParamStatus::kOk) {
        status = read_status;
      }
      reply.param_value.id = id;
      reply.param_value.status = static_cast<uint8_t>(status);
      reply.param_value.value = value;
      send(reply);
      break;
    }
    case MsgType::kWheelState:
    case MsgType::kOdometry:
    case MsgType::kDiagnostics:
    case MsgType::kParamValue:
      // Firmware-to-host messages arriving at the firmware are ignored.
      break;
  }
}

Message FirmwareNode::make_wheel_state(uint32_t now_us) const {
  Message m;
  m.type = MsgType::kWheelState;
  m.wheel_state.t_us = now_us;
  for (int i = 0; i < kWheelCount; ++i) {
    const WheelStatus& w = controller_.wheel(i);
    m.wheel_state.count[i] = w.count;
    m.wheel_state.velocity[i] = w.velocity;
    m.wheel_state.target[i] = w.target;
    m.wheel_state.duty[i] = duty_to_wire(w.duty);
  }
  return m;
}

Message FirmwareNode::make_odometry(uint32_t now_us) const {
  Message m;
  m.type = MsgType::kOdometry;
  const Pose2D& p = controller_.pose();
  const Twist& v = controller_.measured_twist();
  m.odometry.t_us = now_us;
  m.odometry.x = static_cast<float>(p.x);
  m.odometry.y = static_cast<float>(p.y);
  m.odometry.theta = static_cast<float>(wrap_angle(p.theta));
  m.odometry.vx = v.vx;
  m.odometry.vy = v.vy;
  m.odometry.wz = v.wz;
  return m;
}

Message FirmwareNode::make_diagnostics(uint32_t now_us) const {
  Message m;
  m.type = MsgType::kDiagnostics;
  DiagnosticsMsg& d = m.diagnostics;
  d.t_us = now_us;
  for (int i = 0; i < kWheelCount; ++i) {
    d.illegal_transitions[i] = controller_.wheel(i).illegal_transitions;
  }
  d.loop_overruns = loop_.overruns();
  d.watchdog_trips = controller_.watchdog_trips();
  d.max_loop_us = loop_.max_loop_us();
  d.rx_crc_errors = receiver_.crc_errors();
  d.rx_frame_errors = receiver_.frame_errors();
  uint8_t flags = 0;
  if (controller_.watchdog_expired()) {
    flags |= kDiagWatchdogExpired;
  }
  if (hardware_counters_) {
    flags |= kDiagHardwareCounters;
  }
  if (controller_.saturation_scale() < 1.0f) {
    flags |= kDiagSaturated;
  }
  d.flags = flags;
  return m;
}

void FirmwareNode::after_step(uint32_t now_us) {
  const uint32_t divider = controller_.config().telemetry_divider == 0
                               ? 1u
                               : controller_.config().telemetry_divider;
  if (period_counter_ % divider == 0) {
    Message ws = make_wheel_state(now_us);
    send(ws);
    Message od = make_odometry(now_us);
    send(od);
  }
  if (period_counter_ % (divider * 10u) == 0) {
    Message dg = make_diagnostics(now_us);
    send(dg);
  }
  ++period_counter_;
}

}  // namespace odocore
