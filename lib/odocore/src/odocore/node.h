// Glue between the serial protocol and the DriveController: decodes incoming
// frames, applies commands and parameter changes, and schedules telemetry.
// The firmware feeds it received bytes and calls `after_step` once per
// control period; frames to send go to a FrameSink (the UART in firmware, a
// buffer in tests).
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "odocore/drive_controller.h"
#include "odocore/protocol.h"

namespace odocore {

class FrameSink {
 public:
  virtual ~FrameSink() = default;
  virtual void write_frame(const uint8_t* data, size_t length) = 0;
};

/// Loop timing statistics.
class LoopMonitor {
 public:
  explicit LoopMonitor(uint32_t period_us = 10000u) : period_us_(period_us) {}
  void set_period_us(uint32_t period_us) { period_us_ = period_us; }

  /// Record one control period. `missed_ticks` is the number of timer ticks
  /// that elapsed beyond the one being serviced (each is an overrun), and a
  /// run longer than the period is also an overrun.
  void record(uint32_t start_us, uint32_t end_us, uint32_t missed_ticks);

  uint32_t overruns() const { return overruns_; }
  uint32_t max_loop_us() const { return max_loop_us_; }
  uint32_t last_loop_us() const { return last_loop_us_; }

 private:
  uint32_t period_us_;
  uint32_t overruns_ = 0;
  uint32_t max_loop_us_ = 0;
  uint32_t last_loop_us_ = 0;
};

class FirmwareNode {
 public:
  FirmwareNode(DriveController& controller, FrameSink& sink) : controller_(controller), sink_(sink) {}

  /// Feed one received byte; handles a message when a frame completes.
  void receive_byte(uint8_t byte, uint32_t now_us);

  /// Call after each DriveController::step. Sends WHEEL_STATE and ODOMETRY
  /// every `telemetry_divider` periods and DIAGNOSTICS every 10x that.
  void after_step(uint32_t now_us);

  /// Handle an already-decoded message (exposed for tests).
  void handle(const Message& msg, uint32_t now_us);

  /// Encode and send one message with the next sequence number.
  void send(Message& msg);

  void set_hardware_counters(bool hardware) { hardware_counters_ = hardware; }
  LoopMonitor& loop_monitor() { return loop_; }
  const FrameReceiver& receiver() const { return receiver_; }
  uint32_t frames_sent() const { return frames_sent_; }

  Message make_wheel_state(uint32_t now_us) const;
  Message make_odometry(uint32_t now_us) const;
  Message make_diagnostics(uint32_t now_us) const;

 private:
  DriveController& controller_;
  FrameSink& sink_;
  FrameReceiver receiver_;
  LoopMonitor loop_;
  uint8_t tx_seq_ = 0;
  uint32_t period_counter_ = 0;
  uint32_t frames_sent_ = 0;
  bool hardware_counters_ = false;
};

}  // namespace odocore
