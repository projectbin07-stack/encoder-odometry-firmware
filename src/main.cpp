// encoder-odometry-firmware: four quadrature encoders, per-wheel velocity
// loops and wheel odometry over a COBS/CRC-16 serial protocol.
//
// Everything that can be tested without hardware lives in lib/odocore. This
// file wires it to the board: encoders, motor drivers, a fixed-rate timer and
// the serial port. The serial port carries only binary frames (no text), so
// the host decoder never has to skip log lines; the ESP32 ROM bootloader's
// boot message is the one exception and is discarded as a framing error.
#include <Arduino.h>

#include <odocore/drive_controller.h>
#include <odocore/node.h>

#include "board_config.h"
#include "control_timer.h"
#include "encoders.h"
#include "motor_driver.h"

namespace {

class SerialSink : public odocore::FrameSink {
 public:
  void write_frame(const uint8_t* data, size_t length) override { Serial.write(data, length); }
};

odocore::DriveConfig make_config() {
  odocore::DriveConfig config;
  config.control_rate_hz = board::kControlRateHz;
  return config;
}

odocore::DriveController g_controller(make_config());
SerialSink g_sink;
odocore::FirmwareNode g_node(g_controller, g_sink);
odocore::Encoder** g_encoders = nullptr;
MotorDriver* g_motors[4] = {nullptr, nullptr, nullptr, nullptr};
uint32_t g_serviced_ticks = 0;

#if defined(ODO_DRIVER_DRV)
DrvDriver g_drivers[4] = {
    DrvDriver(board::kDrv[0], 0, 1),
    DrvDriver(board::kDrv[1], 2, 3),
    DrvDriver(board::kDrv[2], 4, 5),
    DrvDriver(board::kDrv[3], 6, 7),
};
#else
L298nDriver g_drivers[4] = {
    L298nDriver(board::kL298n[0], 0),
    L298nDriver(board::kL298n[1], 1),
    L298nDriver(board::kL298n[2], 2),
    L298nDriver(board::kL298n[3], 3),
};
#endif

}  // namespace

void setup() {
  Serial.begin(board::kBaud);
  for (int i = 0; i < 4; ++i) {
    g_motors[i] = &g_drivers[i];
    g_motors[i]->begin();
    g_motors[i]->write(0.0f);
  }
  g_encoders = encoders_begin(micros());
  g_node.set_hardware_counters(encoders_are_hardware());
  g_node.loop_monitor().set_period_us(1000000u / board::kControlRateHz);
  control_timer_begin(board::kControlRateHz);
  g_serviced_ticks = control_timer_ticks();
}

void loop() {
  while (Serial.available() > 0) {
    g_node.receive_byte(static_cast<uint8_t>(Serial.read()), micros());
  }

  const uint32_t ticks = control_timer_ticks();
  const uint32_t pending = ticks - g_serviced_ticks;
  if (pending == 0) {
    return;
  }
  g_serviced_ticks = ticks;

  const uint32_t start = micros();
  odocore::EncoderSnapshot snapshots[4];
  for (int i = 0; i < 4; ++i) {
    snapshots[i] = g_encoders[i]->sample(start);
  }
  float duty[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  g_controller.step(snapshots, start, duty);
  for (int i = 0; i < 4; ++i) {
    g_motors[i]->write(duty[i]);
  }
  g_node.after_step(start);
  // More than one pending tick means at least one period was skipped.
  g_node.loop_monitor().record(start, micros(), pending - 1u);
}
