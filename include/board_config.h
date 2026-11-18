// Board pin maps and build-time options.
//
// One block per board, selected by the ODO_BOARD_* flag set in platformio.ini.
// Wheel order everywhere is FL, FR, RL, RR (front-left, front-right,
// rear-left, rear-right), matching mecanum-kinematics.
//
// Polarity: the firmware assumes a positive duty drives that wheel's side of
// the robot forward and makes its encoder count up. Motors on the left and
// right are mirror images, so one side is usually reversed. Fix that with the
// `invert` flags below (or by swapping the motor leads / encoder A and B)
// before tuning anything; see "Polarity check" in the README.
#pragma once

#include <stdint.h>

namespace board {

struct EncoderPins {
  uint8_t a;
  uint8_t b;
  bool invert;  ///< negate the count (swap direction)
};

/// L298N style: EN = PWM, IN1 / IN2 = direction (one channel of a dual board).
struct L298nPins {
  uint8_t en;
  uint8_t in1;
  uint8_t in2;
  bool invert;  ///< negate the duty (motor wired backwards)
};

/// DRV8833 / DRV8871 / TB6612-IN-IN style: both inputs are PWM.
struct DrvPins {
  uint8_t in1;
  uint8_t in2;
  bool invert;
};

constexpr uint32_t kControlRateHz = 100;  ///< fixed-rate control loop
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr uint8_t kPwmResolutionBits = 10;
constexpr uint32_t kPwmMax = (1u << kPwmResolutionBits) - 1u;

}  // namespace board

#if defined(ODO_BOARD_ESP32DEV)
// ESP32 DevKit (WROOM-32, 38-pin). GPIO 34/35/36/39 are input-only and have
// no internal pull-ups: use encoders with push-pull outputs or add 4.7k-10k
// pull-ups to 3.3 V. GPIO 5 is a strapping pin (fine as an output).
// GPIO 16/17 are not available on WROVER modules (PSRAM).
namespace board {
constexpr const char* kName = "esp32dev";
constexpr uint32_t kBaud = 921600;
constexpr EncoderPins kEncoders[4] = {
    {34, 35, false},  // FL
    {36, 39, true},   // FR (mirrored side)
    {32, 33, false},  // RL
    {25, 26, true},   // RR (mirrored side)
};
constexpr L298nPins kL298n[4] = {
    {13, 14, 27, false},  // FL  driver A: ENA, IN1, IN2
    {23, 22, 21, true},   // FR  driver A: ENB, IN3, IN4
    {19, 18, 5, false},   // RL  driver B: ENA, IN1, IN2
    {4, 16, 17, true},    // RR  driver B: ENB, IN3, IN4
};
constexpr DrvPins kDrv[4] = {
    {13, 14, false},  // FL
    {23, 22, true},   // FR
    {19, 18, false},  // RL
    {4, 16, true},    // RR
};
}  // namespace board

#elif defined(ODO_BOARD_ESP32S3)
// ESP32-S3-DevKitC-1. Avoids strapping pins 0/3/45/46, USB 19/20, the
// octal-PSRAM pins 35-37 and the RGB LED on 38/48.
namespace board {
constexpr const char* kName = "esp32-s3-devkitc-1";
constexpr uint32_t kBaud = 921600;
constexpr EncoderPins kEncoders[4] = {
    {4, 5, false},    // FL
    {6, 7, true},     // FR
    {15, 16, false},  // RL
    {17, 18, true},   // RR
};
constexpr L298nPins kL298n[4] = {
    {8, 9, 10, false},    // FL
    {11, 12, 13, true},   // FR
    {14, 21, 41, false},  // RL
    {1, 2, 42, true},     // RR
};
constexpr DrvPins kDrv[4] = {
    {8, 9, false},
    {11, 12, true},
    {14, 21, false},
    {1, 2, true},
};
}  // namespace board

#elif defined(ODO_BOARD_NUCLEO_F401RE)
// NUCLEO-F401RE. Encoders use four hardware timers in encoder mode (pins fixed
// by the timer alternate functions); motor PWM uses TIM3 CH1-4 on PC6-PC9.
// Serial is the ST-LINK virtual COM port (USART2 on PA2/PA3).
#include <Arduino.h>
namespace board {
constexpr const char* kName = "nucleo_f401re";
constexpr uint32_t kBaud = 460800;
/// Timer encoder inputs; listed for documentation and for the README table.
/// FL TIM1 PA8/PA9 (D7/D8), FR TIM2 PA15/PB3 (morpho/D3),
/// RL TIM4 PB6/PB7 (D10/morpho), RR TIM5 PA0/PA1 (A0/A1).
constexpr bool kEncoderInvert[4] = {false, true, false, true};
static const L298nPins kL298n[4] = {
    {PC6, PB12, PB13, false},  // FL
    {PC7, PB14, PB15, true},   // FR
    {PC8, PC10, PC11, false},  // RL
    {PC9, PC12, PD2, true},    // RR
};
}  // namespace board

#else
#error "Select a board: build with -D ODO_BOARD_ESP32DEV, ODO_BOARD_ESP32S3 or ODO_BOARD_NUCLEO_F401RE"
#endif
