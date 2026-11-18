# encoder-odometry-firmware

Firmware that reads four quadrature encoders, closes a velocity loop on each
wheel and streams wheel odometry over a binary serial link, plus a Python
host bridge (`odobridge`) for commanding it, logging it and simulating it.

[![CI](https://github.com/projectbin07-stack/encoder-odometry-firmware/actions/workflows/ci.yml/badge.svg)](https://github.com/projectbin07-stack/encoder-odometry-firmware/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

> **Hardware validation status: nothing in this repository has been run on
> physical hardware.** The control, estimation, kinematics and protocol code is
> unit-tested on the host, and the firmware compiles for every listed board in
> CI. No board has been flashed, no encoder has been read, no motor has been
> driven. See [Hardware validation status](#hardware-validation-status).

## Why

A mecanum or differential base with encoder DC motors and cheap H-bridges
(L298N and friends) needs the same low-level layer every time: count encoder
edges without missing them, estimate wheel speed well at both crawl and full
speed, run a PI loop per wheel with sane limits, stop the motors when the
computer above stops talking, and send odometry upstream in a form that
survives a noisy USB cable. This project is that layer, written so that almost
all of it can be tested without a robot, and using the same wheel-order and
sign conventions as the [`mecanum-kinematics`](https://github.com/projectbin07-stack/mecanum-kinematics)
Python library so the two interoperate (the host tool reads its TOML robot
configs unchanged).

## Features

- **Quadrature decoding**: 4x state-table decoder with illegal-transition
  (missed-edge) counting and edge timestamps; or hardware counting with the
  ESP32 PCNT units or STM32 timers in encoder mode, behind the same interface.
  Narrow hardware counters are extended to 32 bits, including the ESP32 PCNT's
  reset-at-limit behaviour.
- **Velocity estimation**: fixed-period (counts per period), period / 1-over-T
  (time between edges) for low speed, and a blend that switches between them
  by speed. 32-bit count and `micros()` wraparound handled.
- **Per-wheel velocity control** in float (all supported MCUs have an FPU):
  feed-forward from a motor constant plus static friction, PI(D) with
  derivative on measurement, output saturation with conditional-integration
  anti-windup, slew-rate limiting, and a command watchdog.
- **Kinematics**: mecanum (any roller angle) and differential drive, inverse
  and least-squares forward, with uniform direction-preserving saturation.
- **Odometry** on SE(2) with exact arc integration; reset / set pose.
- **Protocol**: COBS framing, CRC-16/CCITT, versioned message types for
  velocity commands, wheel states, odometry, diagnostics (illegal transitions,
  loop overruns, watchdog trips, receive errors) and parameter get/set.
  [Byte layout](docs/protocol.md).
- **Drivers**: L298N style (PWM enable + two direction pins) and DRV style
  (two PWM inputs, e.g. DRV8833 / DRV8871), coast or brake.
- **Host tool** (`odobridge`, stdlib + pyserial): protocol codec, CLI to send
  velocity commands and log odometry to CSV, parameter access, push a
  `mecanum-kinematics` TOML to the firmware, and a pure-Python model of the
  firmware loop (`--port sim`, `odobridge simulate`) for testing without
  hardware.
- **Shared test vectors**: one JSON file of protocol byte vectors checked by
  both the C++ and the Python test suites.
- Optional ROS 2 node example (not run in CI).

## Supported hardware

| PlatformIO env | Board | Motor driver | Encoder back-end | CI build |
|---|---|---|---|---|
| `esp32dev` | ESP32 DevKit (WROOM-32) | L298N | PCNT (hardware) | yes |
| `esp32-s3-devkitc-1` | ESP32-S3-DevKitC-1 | L298N | PCNT (hardware) | yes |
| `esp32dev_drv_isr` | ESP32 DevKit | DRV style (IN/IN PWM) | pin-change ISR + software 4x decoder | yes |
| `nucleo_f401re` | ST NUCLEO-F401RE | L298N | TIM1/2/4/5 encoder mode | yes |
| `native` | host PC | - | - | unit tests |

The STM32 build uses the official STM32duino Arduino core (`ststm32` platform);
it compiles in CI. Any of the three `ODO_DRIVER_*` / `ODO_ENCODER_*`
combinations can be selected by build flags where the board has the pins; the
NUCLEO pin map only defines the L298N layout.

Motors: brushed DC gear motors with two-channel (A/B) quadrature encoders, Hall
or optical, for example the common JGB37-520 / 25GA-370 style motors with an
11 PPR Hall encoder. Encoder counts per wheel revolution (4x) = PPR x 4 x gear
ratio, set as `ticks_per_wheel_rev`.

## Wiring

Wheel order is always FL, FR, RL, RR (front-left, front-right, rear-left,
rear-right). Two dual-channel L298N boards drive the four motors: board A
channel A = FL, board A channel B = FR, board B channel A = RL, board B
channel B = RR. Connect all grounds (MCU, both drivers, encoder supply,
battery negative).

### ESP32 DevKit (`esp32dev`)

| Wheel | Encoder A | Encoder B | L298N EN (PWM) | IN1 | IN2 | Inverted |
|---|---|---|---|---|---|---|
| FL | GPIO34 | GPIO35 | GPIO13 (A: ENA) | GPIO14 (A: IN1) | GPIO27 (A: IN2) | no |
| FR | GPIO36 | GPIO39 | GPIO23 (A: ENB) | GPIO22 (A: IN3) | GPIO21 (A: IN4) | yes |
| RL | GPIO32 | GPIO33 | GPIO19 (B: ENA) | GPIO18 (B: IN1) | GPIO5 (B: IN2) | no |
| RR | GPIO25 | GPIO26 | GPIO4 (B: ENB) | GPIO16 (B: IN3) | GPIO17 (B: IN4) | yes |

GPIO34/35/36/39 are input-only and have no internal pull-ups: use encoders
with push-pull outputs or fit 4.7-10 kOhm pull-ups to 3.3 V. GPIO16/17 do not
exist on WROVER modules. `esp32dev_drv_isr` uses the EN and IN1 columns as the
two PWM inputs (IN1, IN2) of each DRV-style channel.

### ESP32-S3-DevKitC-1 (`esp32-s3-devkitc-1`)

| Wheel | Encoder A | Encoder B | EN (PWM) | IN1 | IN2 | Inverted |
|---|---|---|---|---|---|---|
| FL | GPIO4 | GPIO5 | GPIO8 | GPIO9 | GPIO10 | no |
| FR | GPIO6 | GPIO7 | GPIO11 | GPIO12 | GPIO13 | yes |
| RL | GPIO15 | GPIO16 | GPIO14 | GPIO21 | GPIO41 | no |
| RR | GPIO17 | GPIO18 | GPIO1 | GPIO2 | GPIO42 | yes |

Avoids the strapping pins (0, 3, 45, 46), USB (19, 20), octal-PSRAM pins
(35-37) and the RGB LED pins (38, 48).

### NUCLEO-F401RE (`nucleo_f401re`)

| Wheel | Encoder A | Encoder B | Timer | EN (PWM, TIM3) | IN1 | IN2 | Inverted |
|---|---|---|---|---|---|---|---|
| FL | PA8 (D7) | PA9 (D8) | TIM1 | PC6 (CH1) | PB12 | PB13 | no |
| FR | PA15 | PB3 (D3) | TIM2 | PC7 (D9, CH2) | PB14 | PB15 | yes |
| RL | PB6 (D10) | PB7 | TIM4 | PC8 (CH3) | PC10 | PC11 | no |
| RR | PA0 (A0) | PA1 (A1) | TIM5 | PC9 (CH4) | PC12 | PD2 | yes |

Encoder pins are fixed by the timers' alternate functions. Serial goes over
the ST-LINK virtual COM port (USART2) at 460800 baud. TIM11 provides the
control tick.

Pin maps live in [`include/board_config.h`](include/board_config.h); the
`invert` flags there (or swapping a motor's leads / an encoder's A and B) fix
polarity.

### Encoder hookup

- Most Hall encoder boards take 3.3-5 V. **The ESP32 is not 5 V tolerant**:
  power the encoder from 3.3 V, or level-shift its outputs. On the STM32 check
  the FT (5 V tolerant) column of the datasheet for each pin before feeding it
  5 V signals.
- Keep encoder wires away from motor leads, twist each signal with ground if
  the run is long, and add 100 nF across the motor terminals. Missed or extra
  edges show up as `illegal_transitions` in DIAGNOSTICS (software decoder) or
  as count drift (hardware counters).
- Hardware back-ends apply a glitch filter (ESP32 PCNT: ~1.25 us, STM32 timer:
  ~3 us at an 84 MHz timer clock).

### L298N hookup

- 12 V (or your motor supply) to `+12V`, ground to `GND`. Leave the 5 V
  regulator jumper fitted only if the supply is at most 12 V; do not power the
  MCU from the L298N's 5 V pin and USB at the same time.
- Remove the ENA/ENB jumpers and wire EN to the PWM pin in the table. The
  L298N's inputs accept 3.3 V logic.
- The L298N drops roughly 2-4 V at load currents and switches slowly; PWM is
  20 kHz (`kPwmFrequencyHz` in `board_config.h`). If the motors whine or run
  hot at low duty, try a lower frequency.
- Zero duty brakes (EN high, IN1 = IN2 = low). The watchdog therefore brakes
  the wheels when commands stop.

## Build and flash

Requires [PlatformIO](https://platformio.org/) (CLI or VS Code extension).

```sh
pio run -e esp32dev                    # build
pio run -e esp32dev -t upload          # flash over USB
pio test -e native                     # host unit tests, no hardware
```

For the NUCLEO board, `pio run -e nucleo_f401re -t upload` flashes through the
on-board ST-LINK. The serial port carries binary frames only; use
`odobridge monitor` rather than a text serial monitor.

### Firmware size (from CI)

Copied from the `pio run` output of CI run
[44385102376](https://github.com/projectbin07-stack/encoder-odometry-firmware/actions/runs/44385102376)
(PlatformIO 6.1.18, espressif32 6.9.0, ststm32 19.7.1):

| Env | RAM | Flash |
|---|---|---|
| `esp32dev` | 22 740 B of 327 680 B (6.9%) | 306 013 B of 1 310 720 B (23.3%) |
| `esp32-s3-devkitc-1` | 20 140 B of 327 680 B (6.1%) | 311 609 B of 3 342 336 B (9.3%) |
| `esp32dev_drv_isr` | 23 204 B of 327 680 B (7.1%) | 303 601 B of 1 310 720 B (23.2%) |
| `nucleo_f401re` | 2 736 B of 98 304 B (2.8%) | 39 520 B of 524 288 B (7.5%) |

Most of the ESP32 figure is the Arduino core and ESP-IDF, not this project.

## Host tool

```sh
pip install -e "./host[dev]"     # or: pip install ./host
odobridge --help
```

```
usage: odobridge [-h] [--version]
                 {drive,monitor,param,push-config,simulate,decode,vectors} ...

Host bridge for encoder-odometry-firmware.

positional arguments:
  {drive,monitor,param,push-config,simulate,decode,vectors}
    drive               stream a velocity command and log odometry
    monitor             log telemetry without commanding motion
    param               list, read or write firmware parameters
    push-config         send a robot TOML's geometry to the firmware
    simulate            run a command against the Python firmware model
    decode              decode hex-encoded frames
    vectors             check or regenerate the shared protocol vectors
```

Every command that talks to firmware takes `--port COM5` / `--port
/dev/ttyUSB0`, or `--port sim` to run against the Python model instead.

### Quick start without hardware

The `simulate` command drives the firmware model (same estimator, PI
controller, watchdog, kinematics and odometry as the C++ core, plus a simple
made-up DC motor model) through the real protocol encoder and decoder. This is
real output, with the `generic-medium.toml` config from `mecanum-kinematics`:

```console
$ odobridge simulate --config generic-medium.toml --vx 0.3 --wz 0.5 --duration 3
simulated 3 s of (0.3, 0, 0.5) on generic-medium
odometry (firmware model): x=+0.5950 m  y=+0.5517 m  theta=+1.5003 rad  vx=+0.001 vy=+0.000 wz=+0.002
ground truth (plant):      x=+0.5950 m  y=+0.5517 m  theta=+1.5003 rad
odometry position error:   0.01 mm (encoder quantisation only; no slip modelled)
watchdog trips: 0  frames: 150  saturation scale: 1.000
```

The tiny error says only that the odometry maths is consistent with the
simulated plant. The model has no wheel slip, backlash or encoder noise, so
it says nothing about real-world odometry accuracy.

### With a robot

```sh
odobridge push-config my-robot.toml --port /dev/ttyUSB0      # geometry + encoder counts
odobridge param set kff 0.034 --port /dev/ttyUSB0
odobridge drive --port /dev/ttyUSB0 --vx 0.2 --duration 3 --reset-pose --csv run1.csv
odobridge monitor --port /dev/ttyUSB0 --duration 10 -v
```

`push-config` accepts any `mecanum-kinematics` TOML (`wheel_radius`,
`half_length`, `half_width`, `roller_angle_deg`, `ticks_per_rev`,
`gear_ratio`, `max_wheel_speed`, `wheel_order`). An optional
`drive = "differential"` key selects differential drive; `mecanum-kinematics`
itself would reject that key, so keep it out of files you share with it.
Parameter changes are not saved on the board; push them again after a reset.

`odobridge param list` prints the parameter table; the full list with defaults
is in [docs/protocol.md](docs/protocol.md#0x05-param_get-1-byte-0x06-param_set-5-bytes-0x07-param_value-6-bytes).

## Protocol summary

```
frame  = COBS(packet) 0x00
packet = version:u8 type:u8 seq:u8 length:u8 body crc16:u16le     (little-endian, f32 floats)
```

| Type | Message | Direction |
|---|---|---|
| 0x01 | VELOCITY_COMMAND (vx, vy, wz) | host -> firmware |
| 0x02 | WHEEL_STATE (count, speed, target, duty per wheel) | firmware -> host, 20 Hz |
| 0x03 | ODOMETRY (x, y, theta, vx, vy, wz) | firmware -> host, 20 Hz |
| 0x04 | DIAGNOSTICS (illegal transitions, overruns, watchdog trips, rx errors) | firmware -> host, 2 Hz |
| 0x05 / 0x06 / 0x07 | PARAM_GET / PARAM_SET / PARAM_VALUE | request / reply |
| 0x08 | SET_POSE | host -> firmware |

Full byte layout, error handling and a worked example: [docs/protocol.md](docs/protocol.md).

## Control-loop tuning notes

Work through these in order, with the wheels off the ground first.

1. **Encoder counts**: set `ticks_per_wheel_rev` (PPR x 4 x gear ratio, or
   `ticks_per_rev x gear_ratio` from a `mecanum-kinematics` TOML). Turn one
   wheel one revolution by hand and check that WHEEL_STATE's count changes by
   that amount.
2. **Polarity check**: with `kp = ki = 0` and a small `kff`, command
   `--vx 0.1`. Every wheel must spin so that its side of the robot would move
   forward, and every count must increase. Fix any wheel that does not with
   the `invert` flags in `board_config.h`. Do not tune anything until this is
   right; a reversed encoder turns the PI loop into positive feedback.
3. **Feed-forward**: with `kp = ki = 0`, command a few speeds and read
   `duty` and `velocity` from WHEEL_STATE (`odobridge monitor -v`). The slope
   of duty against speed is `kff` (duty per rad/s); the duty at which the
   wheel just starts moving is `ks`. A good feed-forward does most of the
   work; the PI terms only correct the remainder.
4. **PI**: raise `ki` until a speed step settles without a long tail, then add
   `kp` to speed up the response; back off if the wheel speed oscillates.
   The defaults (`kp = 0.02`, `ki = 0.2`) are deliberately soft. `kd` is
   rarely useful on a velocity loop with a gearbox and is 0 by default.
5. **Slew rate** (`slew_rate`, duty per second, default 4): limits current
   spikes and gearbox shock on steps; lower it for heavy robots.
6. **Speed limit**: set `max_wheel_speed` a little below the no-load wheel
   speed at your battery voltage so the controller keeps some headroom.
   Commands above it are scaled down uniformly, which keeps the path.
7. **Estimator thresholds**: below `estimator_low_cps` the 1/T estimate is
   used, above `estimator_high_cps` counts per period. At 100 Hz, one count per
   period is 100 counts/s, so the defaults (200 / 800) switch at 2 to 8 counts
   per period. With hardware counters the 1/T estimate is quantised to the
   control period, so its benefit is smaller.
8. **Watchdog**: the host must send VELOCITY_COMMAND faster than
   `watchdog_ms` (default 500 ms). `odobridge drive` repeats at 20 Hz.

Overruns: the control step should take a small fraction of the 10 ms period;
`max_loop_us` and `loop_overruns` in DIAGNOSTICS tell you if it does not.

## Architecture

```
lib/odocore/            hardware-independent C++17 core (no Arduino headers)
  quadrature.*          4x decoder, counter extender, Encoder interface
  velocity.*            fixed-period / period / blended estimators
  pid.*, watchdog.h     per-wheel velocity controller, command watchdog
  kinematics.*          mecanum + differential, saturation
  odometry.*            SE(2) exponential map, pose integration
  framing.*, protocol.* CRC-16, COBS, message encode/decode, FrameReceiver
  params.*              runtime parameter table
  drive_controller.*    one control period: encoders -> speeds -> PI -> duties, odometry
  node.*                protocol <-> controller glue, telemetry schedule, loop monitor
  motor_output.*        duty -> L298N / DRV pin states
src/                    Arduino glue: encoder back-ends (PCNT, ISR, STM32 timers),
                        PWM, motor drivers, control timer, main loop
include/board_config.h  pin maps per board
test/                   Unity tests (pio test -e native) + shared protocol vectors
host/                   odobridge Python package, tests, ROS 2 example
docs/protocol.md        byte-level protocol specification
```

The control loop: a hardware timer ticks at 100 Hz and only increments a
counter. `loop()` drains the serial port, and when a tick is pending it
samples the four encoders, runs `DriveController::step`, writes the four
duties, queues telemetry and records the loop time.

## Testing

```sh
pio test -e native                               # 70 C++ test cases (CI)
cd host && pytest && ruff check . && ruff format --check . && mypy
odobridge vectors                                # shared vectors up to date
```

CI runs the native tests, builds all four board environments and prints their
sizes, and runs the host checks on Ubuntu and Windows with Python 3.10 and
3.13. The C++ and Python suites check the same hand-derived values for
kinematics and the same JSON byte vectors for the protocol (the CRC check value
and COBS cases in that file are the published reference values, not outputs
of this code).

## Hardware validation status

Stated plainly:

- **Not tested on any physical hardware.** No board listed above has been
  flashed with this firmware. No encoder, motor driver or motor has been
  connected. Pin maps, polarity, PWM frequency, PCNT / timer configuration and
  the control-tick timing are untested on silicon.
- **What is verified**: the C++ core passes its host unit tests in CI; every
  firmware environment compiles in CI; the Python host tool passes its tests
  on Linux and Windows; the C++ and Python protocol implementations agree on
  the shared byte vectors.
- **Default gains are placeholders** tuned only against the simulated motor
  model. They will need tuning on a real robot.
- The ROS 2 example has never been executed.

If you run it on hardware, please open an issue with what you used and what
happened, including anything that did not work, so this section can be
updated.

## Known limitations

- Parameters are not persisted to flash; they reset to the compiled defaults.
- No open-loop duty command; feed-forward is identified with `kp = ki = 0`
  as described above.
- Hardware counters (PCNT, STM32 timers) cannot detect missed edges, so
  `illegal_transitions` is always 0 with them, and their edge times are
  quantised to the control period.
- The ISR decoder's ceiling depends on interrupt latency; at very high edge
  rates it will miss edges (counted as illegal transitions).
- The protocol has no authentication; anyone with access to the serial port
  can drive the motors (see [SECURITY.md](SECURITY.md)).
- The NUCLEO-F401RE pin map defines only the L298N layout (the board has too
  few free timer channels for eight DRV-style PWM inputs alongside four
  encoder timers).
- The Python firmware model uses double precision where the firmware uses
  float32, so results agree to rounding, not bit-for-bit.

## Roadmap

- Validate on real hardware (ESP32 + L298N + JGB37-520 first) and record
  results here.
- Persist parameters (ESP32 NVS / STM32 flash page).
- Open-loop duty command and a host-side feed-forward identification routine.
- Optional IMU yaw fusion message.
- Covariance estimates in the ROS 2 example.

## License

MIT, see [LICENSE](LICENSE).
