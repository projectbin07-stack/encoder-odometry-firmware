# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [0.2.0] - 2026-11-25

The host firmware model is now a documented, supported API. No behaviour
changes: the firmware and the model produce the same outputs as 0.1.0 for the
same inputs. Still not tested on physical hardware (see README).

### Added

- `docs/host-api.md`: the supported import paths, wheel order, sign, unit,
  clock and angle-wrapping conventions, `DriveController.step` semantics and a
  runnable example with its real output.
- `__all__` in `odobridge.firmware_model`, `odobridge.kinematics` and
  `odobridge.protocol`, and docstrings with units and conventions for every
  public name.
- `DriveConfig`, `DriveController`, `SimulatedRobot`, `wrap_angle`,
  `Geometry`, `Twist` and `duty_to_wire` re-exported from `odobridge`.
- `py.typed` marker (PEP 561).
- Tests for the documented behaviour, including the docs example output and
  a SHA-256 of the telemetry bytes of a fixed `SimulatedFirmware` run
  recorded with 0.1.0.

### Changed

- `odobridge` and `lib/odocore` version 0.2.0 (the firmware code is
  unchanged).

## [0.1.0] - 2026-11-19

First release. Not tested on physical hardware (see README).

### Added

- `lib/odocore` C++17 core: 4x quadrature state-table decoder with
  illegal-transition counting; hardware-counter extension for ESP32 PCNT and
  STM32 timers; fixed-period, period (1/T) and blended velocity estimators
  with 32-bit wraparound handling; per-wheel float PI(D) with feed-forward,
  anti-windup and slew limiting; command watchdog; mecanum and differential
  kinematics; SE(2) odometry; COBS / CRC-16 protocol version 1; parameter
  table; drive controller and protocol node; L298N and DRV-style output tables.
- Arduino firmware for `esp32dev`, `esp32-s3-devkitc-1`, `esp32dev_drv_isr`
  and `nucleo_f401re`.
- `odobridge` host package: protocol codec, CLI (`drive`, `monitor`, `param`,
  `push-config`, `simulate`, `decode`, `vectors`), pure-Python firmware
  model, reading of `mecanum-kinematics` TOML configs.
- Shared JSON protocol vectors checked by both test suites.
- ROS 2 bridge node example (not run in CI).
- CI: native unit tests, four firmware builds with size report, host tool on
  Ubuntu and Windows with Python 3.10 and 3.13.

[0.2.0]: https://github.com/projectbin07-stack/encoder-odometry-firmware/releases/tag/v0.2.0
[0.1.0]: https://github.com/projectbin07-stack/encoder-odometry-firmware/releases/tag/v0.1.0
