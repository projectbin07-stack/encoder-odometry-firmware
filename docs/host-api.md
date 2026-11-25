# odobridge firmware-model API

`odobridge` contains a pure-Python mirror of the firmware's control loop
(`lib/odocore`) plus a simple motor and encoder plant. Other projects can use
it as a test oracle: give it the same commands and clock as the firmware and
compare what comes out. This page describes the part of the package that is
supported for that use.

## Install

```sh
pip install "odobridge @ git+https://github.com/projectbin07-stack/encoder-odometry-firmware@v0.2.0#subdirectory=host"
```

Runtime dependencies: `pyserial` (and `tomli` on Python 3.10). Python >= 3.10.
The package ships a `py.typed` marker, so mypy and other type checkers use its
annotations.

## Supported names

```python
from odobridge.firmware_model import DriveConfig, DriveController, SimulatedRobot, wrap_angle
from odobridge.kinematics import Geometry, Twist
from odobridge.protocol import duty_to_wire
```

The same seven names are re-exported from the package root
(`from odobridge import DriveController, ...`). Each module lists its public
names in `__all__`:

| Module | Public names |
|---|---|
| `odobridge.firmware_model` | `DriveConfig`, `DriveController`, `SimulatedRobot`, `SimulatedFirmware`, `wrap_angle`, and the types they take or expose: `EncoderSnapshot`, `WheelStatus`, `Pose2D`, `PidConfig`, `VelocityConfig`, `CommandWatchdog`, `MotorModel` |
| `odobridge.kinematics` | `Geometry`, `Twist`, `Wheels`, `inverse`, `forward`, `saturate`, `MECANUM`, `DIFFERENTIAL`, `DRIVE_TYPES` |
| `odobridge.protocol` | `duty_to_wire`, the frame codec, the message classes and the protocol constants |

Everything else in `firmware_model` (`VelocityEstimator`, `VelocityPid`,
`se2_exp`, `compose`, `count_delta`, `get_param`, ...) is an implementation
detail and may change without notice. Changes to the supported names, or to the
numbers the model produces for the same inputs, are listed in
[CHANGELOG.md](../CHANGELOG.md).

Every function and class has a docstring with the full details
(`python -m pydoc odobridge.firmware_model`); the rest of this page is the
summary.

## Conventions

- **Wheel order.** Every per-wheel list or tuple is four values in the order
  front-left, front-right, rear-left, rear-right (FL, FR, RL, RR).
- **Signs.** A positive wheel count, speed, target or duty drives that wheel's
  side of the robot forward. Body twists follow REP-103: `vx` forward, `vy` to
  the left, `wz` counter-clockwise seen from above.
- **Units.** Metres, seconds, radians. Wheel speeds and targets are rad/s,
  body twists m/s, m/s, rad/s. Duties are signed fractions of full PWM
  (-1..1). Encoder counts are 4x quadrature counts.
- **Clock.** Times are microseconds on a 32-bit clock that wraps modulo
  2**32, like Arduino `micros()`. All time differences are taken wrap-safely,
  so a run may start just before the wrap.
- **Counts.** Encoder counts are carried as int32 and wrap modulo 2**32;
  increments are taken wrap-safely.
- **Angles.** `DriveController.pose.theta` and `SimulatedRobot.true_pose.theta`
  are not wrapped: they accumulate across turns. `wrap_angle` maps an angle to
  (-pi, pi] (both `pi` and `-pi` give `pi`); the ODOMETRY message carries the
  wrapped heading.

## The names

**`Geometry(wheel_radius=0.05, half_length=0.15, half_width=0.17, roller_angle_deg=45.0, drive="mecanum")`**
Frozen dataclass, the same fields as `odocore::Geometry`. `half_length` (lx)
and `half_width` (ly) are measured from the robot centre to the wheels.
`drive` is `"mecanum"` or `"differential"`. Raises `ValueError` for an unknown
drive, a non-positive or non-finite length, or a roller angle outside (0, 90).

**`Twist(vx=0.0, vy=0.0, wz=0.0)`**
Frozen dataclass: a body twist (m/s, m/s, rad/s), or a body displacement over
one period (m, m, rad). `scaled(f)` multiplies every component.

**`DriveConfig`**
Dataclass mirroring `odocore::DriveConfig`: `geometry`, `ticks_per_wheel_rev`
(default 38000), `max_wheel_speed` (rad/s, default 25; non-positive or
infinite disables saturation), `pid` (`PidConfig`), `velocity`
(`VelocityConfig`), `watchdog_timeout_ms` (default 500), `control_rate_hz`
(default 100) and `telemetry_divider` (default 5). `DriveController.step`
does not read `control_rate_hz`: it takes `dt` from the timestamps it is given.

**`DriveController(config=None)`**
One firmware control period at a time.

- `command(twist, now_us) -> bool`: inverse kinematics, uniform saturation to
  `max_wheel_speed`, and a watchdog feed. The new wheel targets apply from the
  next `step`. Returns `False` and changes nothing if any component is not
  finite. On a differential drive `vy` is ignored.
- `step(snapshots, now_us) -> list[float]`: takes four `EncoderSnapshot`s
  sampled at `now_us` and returns the four duties.
  - The first call only primes the estimators and returns `[0.0] * 4`.
  - Every later call estimates wheel speeds, integrates the pose from the
    count increments and updates `measured`, whether or not commands are
    arriving.
  - If the watchdog has expired (no command yet, or the last one is more than
    `watchdog_timeout_ms` old), the wheel controllers are reset, targets and
    duties are zeroed and `[0.0] * 4` is returned. Otherwise each wheel's
    controller runs: `u = kff*target + ks*sign(target) + kp*e + I (+ D)`,
    clamped to `output_limit` and rate-limited by `slew_rate`, with
    anti-windup.
- State after each step: `pose` (`Pose2D`, may be assigned), `measured`
  (`Twist`), `active` (the command after saturation), `scale` (in (0, 1]),
  `targets` (rad/s), `wheels` (four `WheelStatus`: `count`, `velocity`,
  `target`, `duty`, `illegal_transitions`) and `watchdog` (`expired`,
  `trips`).
- `config` is held by reference. After changing it, call `apply_config()`.

**`SimulatedRobot(geometry, ticks_per_wheel_rev, motors=None, substeps=10, start_us=0)`**
Four first-order motors (`MotorModel`: dead band, gain, time constant; the
defaults are made-up round numbers, not a measured motor) driving ideal
encoders with exact edge timestamps. No slip, backlash or noise.
`snapshots()` returns the four encoders now; `advance(duty, dt)` holds the
duties for `dt` seconds in `substeps` sub-steps, then moves `now_us` on by
`round(dt * 1e6)`. `true_pose` is the ground-truth pose from the real wheel
motion. The loop is:

```python
duty = controller.step(robot.snapshots(), robot.now_us)
robot.advance(duty, dt)
```

**`wrap_angle(angle) -> float`**
Wraps to (-pi, pi], as `odocore::wrap_angle`. NaN stays NaN; an infinite
angle raises `ValueError`.

**`duty_to_wire(duty) -> int`**
The WHEEL_STATE wire value: `duty * 10000`, clamped to [-10000, 10000],
rounded half away from zero. `1.0 -> 10000`, `-0.5 -> -5000`,
`0.00005 -> 1`, `2.0 -> 10000`; NaN and infinities give 0.

`SimulatedFirmware` combines a `DriveController` and a `SimulatedRobot` behind
the real wire protocol (`write` host bytes, `step` / `run`, `read` telemetry
frames); it is what `odobridge simulate` and `--port sim` use.

## Example

A mecanum robot driving forward, left and turning for three seconds, with a
command every 100 ms:

```python
from odobridge.firmware_model import DriveConfig, DriveController, SimulatedRobot, wrap_angle
from odobridge.kinematics import Geometry, Twist
from odobridge.protocol import duty_to_wire

config = DriveConfig(geometry=Geometry())  # mecanum, r = 50 mm, lx = 150 mm, ly = 170 mm
controller = DriveController(config)
robot = SimulatedRobot(config.geometry, config.ticks_per_wheel_rev)

dt = 1.0 / config.control_rate_hz  # 10 ms control period
for k in range(300):  # 3 s
    if k % 10 == 0:  # a command every 100 ms keeps the 500 ms watchdog fed
        controller.command(Twist(vx=0.2, vy=0.1, wz=0.5), robot.now_us)
    duty = controller.step(robot.snapshots(), robot.now_us)
    robot.advance(duty, dt)

p, v = controller.pose, controller.measured
print(f"t = {robot.now_us} us")
print(f"targets  (rad/s): {[round(w, 3) for w in controller.targets]}")
print(f"measured (rad/s): {[round(w.velocity, 3) for w in controller.wheels]}")
print(f"twist: vx={v.vx:.3f} m/s vy={v.vy:.3f} m/s wz={v.wz:.3f} rad/s")
print(f"pose:  x={p.x:.3f} m y={p.y:.3f} m theta={wrap_angle(p.theta):.3f} rad")
print(f"wire duties: {[duty_to_wire(d) for d in duty]}")
```

Output (odobridge 0.2.0, Python 3.14, Windows):

```text
t = 3000000 us
targets  (rad/s): [-1.2, 9.2, 2.8, 5.2]
measured (rad/s): [-1.19, 9.193, 2.811, 5.192]
twist: vx=0.200 m/s vy=0.100 m/s wz=0.499 rad/s
pose:  x=0.229 m y=0.536 m theta=1.421 rad
wire duties: [-1010, 3729, 1550, 2370]
```

The targets follow from the mecanum equations with t = tan 45 deg = 1 and
k = ly + lx = 0.32 m, e.g. FL = (0.2 - 0.1 - 0.32 * 0.5) / 0.05 = -1.2 rad/s.
`host/tests/test_public_api.py` runs this example and checks the output above.

## Fidelity

The model computes in double precision where the firmware uses float32, so
its numbers agree with the firmware's to float rounding, not bit-for-bit. The
model is deterministic: the same inputs give the same outputs on every run.
It has not been compared against a physical robot (see the README's hardware
validation status).
