"""Pure-Python model of the firmware control loop, for testing without hardware.

This mirrors ``lib/odocore`` class by class (velocity estimator, PI(D) with
feed-forward / anti-windup / slew limit, command watchdog, kinematics, SE(2)
odometry, the protocol node) and adds a simple plant: four first-order DC
motors with a static-friction dead zone, driving ideal quadrature encoders.

The model uses Python floats (double precision) where the firmware uses
float32, so results agree with the firmware's to float rounding, not
bit-for-bit. The plant is a deliberately simple model, not a measurement of
any real motor: it is good for exercising the protocol, the CLI and the
control logic, not for predicting how a real robot will behave.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

from . import params as param_table
from .kinematics import DIFFERENTIAL, MECANUM, Geometry, Twist, forward, inverse, saturate
from .protocol import (
    DIAG_HARDWARE_COUNTERS,
    DIAG_SATURATED,
    DIAG_WATCHDOG_EXPIRED,
    PARAM_OK,
    PARAM_OUT_OF_RANGE,
    PARAM_READ_ONLY,
    PARAM_UNKNOWN_ID,
    PROTOCOL_VERSION,
    Diagnostics,
    FrameDecoder,
    Message,
    Odometry,
    ParamGet,
    ParamSet,
    ParamValue,
    SetPose,
    VelocityCommand,
    WheelState,
    duty_to_wire,
    encode_frame,
)

_U32 = 0xFFFFFFFF


def count_delta(a: int, b: int) -> int:
    """Wrap-safe ``a - b`` for counts kept modulo 2**32 (as int32)."""
    d = (a - b) & _U32
    return d - (1 << 32) if d >= (1 << 31) else d


def elapsed_us(now: int, then: int) -> int:
    """Wrap-safe elapsed time on a 32-bit microsecond clock."""
    return (now - then) & _U32


def to_int32(value: int) -> int:
    return count_delta(value, 0)


@dataclass(frozen=True)
class EncoderSnapshot:
    count: int
    last_edge_us: int
    illegal_transitions: int = 0
    edge_time_exact: bool = True


# ------------------------------------------------------------- estimator
@dataclass
class VelocityConfig:
    low_cps: float = 200.0
    high_cps: float = 800.0
    zero_timeout_us: int = 250_000


class VelocityEstimator:
    """Fixed-period / period (1/T) / blended estimator; see ``velocity.h``."""

    def __init__(self, config: VelocityConfig | None = None) -> None:
        self.config = config or VelocityConfig()
        self.reset()

    def reset(self) -> None:
        self._primed = False
        self._have_ref = False
        self._prev_count = 0
        self._prev_us = 0
        self._edge_count = 0
        self._edge_us = 0
        self.fixed_cps = 0.0
        self.period_cps = 0.0
        self.blended_cps = 0.0
        self.weight = 0.0

    @staticmethod
    def weight_for(fixed_cps: float, config: VelocityConfig) -> float:
        speed = abs(fixed_cps)
        if config.high_cps <= config.low_cps:
            return 1.0 if speed >= config.low_cps else 0.0
        if speed <= config.low_cps:
            return 0.0
        if speed >= config.high_cps:
            return 1.0
        return (speed - config.low_cps) / (config.high_cps - config.low_cps)

    def update(self, s: EncoderSnapshot, now_us: int) -> float:
        if not self._primed:
            self._primed = True
            self._prev_count, self._prev_us = s.count, now_us
            self._have_ref = False
            self._edge_count, self._edge_us = s.count, s.last_edge_us
            return self.blended_cps
        dt = elapsed_us(now_us, self._prev_us)
        delta = count_delta(s.count, self._prev_count)
        if dt > 0:
            self.fixed_cps = delta * 1e6 / dt
        self._prev_count, self._prev_us = s.count, now_us

        if s.count != self._edge_count or s.last_edge_us != self._edge_us:
            dc = count_delta(s.count, self._edge_count)
            de = elapsed_us(s.last_edge_us, self._edge_us)
            if dc == 0:
                self.period_cps = 0.0
            elif self._have_ref and de > 0:
                self.period_cps = dc * 1e6 / de
            self._edge_count, self._edge_us = s.count, s.last_edge_us
            self._have_ref = True
        elif self._have_ref:
            since = elapsed_us(now_us, self._edge_us)
            if since >= self.config.zero_timeout_us:
                self.period_cps = 0.0
                self._have_ref = False
            elif since > 0:
                bound = 1e6 / since
                if abs(self.period_cps) > bound:
                    self.period_cps = math.copysign(bound, self.period_cps)

        self.weight = self.weight_for(self.fixed_cps, self.config)
        self.blended_cps = self.weight * self.fixed_cps + (1.0 - self.weight) * self.period_cps
        return self.blended_cps


# ------------------------------------------------------------------ PID
@dataclass
class PidConfig:
    kp: float = 0.02
    ki: float = 0.2
    kd: float = 0.0
    kff: float = 0.03
    ks: float = 0.0
    output_limit: float = 1.0
    slew_rate: float = 4.0
    d_cutoff_hz: float = 20.0


def slew_limit(previous: float, requested: float, max_rate: float, dt: float) -> float:
    if not (max_rate > 0.0 and dt > 0.0):
        return requested
    step = max_rate * dt
    return min(max(requested, previous - step), previous + step)


class VelocityPid:
    """Per-wheel velocity controller; see ``pid.h`` for the control law."""

    def __init__(self, config: PidConfig | None = None) -> None:
        self.config = config or PidConfig()
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.output = 0.0
        self.limited = False
        self._d_state = 0.0
        self._prev_measured = 0.0
        self._have_prev = False

    def feedforward(self, target: float) -> float:
        ff = self.config.kff * target
        if target > 1e-6:
            ff += self.config.ks
        elif target < -1e-6:
            ff -= self.config.ks
        return ff

    def update(self, target: float, measured: float, dt: float) -> float:
        if not (dt > 0.0) or not all(math.isfinite(v) for v in (dt, target, measured)):
            return self.output
        c = self.config
        limit = abs(c.output_limit)
        error = target - measured
        derivative = 0.0
        if c.kd != 0.0 and self._have_prev:
            raw = -(measured - self._prev_measured) / dt
            if c.d_cutoff_hz > 0.0:
                tau = 1.0 / (2.0 * math.pi * c.d_cutoff_hz)
                self._d_state += dt / (dt + tau) * (raw - self._d_state)
            else:
                self._d_state = raw
            derivative = c.kd * self._d_state
        self._prev_measured = measured
        self._have_prev = True

        candidate = self.integral + c.ki * error * dt
        unclipped = self.feedforward(target) + c.kp * error + candidate + derivative
        saturated = min(max(unclipped, -limit), limit)
        out = slew_limit(self.output, saturated, c.slew_rate, dt)
        self.limited = out != unclipped
        pushing = (unclipped > out and error > 0.0) or (unclipped < out and error < 0.0)
        if not (self.limited and pushing):
            self.integral = candidate
        self.integral = min(max(self.integral, -limit), limit)
        self.output = out
        return out


# ------------------------------------------------------------- watchdog
class CommandWatchdog:
    def __init__(self, timeout_us: int = 500_000) -> None:
        self.timeout_us = timeout_us
        self._last_feed = 0
        self.expired = True
        self.trips = 0

    def feed(self, now_us: int) -> None:
        self._last_feed = now_us
        self.expired = False

    def update(self, now_us: int) -> bool:
        if self.expired:
            return False
        if elapsed_us(now_us, self._last_feed) > self.timeout_us:
            self.expired = True
            self.trips += 1
            return False
        return True


# ------------------------------------------------------------- odometry
@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


def wrap_angle(angle: float) -> float:
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def se2_exp(dx: float, dy: float, dtheta: float) -> Pose2D:
    if abs(dtheta) < 1e-6:
        a = 1.0 - dtheta * dtheta / 6.0
        b = dtheta / 2.0 - dtheta**3 / 24.0
    else:
        a = math.sin(dtheta) / dtheta
        b = (1.0 - math.cos(dtheta)) / dtheta
    return Pose2D(a * dx - b * dy, b * dx + a * dy, dtheta)


def compose(p: Pose2D, q: Pose2D) -> Pose2D:
    c, s = math.cos(p.theta), math.sin(p.theta)
    return Pose2D(p.x + c * q.x - s * q.y, p.y + s * q.x + c * q.y, p.theta + q.theta)


# ----------------------------------------------------- drive controller
@dataclass
class DriveConfig:
    geometry: Geometry = field(default_factory=Geometry)
    ticks_per_wheel_rev: float = 2000.0 * 19.0
    max_wheel_speed: float = 25.0
    pid: PidConfig = field(default_factory=PidConfig)
    velocity: VelocityConfig = field(default_factory=VelocityConfig)
    watchdog_timeout_ms: int = 500
    control_rate_hz: int = 100
    telemetry_divider: int = 5

    @property
    def radians_per_tick(self) -> float:
        return 2.0 * math.pi / self.ticks_per_wheel_rev


@dataclass
class WheelStatus:
    count: int = 0
    velocity: float = 0.0
    target: float = 0.0
    duty: float = 0.0
    illegal_transitions: int = 0


class DriveController:
    """Mirror of ``odocore::DriveController``."""

    def __init__(self, config: DriveConfig | None = None) -> None:
        self.config = config or DriveConfig()
        self.estimators = [VelocityEstimator() for _ in range(4)]
        self.pids = [VelocityPid() for _ in range(4)]
        self.wheels = [WheelStatus() for _ in range(4)]
        self.watchdog = CommandWatchdog()
        self.pose = Pose2D()
        self.targets: list[float] = [0.0] * 4
        self.measured = Twist()
        self.active = Twist()
        self.scale = 1.0
        self._primed = False
        self._prev_us = 0
        self._prev_count = [0] * 4
        self.apply_config()

    def apply_config(self) -> None:
        for est in self.estimators:
            est.config = self.config.velocity
        for pid in self.pids:
            pid.config = self.config.pid
        self.watchdog.timeout_us = self.config.watchdog_timeout_ms * 1000

    def command(self, twist: Twist, now_us: int) -> bool:
        if not all(math.isfinite(v) for v in (twist.vx, twist.vy, twist.wz)):
            return False
        wheels, self.scale = saturate(
            inverse(self.config.geometry, twist), self.config.max_wheel_speed
        )
        self.targets = list(wheels)
        vy = 0.0 if self.config.geometry.drive == DIFFERENTIAL else twist.vy
        self.active = Twist(twist.vx, vy, twist.wz).scaled(self.scale)
        self.watchdog.feed(now_us)
        return True

    def _stop_all(self) -> list[float]:
        for i in range(4):
            self.pids[i].reset()
            self.targets[i] = 0.0
            self.wheels[i].target = 0.0
            self.wheels[i].duty = 0.0
        self.active = Twist()
        self.scale = 1.0
        return [0.0] * 4

    def step(self, snapshots: list[EncoderSnapshot], now_us: int) -> list[float]:
        rad_per_tick = self.config.radians_per_tick
        if not self._primed:
            for i, s in enumerate(snapshots):
                self.estimators[i].update(s, now_us)
                self._prev_count[i] = s.count
                self.wheels[i].count = s.count
                self.wheels[i].illegal_transitions = s.illegal_transitions
                self.wheels[i].duty = 0.0
            self._prev_us = now_us
            self._primed = True
            return [0.0] * 4
        dt = elapsed_us(now_us, self._prev_us) * 1e-6
        self._prev_us = now_us
        speeds = [0.0] * 4
        increments = [0.0] * 4
        for i, s in enumerate(snapshots):
            speeds[i] = self.estimators[i].update(s, now_us) * rad_per_tick
            increments[i] = count_delta(s.count, self._prev_count[i]) * rad_per_tick
            self._prev_count[i] = s.count
            self.wheels[i].count = s.count
            self.wheels[i].velocity = speeds[i]
            self.wheels[i].illegal_transitions = s.illegal_transitions
        d = forward(self.config.geometry, increments)
        self.pose = compose(self.pose, se2_exp(d.vx, d.vy, d.wz))
        self.measured = forward(self.config.geometry, speeds)
        if not self.watchdog.update(now_us):
            return self._stop_all()
        duty = [0.0] * 4
        for i in range(4):
            duty[i] = self.pids[i].update(self.targets[i], speeds[i], dt)
            self.wheels[i].target = self.targets[i]
            self.wheels[i].duty = duty[i]
        return duty


# ----------------------------------------------------------------- plant
@dataclass(frozen=True)
class MotorModel:
    """First-order DC motor + gearbox, wheel side.

    Steady-state wheel speed for duty ``u`` is ``sign(u) * (|u| - deadband) /
    duty_per_rad_s`` (zero inside the dead band); the speed approaches it with
    time constant ``tau``. The defaults are made-up round numbers.
    """

    duty_per_rad_s: float = 0.034
    deadband: float = 0.06
    tau: float = 0.08

    def steady_state(self, duty: float) -> float:
        magnitude = abs(duty) - self.deadband
        if magnitude <= 0.0:
            return 0.0
        return math.copysign(magnitude / self.duty_per_rad_s, duty)


class SimulatedRobot:
    """Four motors with ideal encoders, integrated in physics sub-steps.

    Encoder edges are timestamped exactly (by interpolation within a
    sub-step), like the firmware's interrupt decoder.
    """

    def __init__(
        self,
        geometry: Geometry,
        ticks_per_wheel_rev: float,
        motors: tuple[MotorModel, ...] | None = None,
        substeps: int = 10,
        start_us: int = 0,
    ) -> None:
        self.geometry = geometry
        self.rad_per_tick = 2.0 * math.pi / ticks_per_wheel_rev
        self.motors = motors or (MotorModel(),) * 4
        self.substeps = substeps
        self.now_us = start_us & _U32
        self.angle = [0.0] * 4
        self.speed = [0.0] * 4
        self.count = [0] * 4
        self.last_edge_us = [self.now_us] * 4
        self.true_pose = Pose2D()

    def snapshots(self) -> list[EncoderSnapshot]:
        return [
            EncoderSnapshot(to_int32(self.count[i]), self.last_edge_us[i], 0, True)
            for i in range(4)
        ]

    def advance(self, duty: list[float], dt: float) -> None:
        h = dt / self.substeps
        h_us = h * 1e6
        for k in range(self.substeps):
            t0 = k * h_us
            before = list(self.speed)
            for i in range(4):
                motor = self.motors[i]
                target = motor.steady_state(duty[i])
                alpha = 1.0 - math.exp(-h / motor.tau)
                self.speed[i] += alpha * (target - self.speed[i])
                old = self.angle[i]
                self.angle[i] += 0.5 * (before[i] + self.speed[i]) * h
                new_count = math.floor(self.angle[i] / self.rad_per_tick)
                if new_count != self.count[i]:
                    # Interpolate the time of the last boundary crossed.
                    if new_count > self.count[i]:
                        boundary = new_count * self.rad_per_tick
                    else:
                        boundary = (new_count + 1) * self.rad_per_tick
                    span = self.angle[i] - old
                    frac = (boundary - old) / span if span != 0.0 else 1.0
                    frac = min(max(frac, 0.0), 1.0)
                    self.last_edge_us[i] = (self.now_us + round(t0 + frac * h_us)) & _U32
                    self.count[i] = new_count
            mean = [0.5 * (before[i] + self.speed[i]) * h for i in range(4)]
            d = forward(self.geometry, mean)
            self.true_pose = compose(self.true_pose, se2_exp(d.vx, d.vy, d.wz))
        self.now_us = (self.now_us + round(dt * 1e6)) & _U32


# ----------------------------------------------------------- node mirror
_PARAM_FIELDS: dict[str, tuple[str, str]] = {
    "kp": ("pid", "kp"),
    "ki": ("pid", "ki"),
    "kd": ("pid", "kd"),
    "kff": ("pid", "kff"),
    "ks": ("pid", "ks"),
    "output_limit": ("pid", "output_limit"),
    "slew_rate": ("pid", "slew_rate"),
    "d_cutoff_hz": ("pid", "d_cutoff_hz"),
    "max_wheel_speed": ("", "max_wheel_speed"),
    "watchdog_ms": ("", "watchdog_timeout_ms"),
    "ticks_per_wheel_rev": ("", "ticks_per_wheel_rev"),
    "telemetry_divider": ("", "telemetry_divider"),
    "estimator_low_cps": ("velocity", "low_cps"),
    "estimator_high_cps": ("velocity", "high_cps"),
}


def get_param(config: DriveConfig, pid_: int) -> tuple[int, float]:
    info = param_table.BY_ID.get(pid_)
    if info is None:
        return PARAM_UNKNOWN_ID, 0.0
    g = config.geometry
    special: dict[str, float] = {
        "wheel_radius": g.wheel_radius,
        "half_length": g.half_length,
        "half_width": g.half_width,
        "roller_angle_deg": g.roller_angle_deg,
        "drive_type": 0.0 if g.drive == MECANUM else 1.0,
        "zero_timeout_ms": float(config.velocity.zero_timeout_us // 1000),
        "control_rate_hz": float(config.control_rate_hz),
        "protocol_version": float(PROTOCOL_VERSION),
    }
    if info.name in special:
        return PARAM_OK, special[info.name]
    group, attr = _PARAM_FIELDS[info.name]
    target = getattr(config, group) if group else config
    return PARAM_OK, float(getattr(target, attr))


def set_param(config: DriveConfig, pid_: int, value: float) -> int:
    info = param_table.BY_ID.get(pid_)
    if info is None:
        return PARAM_UNKNOWN_ID
    if info.read_only:
        return PARAM_READ_ONLY
    if not math.isfinite(value) or value < info.minimum or value > info.maximum:
        return PARAM_OUT_OF_RANGE
    v: float = float(round(value)) if info.integer else value
    g = config.geometry
    if info.name == "wheel_radius":
        config.geometry = replace(g, wheel_radius=v)
    elif info.name == "half_length":
        config.geometry = replace(g, half_length=v)
    elif info.name == "half_width":
        config.geometry = replace(g, half_width=v)
    elif info.name == "roller_angle_deg":
        config.geometry = replace(g, roller_angle_deg=v)
    elif info.name == "drive_type":
        config.geometry = replace(g, drive=MECANUM if v == 0 else DIFFERENTIAL)
    elif info.name == "zero_timeout_ms":
        config.velocity.zero_timeout_us = int(v) * 1000
    else:
        group, attr = _PARAM_FIELDS[info.name]
        target = getattr(config, group) if group else config
        current = getattr(target, attr)
        setattr(target, attr, int(v) if isinstance(current, int) else v)
    return PARAM_OK


class SimulatedFirmware:
    """The firmware node + controller + plant, speaking the real wire protocol.

    Bytes written with :meth:`write` are decoded exactly as the firmware
    would; :meth:`run` advances simulated time in control periods and queues
    telemetry frames for :meth:`read`.
    """

    def __init__(
        self,
        config: DriveConfig | None = None,
        motors: tuple[MotorModel, ...] | None = None,
        start_us: int = 0,
    ) -> None:
        self.controller = DriveController(config)
        cfg = self.controller.config
        self.robot = SimulatedRobot(
            cfg.geometry, cfg.ticks_per_wheel_rev, motors=motors, start_us=start_us
        )
        self.receiver = FrameDecoder()
        self._out = bytearray()
        self._tx_seq = 0
        self._period = 0
        self._duty = [0.0] * 4
        self.frames_sent = 0

    @property
    def now_us(self) -> int:
        return self.robot.now_us

    @property
    def period_s(self) -> float:
        return 1.0 / self.controller.config.control_rate_hz

    # host side of the "serial port"
    def write(self, data: bytes) -> None:
        for packet in self.receiver.feed(data):
            self.handle(packet.message)

    def read(self) -> bytes:
        data = bytes(self._out)
        self._out.clear()
        return data

    def send(self, message: Message) -> None:
        self._out += encode_frame(message, self._tx_seq)
        self._tx_seq = (self._tx_seq + 1) & 0xFF
        self.frames_sent += 1

    def handle(self, message: Message) -> None:
        c = self.controller
        if isinstance(message, VelocityCommand):
            c.command(Twist(message.vx, message.vy, message.wz), self.now_us)
        elif isinstance(message, SetPose):
            c.pose = Pose2D(message.x, message.y, message.theta)
            self.send(self.odometry_message())
        elif isinstance(message, (ParamGet, ParamSet)):
            status = PARAM_OK
            if isinstance(message, ParamSet):
                status = set_param(c.config, message.id, message.value)
                if status == PARAM_OK:
                    c.apply_config()
                    self.robot.geometry = c.config.geometry
            read_status, value = get_param(c.config, message.id)
            if status == PARAM_OK:
                status = read_status
            self.send(ParamValue(message.id, status, value))

    def odometry_message(self) -> Odometry:
        c = self.controller
        return Odometry(
            self.now_us,
            c.pose.x,
            c.pose.y,
            wrap_angle(c.pose.theta),
            c.measured.vx,
            c.measured.vy,
            c.measured.wz,
        )

    def wheel_state_message(self) -> WheelState:
        w = self.controller.wheels
        return WheelState(
            self.now_us,
            (w[0].count, w[1].count, w[2].count, w[3].count),
            (w[0].velocity, w[1].velocity, w[2].velocity, w[3].velocity),
            (w[0].target, w[1].target, w[2].target, w[3].target),
            (
                duty_to_wire(w[0].duty),
                duty_to_wire(w[1].duty),
                duty_to_wire(w[2].duty),
                duty_to_wire(w[3].duty),
            ),
        )

    def diagnostics_message(self) -> Diagnostics:
        c = self.controller
        flags = 0
        if c.watchdog.expired:
            flags |= DIAG_WATCHDOG_EXPIRED
        if c.scale < 1.0:
            flags |= DIAG_SATURATED
        flags &= ~DIAG_HARDWARE_COUNTERS  # the model times edges exactly
        return Diagnostics(
            t_us=self.now_us,
            illegal_transitions=(0, 0, 0, 0),
            loop_overruns=0,
            watchdog_trips=c.watchdog.trips,
            max_loop_us=0,
            rx_crc_errors=self.receiver.crc_errors,
            rx_frame_errors=self.receiver.frame_errors,
            flags=flags,
        )

    def step(self) -> None:
        """One control period: sample, control, telemetry, then plant."""
        c = self.controller
        self._duty = c.step(self.robot.snapshots(), self.now_us)
        divider = max(1, c.config.telemetry_divider)
        if self._period % divider == 0:
            self.send(self.wheel_state_message())
            self.send(self.odometry_message())
        if self._period % (divider * 10) == 0:
            self.send(self.diagnostics_message())
        self._period += 1
        self.robot.advance(self._duty, self.period_s)

    def run(self, seconds: float) -> None:
        """Advance simulated time by whole control periods."""
        for _ in range(max(0, round(seconds / self.period_s))):
            self.step()
