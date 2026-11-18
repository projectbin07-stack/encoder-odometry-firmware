"""Command-line interface: ``odobridge <command> ...``.

Every command that talks to firmware takes ``--port``; ``--port sim`` runs it
against the pure-Python firmware model instead of a serial port.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import IO

from . import __version__
from . import params as param_table
from .firmware_model import DriveConfig, SimulatedFirmware
from .link import Client, SerialTransport, SimTransport, Transport, describe_status
from .protocol import (
    PARAM_OK,
    Odometry,
    Packet,
    ProtocolError,
    VelocityCommand,
    decode_frame,
    message_to_dict,
)
from .robot_config import ConfigError, RobotConfig
from .vectors import build_vectors, find_repo_file, render

ODOM_FIELDS = ("t_s", "x", "y", "theta", "vx", "vy", "wz")


def _sim_config(config: RobotConfig | None) -> DriveConfig:
    cfg = DriveConfig()
    if config is not None:
        cfg.geometry = config.geometry
        cfg.ticks_per_wheel_rev = config.ticks_per_wheel_rev
        if math.isfinite(config.max_wheel_speed):
            cfg.max_wheel_speed = float(config.max_wheel_speed)
    return cfg


def _load_config(path: str | None) -> RobotConfig | None:
    return RobotConfig.from_toml(path) if path else None


def _open(args: argparse.Namespace, config: RobotConfig | None = None) -> Transport:
    if args.port == "sim":
        return SimTransport(SimulatedFirmware(_sim_config(config)))
    return SerialTransport(args.port, args.baud)


class OdomLogger:
    """Writes ODOMETRY messages to CSV and remembers the last one."""

    def __init__(self, stream: IO[str] | None) -> None:
        self.writer = csv.writer(stream) if stream is not None else None
        if self.writer is not None:
            self.writer.writerow(ODOM_FIELDS)
        self.last: Odometry | None = None
        self.rows = 0

    def __call__(self, packet: Packet) -> None:
        m = packet.message
        if not isinstance(m, Odometry):
            return
        self.last = m
        self.rows += 1
        if self.writer is not None:
            self.writer.writerow(
                [f"{m.t_us / 1e6:.6f}"]
                + [f"{v:.6f}" for v in (m.x, m.y, m.theta, m.vx, m.vy, m.wz)]
            )


def _format_odom(m: Odometry | None) -> str:
    if m is None:
        return "no odometry received"
    return (
        f"x={m.x:+.4f} m  y={m.y:+.4f} m  theta={m.theta:+.4f} rad  "
        f"vx={m.vx:+.3f} vy={m.vy:+.3f} wz={m.wz:+.3f}"
    )


def _open_csv(path: str | None) -> IO[str] | None:
    return Path(path).open("w", newline="", encoding="utf-8") if path else None


# ----------------------------------------------------------------- commands
def cmd_drive(args: argparse.Namespace) -> int:
    config = _load_config(args.config)
    transport = _open(args, config)
    stream = _open_csv(args.csv)
    try:
        client = Client(transport)
        if args.reset_pose:
            client.set_pose()
        logger = OdomLogger(stream)
        client.drive(
            VelocityCommand(args.vx, args.vy, args.wz), args.duration, args.rate, on_packet=logger
        )
        transport.wait(0.2)
        for packet in client.poll():
            logger(packet)
        print(f"final odometry: {_format_odom(logger.last)}")
        diag = client.telemetry.diagnostics
        if diag is not None:
            print(
                f"diagnostics: illegal={list(diag.illegal_transitions)} "
                f"overruns={diag.loop_overruns} watchdog_trips={diag.watchdog_trips} "
                f"rx_crc_errors={diag.rx_crc_errors} rx_frame_errors={diag.rx_frame_errors}"
            )
        print(
            f"frames received: {client.decoder.frames_ok}  "
            f"crc errors: {client.decoder.crc_errors}  "
            f"frame errors: {client.decoder.frame_errors}  "
            f"sequence gaps: {client.telemetry.seq_gaps}"
        )
    finally:
        if stream is not None:
            stream.close()
        transport.close()
    return 0


def cmd_monitor(args: argparse.Namespace) -> int:
    transport = _open(args)
    stream = _open_csv(args.csv)
    try:
        client = Client(transport)
        logger = OdomLogger(stream)
        start = transport.now()
        while transport.now() - start < args.duration:
            for packet in client.poll():
                logger(packet)
                if args.verbose:
                    print(
                        f"{packet.message.NAME} seq={packet.seq} {message_to_dict(packet.message)}"
                    )
            transport.wait(0.01)
        print(f"odometry messages: {logger.rows}")
        print(f"last: {_format_odom(logger.last)}")
    finally:
        if stream is not None:
            stream.close()
        transport.close()
    return 0


def cmd_param(args: argparse.Namespace) -> int:
    if args.action == "list":
        print(f"{'id':>3}  {'name':<20} {'range':<24} {'unit':<15} notes")
        for p in param_table.PARAMS:
            notes = ", ".join(
                n
                for n in (
                    "read-only" if p.read_only else "",
                    "integer" if p.integer else "",
                    p.help,
                )
                if n
            )
            rng = f"{p.minimum:g} .. {p.maximum:g}"
            print(f"{p.id:>3}  {p.name:<20} {rng:<24} {p.unit:<15} {notes}")
        return 0
    if args.name is None:
        print("error: a parameter name or id is required", file=sys.stderr)
        return 2
    try:
        info = param_table.lookup(args.name)
    except KeyError as exc:
        print(f"error: {exc.args[0]}", file=sys.stderr)
        return 2
    transport = _open(args)
    try:
        client = Client(transport)
        if args.action == "get":
            reply = client.get_param(info.id, timeout=args.timeout)
        else:
            if args.value is None:
                print("error: set needs a value", file=sys.stderr)
                return 2
            reply = client.set_param(info.id, args.value, timeout=args.timeout)
    except TimeoutError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        transport.close()
    print(f"{info.name} = {reply.value:g}  ({describe_status(reply.status)})")
    return 0 if reply.status == PARAM_OK else 1


def cmd_push_config(args: argparse.Namespace) -> int:
    config = RobotConfig.from_toml(args.config)
    transport = _open(args)
    failures = 0
    try:
        client = Client(transport)
        for name, value in config.firmware_params():
            info = param_table.lookup(name)
            reply = client.set_param(info.id, value, timeout=args.timeout)
            ok = reply.status == PARAM_OK
            failures += 0 if ok else 1
            print(f"{name:<20} {value:>12g} -> {reply.value:g} ({describe_status(reply.status)})")
    finally:
        transport.close()
    return 0 if failures == 0 else 1


def cmd_simulate(args: argparse.Namespace) -> int:
    config = _load_config(args.config)
    firmware = SimulatedFirmware(_sim_config(config))
    transport = SimTransport(firmware)
    stream = _open_csv(args.csv)
    try:
        client = Client(transport)
        logger = OdomLogger(stream)
        client.drive(
            VelocityCommand(args.vx, args.vy, args.wz), args.duration, args.rate, on_packet=logger
        )
        transport.wait(args.settle)
        for packet in client.poll():
            logger(packet)
    finally:
        if stream is not None:
            stream.close()
    truth = firmware.robot.true_pose
    c = firmware.controller
    name = config.name if config is not None else "default geometry"
    print(f"simulated {args.duration:g} s of ({args.vx:g}, {args.vy:g}, {args.wz:g}) on {name}")
    print(f"odometry (firmware model): {_format_odom(logger.last)}")
    print(
        "ground truth (plant):      "
        f"x={truth.x:+.4f} m  y={truth.y:+.4f} m  theta={truth.theta:+.4f} rad"
    )
    error = math.hypot(c.pose.x - truth.x, c.pose.y - truth.y)
    print(
        f"odometry position error:   {error * 1000:.2f} mm "
        "(encoder quantisation only; no slip modelled)"
    )
    print(
        f"watchdog trips: {c.watchdog.trips}  frames: {client.decoder.frames_ok}  "
        f"saturation scale: {c.scale:.3f}"
    )
    return 0


def cmd_decode(args: argparse.Namespace) -> int:
    status = 0
    for text in args.frames:
        try:
            packet = decode_frame(bytes.fromhex(text))
        except (ValueError, ProtocolError) as exc:
            print(f"{text}: error: {exc}")
            status = 1
            continue
        print(f"{packet.message.NAME} seq={packet.seq} {message_to_dict(packet.message)}")
    return status


def cmd_vectors(args: argparse.Namespace) -> int:
    text = render(build_vectors())
    path = Path(args.path) if args.path else find_repo_file()
    if args.write:
        path.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {path}")
        return 0
    current = path.read_text(encoding="utf-8")
    if current != text:
        print(f"{path} is stale; run `odobridge vectors --write`", file=sys.stderr)
        return 1
    print(f"{path} is up to date")
    return 0


# -------------------------------------------------------------------- parser
def _add_port(p: argparse.ArgumentParser) -> None:
    p.add_argument("--port", required=True, help="serial port (COM5, /dev/ttyUSB0) or 'sim'")
    p.add_argument("--baud", type=int, default=921600, help="baud rate (default 921600)")


def _add_twist(p: argparse.ArgumentParser) -> None:
    p.add_argument("--vx", type=float, default=0.0, help="forward speed, m/s")
    p.add_argument("--vy", type=float, default=0.0, help="leftward speed, m/s (mecanum only)")
    p.add_argument("--wz", type=float, default=0.0, help="yaw rate, rad/s (CCW positive)")
    p.add_argument("--duration", type=float, default=2.0, help="seconds (default 2)")
    p.add_argument("--rate", type=float, default=20.0, help="command rate, Hz (default 20)")
    p.add_argument("--csv", help="write odometry to this CSV file")
    p.add_argument("--config", help="robot TOML (mecanum-kinematics format)")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="odobridge", description="Host bridge for encoder-odometry-firmware."
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("drive", help="stream a velocity command and log odometry")
    _add_port(p)
    _add_twist(p)
    p.add_argument("--reset-pose", action="store_true", help="zero the odometry first")
    p.set_defaults(func=cmd_drive)

    p = sub.add_parser("monitor", help="log telemetry without commanding motion")
    _add_port(p)
    p.add_argument("--duration", type=float, default=5.0)
    p.add_argument("--csv", help="write odometry to this CSV file")
    p.add_argument("-v", "--verbose", action="store_true", help="print every message")
    p.set_defaults(func=cmd_monitor)

    p = sub.add_parser("param", help="list, read or write firmware parameters")
    p.add_argument("action", choices=("list", "get", "set"))
    p.add_argument("name", nargs="?", help="parameter name or id")
    p.add_argument("value", nargs="?", type=float)
    p.add_argument("--port", default="sim", help="serial port or 'sim' (default sim)")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--timeout", type=float, default=1.0)
    p.set_defaults(func=cmd_param)

    p = sub.add_parser("push-config", help="send a robot TOML's geometry to the firmware")
    p.add_argument("config", help="robot TOML (mecanum-kinematics format)")
    _add_port(p)
    p.add_argument("--timeout", type=float, default=1.0)
    p.set_defaults(func=cmd_push_config)

    p = sub.add_parser("simulate", help="run a command against the Python firmware model")
    _add_twist(p)
    p.add_argument("--settle", type=float, default=0.5, help="seconds to run after stopping")
    p.set_defaults(func=cmd_simulate)

    p = sub.add_parser("decode", help="decode hex-encoded frames")
    p.add_argument("frames", nargs="+", help="frame bytes as hex, e.g. 0d0101...00")
    p.set_defaults(func=cmd_decode)

    p = sub.add_parser("vectors", help="check or regenerate the shared protocol vectors")
    p.add_argument("--write", action="store_true", help="rewrite the file")
    p.add_argument("--path", help="vectors file (default: found from the current directory)")
    p.set_defaults(func=cmd_vectors)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result: int = args.func(args)
    except ConfigError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return result


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
