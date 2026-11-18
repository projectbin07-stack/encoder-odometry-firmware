from __future__ import annotations

import csv
from pathlib import Path

import pytest

from odobridge.cli import main
from odobridge.protocol import VelocityCommand, encode_frame

from .conftest import VECTORS_PATH


def test_simulate_reports_pose_and_ground_truth(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["simulate", "--vx", "0.2", "--wz", "0.5", "--duration", "2"]) == 0
    out = capsys.readouterr().out
    assert "odometry (firmware model)" in out
    assert "ground truth (plant)" in out
    assert "watchdog trips: 0" in out


def test_simulate_with_config_and_csv(tmp_path: Path, data_dir: Path) -> None:
    out_csv = tmp_path / "odom.csv"
    code = main(
        [
            "simulate",
            "--config",
            str(data_dir / "diff-drive.toml"),
            "--vx",
            "0.2",
            "--duration",
            "1.5",
            "--csv",
            str(out_csv),
        ]
    )
    assert code == 0
    rows = list(csv.DictReader(out_csv.open(encoding="utf-8")))
    assert len(rows) > 10
    # Driving straight forward: x grows, y and theta stay ~0.
    assert float(rows[-1]["x"]) > 0.15
    assert abs(float(rows[-1]["y"])) < 1e-3


def test_drive_against_sim_port(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out_csv = tmp_path / "drive.csv"
    assert (
        main(
            [
                "drive",
                "--port",
                "sim",
                "--vy",
                "0.1",
                "--duration",
                "1",
                "--reset-pose",
                "--csv",
                str(out_csv),
            ]
        )
        == 0
    )
    out = capsys.readouterr().out
    assert "final odometry" in out
    assert "crc errors: 0" in out
    assert out_csv.read_text(encoding="utf-8").startswith("t_s,x,y,theta,vx,vy,wz")


def test_monitor_sim(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["monitor", "--port", "sim", "--duration", "0.3", "-v"]) == 0
    out = capsys.readouterr().out
    assert "wheel_state seq=" in out
    assert "odometry messages:" in out


def test_param_list_get_set(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["param", "list"]) == 0
    assert "max_wheel_speed" in capsys.readouterr().out
    assert main(["param", "get", "kp"]) == 0
    assert capsys.readouterr().out.startswith("kp = 0.02")
    assert main(["param", "set", "roller_angle_deg", "95"]) == 1
    assert "out of range" in capsys.readouterr().out
    assert main(["param", "set", "control_rate_hz", "50"]) == 1
    assert "read-only" in capsys.readouterr().out
    assert main(["param", "get", "bogus"]) == 2
    assert main(["param", "get"]) == 2
    assert main(["param", "set", "kp"]) == 2


def test_push_config(data_dir: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["push-config", str(data_dir / "generic-medium.toml"), "--port", "sim"]) == 0
    out = capsys.readouterr().out
    assert "ticks_per_wheel_rev" in out
    assert "38000 -> 38000 (ok)" in out


def test_decode(capsys: pytest.CaptureFixture[str]) -> None:
    frame = encode_frame(VelocityCommand(0.5, 0.0, 0.0), 3).hex()
    assert main(["decode", frame, "00ff"]) == 1
    out = capsys.readouterr().out
    assert "velocity_command seq=3 {'vx': 0.5" in out
    assert "error" in out


def test_vectors_check_and_stale(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["vectors", "--path", str(VECTORS_PATH)]) == 0
    stale = tmp_path / "v.json"
    stale.write_text("{}", encoding="utf-8")
    assert main(["vectors", "--path", str(stale)]) == 1
    assert main(["vectors", "--write", "--path", str(stale)]) == 0
    assert stale.read_text(encoding="utf-8") == VECTORS_PATH.read_text(encoding="utf-8")


def test_bad_config_is_reported(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    bad = tmp_path / "bad.toml"
    bad.write_text("wheel_radius = 1\n", encoding="utf-8")
    assert main(["simulate", "--config", str(bad)]) == 2
    assert "missing" in capsys.readouterr().err
