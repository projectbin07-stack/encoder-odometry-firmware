from __future__ import annotations

import math
from pathlib import Path

import pytest

from odobridge.kinematics import DIFFERENTIAL, MECANUM
from odobridge.robot_config import ConfigError, RobotConfig


def test_loads_mecanum_kinematics_file_unchanged(data_dir: Path) -> None:
    cfg = RobotConfig.from_toml(data_dir / "generic-medium.toml")
    assert cfg.name == "generic-medium"
    assert cfg.drive == MECANUM
    # 2000 counts per encoder rev x 19:1 gearbox.
    assert cfg.ticks_per_wheel_rev == 38000.0
    params = dict(cfg.firmware_params())
    assert params == {
        "wheel_radius": 0.05,
        "half_length": 0.15,
        "half_width": 0.17,
        "roller_angle_deg": 45.0,
        "ticks_per_wheel_rev": 38000.0,
        "drive_type": 0.0,
        "max_wheel_speed": 25.0,
    }


def test_differential_file(data_dir: Path) -> None:
    cfg = RobotConfig.from_toml(data_dir / "diff-drive.toml")
    assert cfg.drive == DIFFERENTIAL
    assert cfg.ticks_per_wheel_rev == pytest.approx(44 * 90.0)
    assert dict(cfg.firmware_params())["drive_type"] == 1.0
    assert cfg.geometry.drive == DIFFERENTIAL


def test_wheel_order_reorders_canonical_arrays() -> None:
    cfg = RobotConfig(0.05, 0.15, 0.17, wheel_order=("fr", "FL", "RR", "RL"))
    assert cfg.wheel_order == ("FR", "FL", "RR", "RL")
    assert cfg.to_config_order((1.0, 2.0, 3.0, 4.0)) == (2.0, 1.0, 4.0, 3.0)


def test_infinite_speed_limit_is_not_pushed() -> None:
    cfg = RobotConfig(0.05, 0.15, 0.17)
    assert math.isinf(cfg.max_wheel_speed)
    assert "max_wheel_speed" not in dict(cfg.firmware_params())


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("wheel_radius = 0.05\nhalf_length = 0.1\nhalf_width = 0.1\nwheel_raduis = 1\n", "unknown"),
        ("wheel_radius = 0.05\nhalf_length = 0.1\n", "missing"),
        ("wheel_radius = -1\nhalf_length = 0.1\nhalf_width = 0.1\n", "positive"),
        ("wheel_radius = 0.05\nhalf_length = 0.1\nhalf_width = 0.1\nroller_angle_deg = 90\n", "90"),
        ('wheel_radius = 0.05\nhalf_length = 0.1\nhalf_width = 0.1\nwheel_order = "FL"\n', "list"),
        (
            "wheel_radius = 0.05\nhalf_length = 0.1\nhalf_width = 0.1\n"
            'wheel_order = ["FL","FL","RL","RR"]\n',
            "permutation",
        ),
        ('wheel_radius = 0.05\nhalf_length = 0.1\nhalf_width = 0.1\ndrive = "omni"\n', "drive"),
        ("wheel_radius = = 1\n", "invalid TOML"),
    ],
)
def test_invalid_configs(tmp_path: Path, text: str, message: str) -> None:
    path = tmp_path / "robot.toml"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(ConfigError, match=message):
        RobotConfig.from_toml(path)
