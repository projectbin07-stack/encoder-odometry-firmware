"""Host bridge for encoder-odometry-firmware: protocol, CLI and firmware model.

The names below are re-exported for convenience; they are the supported
firmware-model API described in ``docs/host-api.md``. Each module also
declares its own public names in ``__all__``.
"""

from .firmware_model import DriveConfig, DriveController, SimulatedRobot, wrap_angle
from .kinematics import Geometry, Twist
from .protocol import duty_to_wire

__version__ = "0.2.0"

__all__ = [
    "DriveConfig",
    "DriveController",
    "Geometry",
    "SimulatedRobot",
    "Twist",
    "__version__",
    "duty_to_wire",
    "wrap_angle",
]
