from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import numpy as np


@dataclass
class RobotObservation:
    joint_positions: np.ndarray
    ee_pose: np.ndarray          # [x, y, z, qw, qx, qy, qz]
    odom: np.ndarray             # [x, y, theta]
    gripper_position: float = 0.0
    image: Optional[np.ndarray] = None
    timestamp_ms: int = 0


@dataclass
class RobotAction:
    command_id: str = ""
    mode: str = "arm"           # arm / pose / gripper / system
    target: np.ndarray = field(default_factory=lambda: np.zeros(6))
    gripper_mode: str = "hold"
    gripper: float = 0.0
    system_operation: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)
