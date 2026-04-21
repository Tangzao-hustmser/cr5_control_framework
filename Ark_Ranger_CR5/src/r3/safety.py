"""Safety filter V1: pass / clamp / block decisions with reason codes."""

from dataclasses import dataclass, replace
from typing import Any, Dict, List, Optional

from .protocol import NormalizedCommand
from .validator import RuntimeModel


@dataclass
class RobotStateSnapshot:
    joint_positions_rad: List[float]
    ee_pose_wxyz: List[float]
    gripper_position_rad: float


@dataclass
class SafetyDecision:
    decision: str
    command: Optional[NormalizedCommand]
    reason_codes: List[str]
    details: Dict[str, Any]


class SafetyFilterV1:
    def __init__(
        self,
        runtime: RuntimeModel,
        max_pose_deviation_m: float = 0.40,
        max_joint_deviation_rad: float = 1.20,
    ) -> None:
        self.runtime = runtime
        self.max_pose_deviation_m = max_pose_deviation_m
        self.max_joint_deviation_rad = max_joint_deviation_rad

    def apply(self, command: NormalizedCommand, state: RobotStateSnapshot) -> SafetyDecision:
        reasons: List[str] = []
        details: Dict[str, Any] = {}

        if command.command_type == "pose" and command.pose_target_wxyz is not None:
            target = list(command.pose_target_wxyz)
            original = list(target)
            clamped = False
            workspace = self.runtime.workspace_limits_m
            for idx, axis in enumerate(("x", "y", "z")):
                lo, hi = workspace[axis]
                val = target[idx]
                if val < lo:
                    target[idx] = lo
                    clamped = True
                elif val > hi:
                    target[idx] = hi
                    clamped = True

            if clamped:
                max_dev = max(abs(target[i] - original[i]) for i in range(3))
                details["pose_original"] = original[:3]
                details["pose_clamped"] = target[:3]
                if max_dev > self.max_pose_deviation_m:
                    reasons.append("R3-E1401")
                    return SafetyDecision(
                        decision="block",
                        command=None,
                        reason_codes=reasons,
                        details=details,
                    )
                reasons.append("R3-E1402")
                return SafetyDecision(
                    decision="clamp",
                    command=replace(command, pose_target_wxyz=target),
                    reason_codes=reasons,
                    details=details,
                )
            return SafetyDecision("pass", command, reasons, details)

        if command.command_type == "arm" and command.arm_target_rad is not None:
            target = list(command.arm_target_rad)
            original = list(target)
            clamped = False
            for idx, value in enumerate(target):
                lo, hi = self.runtime.arm_joint_limits_rad[idx]
                if value < lo:
                    target[idx] = lo
                    clamped = True
                elif value > hi:
                    target[idx] = hi
                    clamped = True

            if clamped:
                max_dev = max(abs(target[i] - original[i]) for i in range(len(target)))
                details["arm_original"] = original
                details["arm_clamped"] = target
                if max_dev > self.max_joint_deviation_rad:
                    reasons.append("R3-E1401")
                    return SafetyDecision("block", None, reasons, details)
                reasons.append("R3-E1402")
                return SafetyDecision("clamp", replace(command, arm_target_rad=target), reasons, details)
            return SafetyDecision("pass", command, reasons, details)

        if command.command_type == "gripper":
            if command.gripper_mode == "position" and command.gripper_position_rad is not None:
                lo, hi = self.runtime.gripper_limits_rad
                original = float(command.gripper_position_rad)
                clamped_value = min(max(original, lo), hi)
                if clamped_value != original:
                    details["gripper_original"] = original
                    details["gripper_clamped"] = clamped_value
                    reasons.append("R3-E1402")
                    return SafetyDecision(
                        "clamp",
                        replace(command, gripper_position_rad=clamped_value),
                        reasons,
                        details,
                    )
            return SafetyDecision("pass", command, reasons, details)

        return SafetyDecision("pass", command, reasons, details)
