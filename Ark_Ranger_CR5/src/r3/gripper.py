"""Closed-loop gripper control module (open/close/hold/position) with stall detection."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class GripperStatus:
    mode: str
    target_rad: float
    feedback_rad: float
    stalled: bool
    reason_code: str


class GripperController:
    def __init__(
        self,
        lower_rad: float,
        upper_rad: float,
        stall_error_threshold: float = 0.03,
        movement_threshold: float = 1e-4,
        stall_steps: int = 45,
    ) -> None:
        self.lower_rad = float(lower_rad)
        self.upper_rad = float(upper_rad)
        self.mode = "hold"
        self.target_rad = self.lower_rad
        self._hold_target = self.lower_rad
        self._last_feedback: Optional[float] = None
        self._stall_counter = 0
        self._stall_error_threshold = stall_error_threshold
        self._movement_threshold = movement_threshold
        self._stall_steps = stall_steps

    def set_command(self, mode: str, position_rad: Optional[float], feedback_rad: float) -> None:
        if mode == "open":
            self.mode = mode
            self.target_rad = self.lower_rad
            self._hold_target = self.target_rad
        elif mode == "close":
            self.mode = mode
            self.target_rad = self.upper_rad
            self._hold_target = self.target_rad
        elif mode == "hold":
            self.mode = mode
            self._hold_target = float(feedback_rad)
            self.target_rad = self._hold_target
        elif mode == "position":
            self.mode = mode
            if position_rad is None:
                self.target_rad = self._hold_target
            else:
                self.target_rad = float(position_rad)
                self._hold_target = self.target_rad

        self.target_rad = min(max(self.target_rad, self.lower_rad), self.upper_rad)

    def step(self, feedback_rad: float) -> GripperStatus:
        desired = self.target_rad
        desired = min(max(desired, self.lower_rad), self.upper_rad)

        stalled = False
        reason = ""
        if self._last_feedback is None:
            self._last_feedback = float(feedback_rad)

        error = abs(desired - feedback_rad)
        moved = abs(float(feedback_rad) - float(self._last_feedback))

        if error > self._stall_error_threshold and moved < self._movement_threshold:
            self._stall_counter += 1
        else:
            self._stall_counter = 0

        if self._stall_counter >= self._stall_steps:
            stalled = True
            reason = "R3-E1601"

        self._last_feedback = float(feedback_rad)
        return GripperStatus(
            mode=self.mode,
            target_rad=desired,
            feedback_rad=float(feedback_rad),
            stalled=stalled,
            reason_code=reason,
        )
