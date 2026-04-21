"""IK failure fallback strategy state machine."""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class IKPolicyDecision:
    state: str
    target_joint_rad: List[float]
    reason_code: str
    retry_count: int


class IKFailurePolicy:
    """Predictable IK fallback: hold -> retry -> fallback_last_valid."""

    def __init__(self, max_retries: int = 3) -> None:
        self.max_retries = max_retries
        self.retry_count = 0
        self.state = "NORMAL"
        self.last_valid_target: Optional[List[float]] = None

    def on_success(self, target_joint_rad: List[float]) -> IKPolicyDecision:
        self.retry_count = 0
        self.state = "NORMAL"
        self.last_valid_target = list(target_joint_rad)
        return IKPolicyDecision(
            state=self.state,
            target_joint_rad=list(target_joint_rad),
            reason_code="",
            retry_count=self.retry_count,
        )

    def on_failure(self, current_joint_rad: List[float]) -> IKPolicyDecision:
        self.retry_count += 1
        if self.retry_count <= self.max_retries:
            self.state = "RETRY_HOLD"
            return IKPolicyDecision(
                state=self.state,
                target_joint_rad=list(current_joint_rad),
                reason_code="R3-E1501",
                retry_count=self.retry_count,
            )

        self.state = "FALLBACK_LAST_VALID"
        fallback = list(self.last_valid_target) if self.last_valid_target is not None else list(current_joint_rad)
        return IKPolicyDecision(
            state=self.state,
            target_joint_rad=fallback,
            reason_code="R3-E1502",
            retry_count=self.retry_count,
        )
