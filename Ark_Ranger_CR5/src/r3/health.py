"""Startup health checks with explicit normal/degraded/blocked modes."""

from dataclasses import dataclass, asdict
from typing import Any, Dict, List, Optional
from datetime import datetime, timezone
import importlib.util
import json
import os


@dataclass
class HealthIssue:
    code: str
    level: str
    component: str
    message: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class HealthReport:
    status: str
    checked_at: str
    issues: List[HealthIssue]
    details: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status,
            "checked_at": self.checked_at,
            "issues": [issue.to_dict() for issue in self.issues],
            "details": self.details,
        }

    def render_text(self) -> str:
        lines = [
            "\n==================== Startup Health Report ====================",
            f"Status: {self.status}",
            f"Checked At (UTC): {self.checked_at}",
        ]
        if self.issues:
            lines.append("Issues:")
            for issue in self.issues:
                lines.append(f"- [{issue.level.upper()}] {issue.code} {issue.component}: {issue.message}")
        else:
            lines.append("Issues: none")
        lines.append("================================================================")
        return "\n".join(lines)



def _module_available(name: str) -> bool:
    return importlib.util.find_spec(name) is not None



def _has_config_keys(config: Dict[str, Any], path: str) -> bool:
    current: Any = config
    for token in path.split("."):
        if not isinstance(current, dict) or token not in current:
            return False
        current = current[token]
    return True



def run_startup_health_check(
    robot_cfg: Dict[str, Any],
    runtime_cfg: Dict[str, Any],
    asset_report: Optional[Dict[str, Any]] = None,
) -> HealthReport:
    issues: List[HealthIssue] = []

    rclpy_available = _module_available("rclpy")
    r3_msgs_available = _module_available("r3_msgs")

    required_paths = [
        "robot.name",
        "robot.prim_path",
        "robot.usd_path",
        "robot.urdf_path",
        "action_space.arm.joints",
        "action_space.gripper.joints",
    ]
    for path in required_paths:
        if not _has_config_keys(robot_cfg, path):
            issues.append(HealthIssue("R3-E1101", "error", "config", f"Missing config key: {path}"))

    runtime_mode = str(runtime_cfg.get("mode", "")).strip().lower()
    input_source = str(runtime_cfg.get("input_source", "ros2_action")).strip().lower()
    compat_cfg = runtime_cfg.get("compat", {})
    compat_enabled = bool(compat_cfg.get("enable_adapters", False))
    degraded_fallback_enabled = bool(compat_cfg.get("enable_degraded_fallback", False))

    if runtime_mode == "ros2_only" and input_source not in ("ros2", "ros2_action", "ros2_only"):
        issues.append(
            HealthIssue(
                "R3-E1701",
                "error",
                "config",
                "runtime.mode is ros2_only but input_source is not ROS2.",
            )
        )

    if input_source in ("ros2", "ros2_action", "ros2_only"):
        if not rclpy_available:
            issues.append(HealthIssue("R3-E1702", "error", "dependency", "ROS2 input selected but rclpy is unavailable."))
        if not r3_msgs_available:
            issues.append(HealthIssue("R3-E1702", "error", "dependency", "ROS2 r3_msgs package is unavailable."))
    elif input_source in ("local_terminal", "none", "disabled", "tcp_json"):
        if input_source in ("local_terminal", "tcp_json") and not compat_enabled:
            issues.append(
                HealthIssue(
                    "R3-E1701",
                    "error",
                    "adapter",
                    f"Input source '{input_source}' requires runtime.compat.enable_adapters=true.",
                )
            )
    else:
        issues.append(HealthIssue("R3-E1701", "error", "adapter", f"Unsupported input source: {input_source}"))

    require_lcm = bool(runtime_cfg.get("health", {}).get("require_lcm", False))
    require_arktypes = bool(runtime_cfg.get("health", {}).get("require_arktypes", False))

    if require_lcm and not _module_available("lcm"):
        issues.append(HealthIssue("R3-E1701", "error", "dependency", "LCM is required but unavailable."))

    if require_arktypes and not _module_available("arktypes"):
        issues.append(HealthIssue("R3-E1701", "error", "dependency", "arktypes is required but unavailable."))

    if asset_report is not None:
        critical_count = int(asset_report.get("summary", {}).get("critical", 0))
        warning_count = int(asset_report.get("summary", {}).get("warning", 0))
        if critical_count > 0:
            issues.append(
                HealthIssue(
                    "R3-E1801",
                    "error",
                    "asset_check",
                    f"Asset consistency has {critical_count} critical issue(s).",
                )
            )
        if warning_count > 0:
            issues.append(
                HealthIssue(
                    "R3-E1902",
                    "warning",
                    "asset_check",
                    f"Asset consistency has {warning_count} warning(s).",
                )
            )

    has_error = any(issue.level == "error" for issue in issues)
    has_warning = any(issue.level == "warning" for issue in issues)
    if has_error:
        status = "BLOCKED"
    elif has_warning:
        status = "DEGRADED"
    else:
        status = "NORMAL"

    checked_at = datetime.now(timezone.utc).isoformat()
    report = HealthReport(
        status=status,
        checked_at=checked_at,
        issues=issues,
        details={
            "mode": runtime_mode,
            "input_source": input_source,
            "compat": {
                "enable_adapters": compat_enabled,
                "enable_degraded_fallback": degraded_fallback_enabled,
            },
            "require_lcm": require_lcm,
            "require_arktypes": require_arktypes,
            "ros2": {
                "rclpy": rclpy_available,
                "r3_msgs": r3_msgs_available,
            },
        },
    )
    return report



def save_health_report(report: HealthReport, output_path: str) -> None:
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as fh:
        json.dump(report.to_dict(), fh, indent=2, ensure_ascii=False)
