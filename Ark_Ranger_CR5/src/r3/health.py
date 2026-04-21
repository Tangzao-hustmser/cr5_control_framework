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

    input_source = runtime_cfg.get("input_source", "local_terminal")
    if input_source == "ros2":
        if not _module_available("rclpy"):
            issues.append(HealthIssue("R3-E1702", "error", "dependency", "ROS2 adapter selected but rclpy is unavailable."))
    elif input_source != "local_terminal":
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
            "input_source": input_source,
            "require_lcm": require_lcm,
            "require_arktypes": require_arktypes,
        },
    )
    return report



def save_health_report(report: HealthReport, output_path: str) -> None:
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as fh:
        json.dump(report.to_dict(), fh, indent=2, ensure_ascii=False)
