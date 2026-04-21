"""ROS2 services for runtime control and runtime checks."""

from __future__ import annotations

from typing import Any, Dict
import json
import os
import time

from r3_msgs.srv import AssetCheck, HealthCheck, SystemControl

from src.r3.asset_check import run_asset_consistency_check
from src.r3.events import EVENT_TYPES
from src.r3.health import run_startup_health_check, save_health_report
from src.r3.protocol import generate_command_id


class R3Services:
    def __init__(
        self,
        node: Any,
        isaac_node: Any,
        project_root: str,
        robot_cfg_path: str,
        runtime_cfg_path: str,
        robot_cfg: Dict[str, Any],
        runtime_cfg: Dict[str, Any],
    ) -> None:
        self._node = node
        self._isaac_node = isaac_node
        self._project_root = project_root
        self._robot_cfg_path = robot_cfg_path
        self._runtime_cfg_path = runtime_cfg_path
        self._robot_cfg = robot_cfg
        self._runtime_cfg = runtime_cfg

        self._services = []
        self._services.append(self._node.create_service(SystemControl, "/r3/pause", self._handle_pause))
        self._services.append(self._node.create_service(SystemControl, "/r3/resume", self._handle_resume))
        self._services.append(self._node.create_service(SystemControl, "/r3/reset", self._handle_reset))
        self._services.append(self._node.create_service(SystemControl, "/r3/stop", self._handle_stop))
        self._services.append(self._node.create_service(HealthCheck, "/r3/health", self._handle_health))
        self._services.append(self._node.create_service(AssetCheck, "/r3/asset_check", self._handle_asset_check))

    def shutdown(self) -> None:
        for svc in self._services:
            try:
                self._node.destroy_service(svc)
            except Exception:
                pass
        self._services = []

    @staticmethod
    def _resolve_command_id(request_id: str) -> str:
        request_id = str(request_id or "").strip()
        return request_id if request_id else generate_command_id()

    @staticmethod
    def _trim_health_report(report: Dict[str, Any]) -> Dict[str, Any]:
        issues = report.get("issues", [])
        error_count = sum(1 for issue in issues if str(issue.get("level", "")).lower() == "error")
        warning_count = sum(1 for issue in issues if str(issue.get("level", "")).lower() == "warning")
        return {
            "status": report.get("status", "UNKNOWN"),
            "checked_at": report.get("checked_at", ""),
            "issue_counts": {"error": error_count, "warning": warning_count, "total": len(issues)},
        }

    @staticmethod
    def _trim_asset_report(report: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "summary": report.get("summary", {}),
            "facts": report.get("facts", {}),
        }

    def _emit_service_event(self, event_type: str, command_id: str, fault_code: str, reason: str, details: Dict[str, Any]) -> None:
        if not hasattr(self._isaac_node, "events"):
            return
        self._isaac_node.events.publish(
            event_type,
            command_id=command_id,
            fault_code=fault_code,
            reason=reason,
            details=details,
        )

    def _system_response(self, result: Any, response: SystemControl.Response) -> SystemControl.Response:
        response.ok = bool(result.accepted and result.success)
        response.message = result.message or result.stage
        response.timestamp_ms = int(time.time() * 1000)
        response.command_id = str(result.command_id)
        return response

    def _handle_pause(self, request: SystemControl.Request, response: SystemControl.Response) -> SystemControl.Response:
        result = self._isaac_node.handle_system_operation("pause", request.reason)
        return self._system_response(result, response)

    def _handle_resume(self, request: SystemControl.Request, response: SystemControl.Response) -> SystemControl.Response:
        result = self._isaac_node.handle_system_operation("resume", request.reason)
        return self._system_response(result, response)

    def _handle_reset(self, request: SystemControl.Request, response: SystemControl.Response) -> SystemControl.Response:
        result = self._isaac_node.handle_system_operation("reset", request.reason)
        return self._system_response(result, response)

    def _handle_stop(self, request: SystemControl.Request, response: SystemControl.Response) -> SystemControl.Response:
        result = self._isaac_node.handle_system_operation("stop", request.reason)
        return self._system_response(result, response)

    def _handle_health(self, request: HealthCheck.Request, response: HealthCheck.Response) -> HealthCheck.Response:
        command_id = self._resolve_command_id(request.request_id)
        timestamp_ms = int(time.time() * 1000)

        asset_report_path = os.path.join(self._project_root, "reports", "asset_consistency_report.json")
        asset_report = run_asset_consistency_check(
            project_root=self._project_root,
            robot_config_path=self._robot_cfg_path,
            kinematics_config_path=os.path.join(self._project_root, "configs", "kinematics_config.yaml"),
            report_output_path=asset_report_path,
        )
        report = run_startup_health_check(self._robot_cfg, self._runtime_cfg, asset_report=asset_report)
        health_report_path = os.path.join(self._project_root, "reports", "startup_health_report.json")
        save_health_report(report, health_report_path)

        report_dict = report.to_dict()
        response.success = report.status != "BLOCKED"
        response.status = report.status
        response.report_json = json.dumps(
            report_dict if request.include_details else self._trim_health_report(report_dict),
            ensure_ascii=False,
        )
        response.timestamp_ms = timestamp_ms
        response.command_id = command_id

        fault_code = ""
        if report.status == "BLOCKED":
            fault_code = "R3-E1901"
        elif report.status == "DEGRADED":
            fault_code = "R3-E1902"
        self._emit_service_event(
            EVENT_TYPES["HEALTH_CHECK"],
            command_id=command_id,
            fault_code=fault_code,
            reason=f"health_check:{report.status}",
            details={
                "status": report.status,
                "include_details": bool(request.include_details),
                "report_path": health_report_path,
            },
        )
        return response

    def _handle_asset_check(self, request: AssetCheck.Request, response: AssetCheck.Response) -> AssetCheck.Response:
        command_id = self._resolve_command_id(request.request_id)
        timestamp_ms = int(time.time() * 1000)
        asset_report_path = os.path.join(self._project_root, "reports", "asset_consistency_report.json")
        asset_report = run_asset_consistency_check(
            project_root=self._project_root,
            robot_config_path=self._robot_cfg_path,
            kinematics_config_path=os.path.join(self._project_root, "configs", "kinematics_config.yaml"),
            report_output_path=asset_report_path,
        )
        status = str(asset_report.get("summary", {}).get("status", "UNKNOWN"))
        critical = int(asset_report.get("summary", {}).get("critical", 0))
        warning = int(asset_report.get("summary", {}).get("warning", 0))

        response.success = critical == 0
        response.status = status
        response.report_json = json.dumps(
            asset_report if request.include_details else self._trim_asset_report(asset_report),
            ensure_ascii=False,
        )
        response.timestamp_ms = timestamp_ms
        response.command_id = command_id

        fault_code = "R3-E1801" if critical > 0 else ("R3-E1902" if warning > 0 else "")
        self._emit_service_event(
            EVENT_TYPES["ASSET_CHECK"],
            command_id=command_id,
            fault_code=fault_code,
            reason=f"asset_check:{status}",
            details={
                "status": status,
                "critical": critical,
                "warning": warning,
                "include_details": bool(request.include_details),
                "report_path": asset_report_path,
            },
        )
        return response
