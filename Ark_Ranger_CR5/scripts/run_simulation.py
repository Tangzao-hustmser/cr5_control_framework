import argparse
import os
import sys
import traceback
from typing import Any
import yaml

from isaacsim import SimulationApp


root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

from src.r3.adapters import build_adapters
from src.r3.asset_check import run_asset_consistency_check
from src.r3.health import HealthIssue, run_startup_health_check, save_health_report


def _load_yaml(path: str):
    with open(path, "r", encoding="utf-8-sig") as fh:
        return yaml.safe_load(fh)


def _select_degraded_input_source(runtime_cfg: dict) -> str:
    compat_cfg = runtime_cfg.get("compat", {})
    preferred = str(compat_cfg.get("degraded_input_source", "")).strip().lower()
    if preferred in ("local_terminal", "tcp_json"):
        return preferred
    return "local_terminal"


def _apply_ros2_degraded_fallback(runtime_cfg: dict, health_report: Any) -> bool:
    compat_cfg = runtime_cfg.get("compat", {})
    if not bool(compat_cfg.get("enable_degraded_fallback", False)):
        return False

    runtime_mode = str(runtime_cfg.get("mode", "")).strip().lower()
    input_source = str(runtime_cfg.get("input_source", "ros2_action")).strip().lower()
    if runtime_mode == "ros2_only" or input_source == "ros2_only":
        return False

    if input_source not in ("ros2", "ros2_action"):
        return False

    if health_report.status != "BLOCKED":
        return False

    ros2_error_codes = {"R3-E1702"}
    error_issues = [issue for issue in health_report.issues if issue.level == "error"]
    non_ros2_errors = [issue for issue in error_issues if issue.code not in ros2_error_codes]
    if non_ros2_errors:
        return False

    degraded_to = _select_degraded_input_source(runtime_cfg)
    for issue in health_report.issues:
        if issue.level == "error" and issue.code in ros2_error_codes:
            issue.level = "warning"
            issue.message = f"{issue.message} (degraded)"

    health_report.issues.append(
        HealthIssue(
            "R3-W1703",
            "warning",
            "dependency",
            f"ROS2 unavailable; falling back to {degraded_to}.",
        )
    )
    health_report.status = "DEGRADED"
    if isinstance(health_report.details, dict):
        health_report.details["degraded_to"] = degraded_to

    runtime_cfg["input_source"] = degraded_to
    runtime_cfg.setdefault("compat", {})["enable_adapters"] = True
    runtime_cfg.setdefault("ros2", {})["enabled"] = False
    if degraded_to == "tcp_json":
        adapters_cfg = runtime_cfg.setdefault("adapters", {})
        adapters_cfg.setdefault(
            "tcp_json",
            {
                "host": "0.0.0.0",
                "port": 58000,
                "max_line_bytes": 1024 * 1024,
                "accept_timeout_s": 0.5,
            },
        )

    return True


def _run_preflight(project_root: str, robot_cfg_path: str, runtime_cfg_path: str):
    robot_cfg = _load_yaml(robot_cfg_path)
    runtime_root = _load_yaml(runtime_cfg_path)
    runtime_cfg = runtime_root.get("runtime", {})

    report_dir = os.path.join(project_root, "reports")
    os.makedirs(report_dir, exist_ok=True)

    asset_report_path = os.path.join(report_dir, "asset_consistency_report.json")
    asset_report = run_asset_consistency_check(
        project_root=project_root,
        robot_config_path=robot_cfg_path,
        kinematics_config_path=os.path.join(project_root, "configs", "kinematics_config.yaml"),
        report_output_path=asset_report_path,
    )

    health_report = run_startup_health_check(robot_cfg, runtime_cfg, asset_report=asset_report)
    degraded = _apply_ros2_degraded_fallback(runtime_cfg, health_report)
    health_report_path = os.path.join(report_dir, "startup_health_report.json")
    save_health_report(health_report, health_report_path)

    print(health_report.render_text())
    print(f"[Startup] Asset report: {asset_report_path}")
    print(f"[Startup] Health report: {health_report_path}")

    if health_report.status == "BLOCKED":
        raise RuntimeError("Startup blocked by health check. See reports for details.")

    if degraded:
        print("[Startup] ROS2 unavailable; explicit degraded fallback is enabled.")
        degraded_to = health_report.details.get("degraded_to")
        if degraded_to:
            print(f"[Startup] Input source switched to: {degraded_to}")

    return robot_cfg, runtime_cfg, health_report


def _resolve_status_destination(runtime_cfg: dict) -> str:
    status_cfg = runtime_cfg.get("status_output", {})
    destination = str(status_cfg.get("destination", "auto")).strip().lower()
    if destination in ("console", "file", "both"):
        return destination
    selected_input = str(runtime_cfg.get("input_source", "ros2_action")).strip().lower()
    if selected_input in ("local_terminal", "tcp_json"):
        return "file"
    return "console"


def _print_runtime_guide(project_root: str, runtime_cfg: dict) -> None:
    input_source = str(runtime_cfg.get("input_source", "ros2_action")).strip().lower()
    status_cfg = runtime_cfg.get("status_output", {})
    destination = _resolve_status_destination(runtime_cfg)
    live_status_rel = str(status_cfg.get("live_status_file", "reports/live_status/current_status.json"))
    live_status_abs = os.path.join(project_root, live_status_rel)

    if input_source in ("ros2_action", "ros2", "ros2_only"):
        print("[Simulation] ROS2 action server ready: /r3/command")
        print("[Simulation] Example goal (WSL):")
        print(
            "  ros2 action send_goal /r3/command r3_msgs/action/Command "
            "\"{command: {protocol_version: 'r3.v1', command_id: 'cmd-demo-001', timestamp_ms: 1710000000000, "
            "source: 'ros2_cli', command_type: 'pose', "
            "payload_json: '{\\\"frame\\\":\\\"world\\\",\\\"position_m\\\":[0.40,0.00,0.55],"
            "\\\"orientation_wxyz\\\":[1,0,0,0]}', metadata_json: '{}'}}\" --feedback"
        )
        print("[Simulation] System services: /r3/pause /r3/resume /r3/reset /r3/stop")
        print("  ros2 service call /r3/pause r3_msgs/srv/SystemControl \"{operation: 'pause', reason: 'manual'}\"")
        print("[Simulation] Health services: /r3/health /r3/asset_check")
        print("  ros2 service call /r3/health r3_msgs/srv/HealthCheck \"{include_details: true, request_id: ''}\"")
        print("  ros2 service call /r3/asset_check r3_msgs/srv/AssetCheck \"{include_details: true, request_id: ''}\"")
        print("[Simulation] Telemetry topics: /r3/events /r3/status /r3/validation")
        print("  ros2 topic echo /r3/status")
        print("[Simulation] JSON compatibility (optional): python scripts/run_json_bridge.py")
    elif input_source == "tcp_json":
        tcp_cfg = runtime_cfg.get("adapters", {}).get("tcp_json", {})
        host = tcp_cfg.get("host", "0.0.0.0")
        port = tcp_cfg.get("port", 58000)
        print(f"[Simulation] TCP JSON server ready: {host}:{port}")
        if str(host) in ("0.0.0.0", "127.0.0.1"):
            print("[Simulation] WSL should connect to the Windows host IP (try: ip route | awk '/default/ {print $3; exit}').")
        print("[Simulation] Example (WSL):")
        print(f"  nc {host} {port}")
        print('  {"command_type":"pose","payload":{"frame":"world","position_m":[0.40,0.00,0.55],"orientation_wxyz":[1,0,0,0]}}')
        print("[Simulation] ROS2 bridge (WSL):")
        print("  WIN_IP=$(ip route | awk '/default/ {print $3; exit}')")
        print(f"  /usr/bin/python3 scripts/run_ros2_tcp_bridge.py --host \"$WIN_IP\" --port {port}")
    else:
        print("[Simulation] Ready. Use unified protocol JSON commands (plus local arm shortcuts).")
        print("[Simulation] Quick commands (paste one line each):")
        print('  {"command_type":"pose","payload":{"frame":"world","position_m":[0.40,0.00,0.55],"orientation_wxyz":[1,0,0,0]}}')
        print('  {"command_type":"arm","payload":{"joint_positions_rad":[0.0,-0.5,1.0,0.0,0.5,0.0]}}')
        print("  arm 0 -30 45 0 0 0          # local terminal shortcut, degrees")
        print("  arm_rad 0 -0.52 0.79 0 0 0  # local terminal shortcut, radians")
        print('  {"command_type":"gripper","payload":{"mode":"open"}}')
        print('  {"command_type":"gripper","payload":{"mode":"close"}}')
        print('  {"command_type":"system","payload":{"operation":"pause"}}')
        print('  {"command_type":"system","payload":{"operation":"resume"}}')
        print('  {"command_type":"system","payload":{"operation":"stop"}}')

    print("[Simulation] Status output rate: configs/runtime_config.yaml -> runtime.status_output.interval_s")
    if destination in ("file", "both"):
        print(f"[Simulation] Live status file: {live_status_abs}")
        print("[Simulation] Open a second terminal to watch status:")
        print(f'  python scripts/watch_live_status.py --file "{live_status_abs}"')
    if destination == "file" and input_source in ("local_terminal", "tcp_json"):
        print("[Simulation] Console status lines are disabled; use watch_live_status.py in a second terminal.")
    print("[Simulation] After stop, run replay + benchmark (and optional run_ros2_e2e.py) with the log session path.")


def main():
    parser = argparse.ArgumentParser(description="Run Ark Ranger CR5 simulation with R3 safety pipeline.")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without GUI rendering.")
    parser.add_argument(
        "--quick-safe",
        action="store_true",
        help="Quick experiment: reduce RTX/material preload via extra args (best-effort).",
    )
    parser.add_argument(
        "--quick-safe-ui",
        action="store_true",
        help="Quick experiment: minimal UI experience without RTX/material library.",
    )
    parser.add_argument(
        "--config",
        default=os.path.join(root, "configs", "robot_config.yaml"),
        help="Path to robot config YAML.",
    )
    parser.add_argument(
        "--runtime-config",
        default=os.path.join(root, "configs", "runtime_config.yaml"),
        help="Path to runtime config YAML.",
    )
    args = parser.parse_args()

    robot_cfg, runtime_cfg, health_report = _run_preflight(root, args.config, args.runtime_config)

    sim_app_config = {"headless": bool(args.headless)}
    if args.quick_safe_ui:
        sim_app_config["experience"] = os.path.join(root, "configs", "isaacsim.exp.quick_safe_ui.kit")
    elif args.quick_safe:
        sim_app_config["experience"] = os.path.join(root, "configs", "isaacsim.exp.quick_safe.kit")
    if args.quick_safe or args.quick_safe_ui:
        sim_app_config.setdefault("extra_args", [])
        sim_app_config["extra_args"].extend(
            [
                "--/exts/omni.kit.material.library/cache_mdl=false",
                "--/exts/omni.kit.material.library/lib_paths=[]",
                "--/exts/omni.kit.material.library/usd_source_asset_list=[]",
                "--/persistent/rtx/modes/rt/enabled=false",
                "--/persistent/rtx/modes/pt/enabled=false",
            ]
        )
    if args.quick_safe_ui:
        sim_app_config["extra_args"].append("--/exts/omni.kit.mainwindow/startup/dockWindows=false")
    sim_app = SimulationApp(sim_app_config)
    node = None
    ros2_runtime = None

    try:
        from omni.isaac.core import World

        input_source = str(runtime_cfg.get("input_source", "ros2_action")).strip().lower()
        compat_cfg = runtime_cfg.get("compat", {})
        if bool(compat_cfg.get("enable_ark_patch", False)):
            os.environ["ARK_ENABLE_COMPAT"] = "1"

        ros2_cfg = runtime_cfg.get("ros2", {})
        ros2_inputs = ("ros2_action", "ros2", "ros2_only")
        ros2_enabled = bool(ros2_cfg.get("enabled", input_source in ros2_inputs))
        force_ros2 = bool(ros2_cfg.get("force_enable", False))
        if input_source not in ros2_inputs and not force_ros2:
            ros2_enabled = False

        if ros2_enabled:
            try:
                import rclpy  # noqa: F401
            except Exception as exc:
                raise RuntimeError(f"ROS2 is required but rclpy is unavailable: {exc}")

        compat_enabled = bool(compat_cfg.get("enable_adapters", False))
        if input_source in ("local_terminal", "tcp_json", "ros2") and not compat_enabled:
            raise RuntimeError(
                f"input_source '{input_source}' requires runtime.compat.enable_adapters=true"
            )

        from src.isaac_node import ArkIsaacSimNode

        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()

        adapters = build_adapters(runtime_cfg) if compat_enabled else []
        node = ArkIsaacSimNode(
            world,
            args.config,
            args.runtime_config,
            adapters=adapters,
            runtime_cfg_override=runtime_cfg,
        )
        node.logger.log_startup(health_report.to_dict())

        if ros2_enabled:
            from src.ros2.r3_runtime import R3Ros2Runtime

            ros2_runtime = R3Ros2Runtime(
                node,
                root,
                args.config,
                args.runtime_config,
                robot_cfg,
                runtime_cfg,
            )

        world.reset()
        world.step(render=not args.headless)
        _print_runtime_guide(root, runtime_cfg)
        node.initialize_physics()

        while sim_app.is_running() and not node.shutdown_requested:
            world.step(render=not args.headless)
            node.step_node()

    except KeyboardInterrupt:
        print("\n[Simulation] Interrupted by user.")
    except Exception as exc:
        print(f"\n[Fatal] Simulation failed: {exc}")
        traceback.print_exc()
        raise
    finally:
        if ros2_runtime is not None:
            ros2_runtime.shutdown()
        if node is not None:
            node.shutdown()
            print(f"\n[Simulation] Log session: {node.log_session_path}")
        sim_app.close()


if __name__ == "__main__":
    main()
