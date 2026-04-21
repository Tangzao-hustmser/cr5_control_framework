import argparse
import os
import sys
import traceback
import yaml

from isaacsim import SimulationApp


root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

import ark_patch  # noqa: F401
from src.r3.adapters import build_adapters
from src.r3.asset_check import run_asset_consistency_check
from src.r3.health import run_startup_health_check, save_health_report



def _load_yaml(path: str):
    with open(path, "r", encoding="utf-8-sig") as fh:
        return yaml.safe_load(fh)



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
    health_report_path = os.path.join(report_dir, "startup_health_report.json")
    save_health_report(health_report, health_report_path)

    print(health_report.render_text())
    print(f"[Startup] Asset report: {asset_report_path}")
    print(f"[Startup] Health report: {health_report_path}")

    if health_report.status == "BLOCKED":
        raise RuntimeError("Startup blocked by health check. See reports for details.")

    return robot_cfg, runtime_cfg, health_report



def _resolve_status_destination(runtime_cfg: dict) -> str:
    status_cfg = runtime_cfg.get("status_output", {})
    destination = str(status_cfg.get("destination", "auto")).strip().lower()
    if destination in ("console", "file", "both"):
        return destination
    selected_input = str(runtime_cfg.get("input_source", "local_terminal")).strip().lower()
    return "file" if selected_input == "local_terminal" else "console"



def _print_runtime_guide(project_root: str, runtime_cfg: dict) -> None:
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
    status_cfg = runtime_cfg.get("status_output", {})
    destination = _resolve_status_destination(runtime_cfg)
    live_status_rel = str(status_cfg.get("live_status_file", "reports/live_status/current_status.json"))
    live_status_abs = os.path.join(project_root, live_status_rel)
    if destination in ("file", "both"):
        print(f"[Simulation] Live status file: {live_status_abs}")
        print(f"[Simulation] Open a second terminal to watch status:")
        print(f'  python scripts/watch_live_status.py --file "{live_status_abs}"')
    if destination == "file":
        print("[Simulation] Console status lines are disabled to keep r3-json input clean.")
    print("[Simulation] After stop, run replay + benchmark with the printed log session path.")



def main():
    parser = argparse.ArgumentParser(description="Run Ark Ranger CR5 simulation with R3 safety pipeline.")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without GUI rendering.")
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

    sim_app = SimulationApp({"headless": bool(args.headless)})
    node = None

    try:
        from omni.isaac.core import World
        from src.isaac_node import ArkIsaacSimNode

        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()

        adapters = build_adapters(runtime_cfg)
        node = ArkIsaacSimNode(world, args.config, args.runtime_config, adapters=adapters)
        node.logger.log_startup(health_report.to_dict())

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
        if node is not None:
            node.shutdown()
            print(f"\n[Simulation] Log session: {node.log_session_path}")
        sim_app.close()


if __name__ == "__main__":
    main()
