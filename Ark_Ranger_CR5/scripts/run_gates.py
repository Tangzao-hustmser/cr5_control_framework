import argparse
import os
import subprocess
import sys

import yaml


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def run(cmd):
    print(f"[Gate] Run: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=ROOT)
    if result.returncode != 0:
        raise SystemExit(result.returncode)


def _check_ros2_layout():
    pkg_root = os.path.join(ROOT, "ros2_ws", "src", "r3_msgs")
    required = [
        os.path.join(pkg_root, "package.xml"),
        os.path.join(pkg_root, "CMakeLists.txt"),
        os.path.join(pkg_root, "msg", "Command.msg"),
        os.path.join(pkg_root, "msg", "CommandResult.msg"),
        os.path.join(pkg_root, "msg", "RuntimeStatus.msg"),
        os.path.join(pkg_root, "msg", "SafetyDecision.msg"),
        os.path.join(pkg_root, "msg", "ValidationIssue.msg"),
        os.path.join(pkg_root, "msg", "SystemEvent.msg"),
        os.path.join(pkg_root, "srv", "SystemControl.srv"),
        os.path.join(pkg_root, "srv", "HealthCheck.srv"),
        os.path.join(pkg_root, "srv", "AssetCheck.srv"),
        os.path.join(pkg_root, "action", "Command.action"),
    ]
    missing = [path for path in required if not os.path.exists(path)]
    if missing:
        raise SystemExit(f"[Gate] Missing ROS2 interface files: {missing}")

    runtime_cfg_path = os.path.join(ROOT, "configs", "runtime_config.yaml")
    with open(runtime_cfg_path, "r", encoding="utf-8-sig") as fh:
        cfg = yaml.safe_load(fh) or {}
    runtime_cfg = cfg.get("runtime", {}) if isinstance(cfg, dict) else {}
    mode = str(runtime_cfg.get("mode", "")).strip().lower()
    input_source = str(runtime_cfg.get("input_source", "")).strip().lower()
    if mode != "ros2_only" or input_source not in ("ros2_action", "ros2", "ros2_only"):
        raise SystemExit("[Gate] runtime_config.yaml must default to ROS2-only mode with ROS2 input source.")
    compat_cfg = runtime_cfg.get("compat", {}) if isinstance(runtime_cfg, dict) else {}
    if bool(compat_cfg.get("enable_degraded_fallback", False)):
        raise SystemExit("[Gate] runtime.compat.enable_degraded_fallback must default to false.")


def main():
    parser = argparse.ArgumentParser(description="Run R3 gate checks.")
    parser.add_argument("--ros2-only", action="store_true", help="Include ROS2-only sanity checks")
    args = parser.parse_args()

    run([sys.executable, "scripts/run_asset_check.py"])
    run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py", "-v"])

    if args.ros2_only:
        _check_ros2_layout()
        print("[Gate] ROS2 interface package present")

    print("[Gate] PASS")


if __name__ == "__main__":
    main()
