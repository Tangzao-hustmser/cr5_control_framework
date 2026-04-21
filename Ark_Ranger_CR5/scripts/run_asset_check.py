import argparse
import os
import sys

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

from src.r3.asset_check import run_asset_consistency_check


def main():
    parser = argparse.ArgumentParser(description="Run Ark Ranger CR5 asset consistency checks.")
    parser.add_argument("--robot-config", default=os.path.join(root, "configs", "robot_config.yaml"))
    parser.add_argument("--kinematics-config", default=os.path.join(root, "configs", "kinematics_config.yaml"))
    parser.add_argument("--output", default=os.path.join(root, "reports", "asset_consistency_report.json"))
    args = parser.parse_args()

    report = run_asset_consistency_check(
        project_root=root,
        robot_config_path=args.robot_config,
        kinematics_config_path=args.kinematics_config,
        report_output_path=args.output,
    )
    print(f"Asset consistency status: {report['summary']['status']}")
    print(f"Critical: {report['summary']['critical']}  Warning: {report['summary']['warning']}")
    print(f"Report: {args.output}")
    if report["summary"]["critical"] > 0:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
