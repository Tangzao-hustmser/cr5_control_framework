import argparse
import os
import sys

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

from src.r3.benchmark import compare_with_baseline, compute_metrics_from_logs, load_scenarios, write_benchmark_report


def main():
    parser = argparse.ArgumentParser(description="Compute benchmark metrics from R3 logs.")
    parser.add_argument("--log-dir", required=True)
    parser.add_argument("--baseline", default=os.path.join(root, "configs", "benchmark_baseline.json"))
    parser.add_argument("--scenarios", default=os.path.join(root, "configs", "benchmark_scenarios.yaml"))
    parser.add_argument("--output", default=os.path.join(root, "reports", "benchmark_report.json"))
    parser.add_argument("--ros2-only", action="store_true", help="Mark report as ROS2-only pipeline")
    args = parser.parse_args()

    metrics = compute_metrics_from_logs(args.log_dir)
    report = compare_with_baseline(metrics, args.baseline)
    report["scenarios"] = load_scenarios(args.scenarios).get("scenarios", [])
    if args.ros2_only:
        report["ros2_only"] = True
    write_benchmark_report(report, args.output)

    print("Benchmark report generated")
    print(f"Output: {args.output}")
    print(report)


if __name__ == "__main__":
    main()

