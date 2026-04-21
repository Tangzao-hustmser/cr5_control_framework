import argparse
import json
import os
import sys
from datetime import datetime, timezone
from typing import Any, Dict, List, Set

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

from src.r3.benchmark import compare_with_baseline, compute_metrics_from_logs, load_scenarios
from src.r3.replay import ReplayEngine


def _read_jsonl(path: str) -> List[Dict[str, Any]]:
    if not os.path.exists(path):
        return []
    rows: List[Dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def _extract_event_payload(row: Dict[str, Any]) -> Dict[str, Any]:
    event = row.get("event")
    if isinstance(event, dict):
        return event
    return {}


def _build_alignment_report(log_dir: str) -> Dict[str, Any]:
    timeline_rows = _read_jsonl(os.path.join(log_dir, "timeline_index.jsonl"))
    event_rows = _read_jsonl(os.path.join(log_dir, "events.jsonl"))

    timeline_event_map: Dict[str, Dict[str, Any]] = {}
    timeline_command_ids: Set[str] = set()
    for row in timeline_rows:
        if str(row.get("kind")) != "event_index":
            continue
        ref_id = str(row.get("ref_id", ""))
        payload = row.get("payload", {}) if isinstance(row.get("payload"), dict) else {}
        if ref_id:
            timeline_event_map[ref_id] = payload
        command_id = str(payload.get("command_id", ""))
        if command_id:
            timeline_command_ids.add(command_id)

    logged_event_ids: List[str] = []
    logged_command_ids: Set[str] = set()
    missing_in_timeline = 0
    missing_event_id = 0
    for row in event_rows:
        event = _extract_event_payload(row)
        event_id = str(event.get("event_id", ""))
        command_id = str(event.get("command_id", ""))
        if command_id:
            logged_command_ids.add(command_id)
        if not event_id:
            missing_event_id += 1
            continue
        logged_event_ids.append(event_id)
        if event_id not in timeline_event_map:
            missing_in_timeline += 1

    command_id_overlap = len(logged_command_ids & timeline_command_ids)
    command_ids_aligned = bool(logged_command_ids) and command_id_overlap == len(logged_command_ids)
    event_ids_aligned = bool(logged_event_ids) and missing_in_timeline == 0 and missing_event_id == 0

    return {
        "timeline_event_count": len(timeline_event_map),
        "logged_event_count": len(event_rows),
        "logged_event_ids": len(logged_event_ids),
        "missing_event_id": missing_event_id,
        "missing_in_timeline": missing_in_timeline,
        "command_id_overlap": command_id_overlap,
        "command_ids_aligned": command_ids_aligned,
        "event_ids_aligned": event_ids_aligned,
    }


def _default_output_path(log_dir: str) -> str:
    reports_dir = os.path.join(root, "reports")
    os.makedirs(reports_dir, exist_ok=True)
    session_name = os.path.basename(os.path.normpath(log_dir)) or "session"
    return os.path.join(reports_dir, f"ros2_e2e_report_{session_name}.json")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run ROS2-only E2E checks from recorded logs.")
    parser.add_argument("--log-dir", required=True, help="Session log directory (reports/logs/<session_id>)")
    parser.add_argument("--baseline", default=os.path.join(root, "configs", "benchmark_baseline.json"))
    parser.add_argument("--scenarios", default=os.path.join(root, "configs", "benchmark_scenarios.yaml"))
    parser.add_argument("--output", default="", help="Output report path (default: reports/ros2_e2e_report_<session>.json)")
    parser.add_argument("--strict", action="store_true", help="Return non-zero if alignment checks fail")
    args = parser.parse_args()

    if not os.path.isdir(args.log_dir):
        raise SystemExit(f"[E2E] log dir not found: {args.log_dir}")

    replay_engine = ReplayEngine(args.log_dir)
    replay_result = replay_engine.replay(headless=True, speed=0.0)

    metrics = compute_metrics_from_logs(args.log_dir)
    benchmark_report = compare_with_baseline(metrics, args.baseline)
    benchmark_report["scenarios"] = load_scenarios(args.scenarios).get("scenarios", [])
    benchmark_report["ros2_only"] = True

    alignment = _build_alignment_report(args.log_dir)
    checks = {
        "replay_has_commands": replay_result.replayed_commands > 0,
        "timeline_event_alignment": bool(alignment.get("event_ids_aligned", False)),
        "timeline_command_alignment": bool(alignment.get("command_ids_aligned", False)),
    }
    passed = all(checks.values())

    report = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "log_dir": os.path.abspath(args.log_dir),
        "ros2_only": True,
        "checks": checks,
        "passed": passed,
        "alignment": alignment,
        "replay": replay_result.to_dict(),
        "benchmark": benchmark_report,
    }

    output_path = args.output or _default_output_path(args.log_dir)
    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2, ensure_ascii=False)

    print("[E2E] ROS2-only report generated")
    print(f"[E2E] Output: {output_path}")
    print(f"[E2E] Passed: {passed}")
    if args.strict and not passed:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
