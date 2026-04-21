"""Benchmark scenarios and metric computation for post-run evaluation."""

from dataclasses import dataclass
from typing import Any, Dict, List
import json
import math
import os
import yaml


@dataclass
class BenchmarkMetrics:
    mean_error: float
    max_error: float
    overshoot: float
    settling_time_s: float
    ik_failure_rate: float
    safety_intercept_rate: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mean_error": self.mean_error,
            "max_error": self.max_error,
            "overshoot": self.overshoot,
            "settling_time_s": self.settling_time_s,
            "ik_failure_rate": self.ik_failure_rate,
            "safety_intercept_rate": self.safety_intercept_rate,
        }



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



def _vector_error(a: List[float], b: List[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))



def compute_metrics_from_logs(log_dir: str) -> BenchmarkMetrics:
    states = _read_jsonl(os.path.join(log_dir, "state.jsonl"))
    safety = _read_jsonl(os.path.join(log_dir, "safety.jsonl"))
    events = _read_jsonl(os.path.join(log_dir, "events.jsonl"))

    errors: List[float] = []
    overshoot = 0.0
    settling_time_s = 0.0

    first_time = None
    settle_time = None
    settle_threshold = 0.03

    for row in states:
        state = row.get("state", {})
        target = state.get("target_arm_rad")
        current = state.get("current_arm_rad")
        if isinstance(target, list) and isinstance(current, list) and len(target) == len(current) and len(target) > 0:
            err = _vector_error(target, current)
            errors.append(err)
            overshoot = max(overshoot, err)

            ts = float(row.get("monotonic_s", 0.0))
            if first_time is None:
                first_time = ts
            if err <= settle_threshold and settle_time is None:
                settle_time = ts

    if first_time is not None and settle_time is not None:
        settling_time_s = max(0.0, settle_time - first_time)

    mean_error = sum(errors) / len(errors) if errors else 0.0
    max_error = max(errors) if errors else 0.0

    ik_failures = 0
    for row in events:
        event = row.get("event", {})
        code = event.get("fault_code") if isinstance(event, dict) else None
        if code in ("R3-E1501", "R3-E1502"):
            ik_failures += 1
    ik_failure_rate = ik_failures / max(1, len(events))

    intercepts = 0
    for row in safety:
        if row.get("decision") in ("clamp", "block"):
            intercepts += 1
    safety_intercept_rate = intercepts / max(1, len(safety))

    return BenchmarkMetrics(
        mean_error=mean_error,
        max_error=max_error,
        overshoot=overshoot,
        settling_time_s=settling_time_s,
        ik_failure_rate=ik_failure_rate,
        safety_intercept_rate=safety_intercept_rate,
    )



def compare_with_baseline(metrics: BenchmarkMetrics, baseline_path: str) -> Dict[str, Any]:
    baseline: Dict[str, Any] = {}
    if os.path.exists(baseline_path):
        with open(baseline_path, "r", encoding="utf-8-sig") as fh:
            baseline = json.load(fh)

    current = metrics.to_dict()
    delta: Dict[str, Any] = {}
    for key, value in current.items():
        if key in baseline:
            delta[key] = value - float(baseline[key])
        else:
            delta[key] = None

    return {
        "baseline": baseline,
        "current": current,
        "delta": delta,
    }


def load_scenarios(scenarios_path: str) -> Dict[str, Any]:
    if not os.path.exists(scenarios_path):
        return {"scenarios": []}
    with open(scenarios_path, "r", encoding="utf-8-sig") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        return {"scenarios": []}
    scenarios = data.get("scenarios", [])
    if not isinstance(scenarios, list):
        scenarios = []
    return {"scenarios": scenarios}



def write_benchmark_report(report: Dict[str, Any], output_path: str) -> None:
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2, ensure_ascii=False)
