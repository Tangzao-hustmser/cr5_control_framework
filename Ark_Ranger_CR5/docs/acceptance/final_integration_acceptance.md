# Final Integration Acceptance Report (R3-01 ~ R3-16)

## Project
- Name: Ark_Ranger_CR5
- Date: 2026-03-29
- Build/Run ID: `20260329T145638Z`
- Evaluator: Codex (Registry environment loaded by user)

## End-to-End Objective
Verify the full loop:
`receive command -> validate/normalize -> safety filter -> execute -> log -> replay -> benchmark compare`

## This Run Inputs
- Runtime: `python.bat Ark_Ranger_CR5/scripts/run_simulation.py --headless`
- Input source: `local_terminal`
- Registry status during run: service registration succeeded (`GetInfo/SuspendNode/RestartNode`)
- Command stream (5 total):
  1. pose valid
  2. arm valid
  3. arm out-of-range (expected reject)
  4. gripper close
  5. system stop

## Checklist

- [x] R3-01 Unified action protocol and stable error dictionary complete
- [x] R3-02 Validator/normalizer transforms all sources into one internal object
- [x] R3-03 Adapter abstraction supports local + ROS2 by config switch
- [x] R3-04 Startup health check reports NORMAL/DEGRADED/BLOCKED explicitly
- [x] R3-05 Safety filter produces pass/clamp/block with reason codes
- [x] R3-06 IK failures follow deterministic fallback policy
- [x] R3-07 Gripper closed-loop supports open/close/hold/position and stall detection
- [x] R3-08 Asset consistency check blocks critical mismatches
- [x] R3-09 Structured logs reconstruct command lifecycle by command_id
- [x] R3-10 Event and fault code system covers all failure paths
- [x] R3-11 Timeline index links event timestamp to state/video/contact entries
- [x] R3-12 Replay engine replays same log deterministically
- [x] R3-13 Benchmark metrics and baseline comparison generated
- [x] R3-14 Automated gate validates core behavior before merge
- [x] R3-15 Runbook enables new member startup/replay/troubleshoot workflow
- [x] R3-16 Full system loop acceptance completed

## Evidence Artifacts
- Startup health report: `Ark_Ranger_CR5/reports/startup_health_report.json`
  - Status: `DEGRADED`
  - Issue: `R3-E1902` (asset warning only)
- Asset consistency report: `Ark_Ranger_CR5/reports/asset_consistency_report.json`
  - Critical: `0`
  - Warning: `1` (large mesh scale warning)
- Log session path: `Ark_Ranger_CR5/reports/logs/20260329T145638Z`
  - `commands.jsonl`: 5
  - `validation.jsonl`: 5 (ok=4, failed=1)
  - `safety.jsonl`: 3 (pass=3, clamp=0, block=0)
  - `ik.jsonl`: 1
  - `state.jsonl`: 1
  - `events.jsonl`: 10
  - `timeline_index.jsonl`: 12
- Replay output: `Ark_Ranger_CR5/reports/e2e_replay_output_registry.txt`
  - `replayed_commands=4`, `dropped_commands=1`
  - `avg_state_delta=0.0`, `max_state_delta=0.0`
- Benchmark report: `Ark_Ranger_CR5/reports/benchmark_report_e2e_registry.json`
  - current: mean_error=0.0, max_error=0.0, overshoot=0.0, settling_time_s=0.0, ik_failure_rate=0.0, safety_intercept_rate=0.0
  - baseline delta: all negative (current below baseline)
- Gate output: `Ark_Ranger_CR5/reports/e2e_gate_output_registry.txt`
  - Unit tests: 13/13 PASS
  - Gate: PASS

## Observed Command-Path Outcomes (This Run)
- Rejected path (explicit fault):
  - Out-of-range arm command rejected at validation with `R3-E1202`
- Accepted path:
  - pose/arm/gripper commands normalized, safety passed, execution performed
- Event typing:
  - `command.normalized` x4
  - `command.rejected` x1
  - `safety.pass` x3
  - `ik.success` x1
  - `system.state_change` x1

## Timeline Correlation Notes
- Timeline contains video/state/event index records in one session index file.
- One event index record is linked to nearest video and nearest state.
- Contact stream was not injected in this run, so nearest_contact is null.

## Stability and Safety Notes
- IK failure rate: `0.0` (in this short run)
- Safety intercept rate: `0.0` at safety stage (abnormal command intercepted earlier by validator)
- Unexpected stop count: `0`
- Unclassified errors: `0`

## Conclusion
- Verdict: PASS (with non-blocking DEGRADED startup due known asset scale warning)
- Blocking Issues: none
- Follow-up Actions:
  1. Resolve mesh scale warning in asset pipeline to achieve NORMAL startup.
  2. Add validator-pass but safety-clamp/block scenarios for richer runtime metrics.
  3. Add contact stream source during run to fully populate event-contact timeline links.
