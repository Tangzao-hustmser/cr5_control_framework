# Ark Ranger CR5 Runbook

For a full Chinese step-by-step guide, see `docs/simulation_guide_zh.md`.

## 1. Startup

1. Run gate checks:
```bash
python scripts/run_gates.py
```
2. Start simulation:
```bash
python scripts/run_simulation.py
```
3. The system runs preflight checks:
- Asset consistency (`reports/asset_consistency_report.json`)
- Startup health (`reports/startup_health_report.json`)
- Startup mode: `NORMAL`, `DEGRADED`, or `BLOCKED`
4. Optional live status watcher (second terminal):
```bash
python scripts/watch_live_status.py
```

If status is `BLOCKED`, simulation does not start.

## 2. Send Commands

Only unified protocol JSON is accepted.

Example:
```json
{"command_type":"pose","payload":{"frame":"world","position_m":[0.4,0.1,0.5],"orientation_wxyz":[1,0,0,0]}}
```

Arm by joints (JSON):
```json
{"command_type":"arm","payload":{"joint_positions_rad":[0.0,-0.5,1.0,0.0,0.5,0.0]}}
```

Arm by joints (terminal shortcut):
```text
arm 0 -30 45 0 0 0
arm_rad 0 -0.52 0.79 0 0 0
```

Invalid fields are rejected with stable fault codes.

## 3. Switch Input Source

Edit `configs/runtime_config.yaml`:
- `runtime.input_source: local_terminal` or `ros2`
- When `ros2`, ensure ROS2 dependencies are installed.

No control-core code changes are needed.

## 4. Replay

Replay an existing log session:
```bash
python scripts/replay_log.py --log-dir reports/logs/<session_id> --headless
```

## 5. Benchmark and Baseline Compare

```bash
python scripts/run_benchmark.py --log-dir reports/logs/<session_id>
```

Output:
- `reports/benchmark_report.json`

## 6. Troubleshooting

### Startup blocked
- Check `reports/startup_health_report.json`
- Resolve missing dependencies or critical asset mismatch

### Commands rejected
- Check `validation.jsonl` for exact fault code and field path

### Safety blocks/clamps
- Check `safety.jsonl` and `events.jsonl` for reason codes

### IK failures
- Check `ik.jsonl` and corresponding events (`R3-E1501`, `R3-E1502`)

### Gripper not moving
- Check `events.jsonl` for `R3-E1601` stall events

## 7. Log Files

Per session under `reports/logs/<session_id>/`:
- `commands.jsonl`
- `validation.jsonl`
- `safety.jsonl`
- `ik.jsonl`
- `state.jsonl`
- `events.jsonl`
- `timeline_index.jsonl`

