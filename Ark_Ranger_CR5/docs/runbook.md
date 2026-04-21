# Ark Ranger CR5 Runbook (ROS2 Primary)

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
3. Preflight checks run automatically:
- Asset consistency (`reports/asset_consistency_report.json`)
- Startup health (`reports/startup_health_report.json`)

If status is `BLOCKED`, simulation does not start.
Startup only degrades when `runtime.compat.enable_degraded_fallback: true` is explicitly enabled.
In default `ros2_only` mode, missing ROS2 dependencies remain `BLOCKED`.

Current workspace default is bridge-friendly (`mode: compat`, `input_source: tcp_json`) for Windows + WSL deployment.

## 2. Send Commands (ROS2 Action)

Example pose goal:
```bash
ros2 action send_goal /r3/command r3_msgs/action/Command "{command: {protocol_version: 'r3.v1', command_type: 'pose', payload_json: '{\"frame\":\"world\",\"position_m\":[0.4,0.1,0.5],\"orientation_wxyz\":[1,0,0,0]}', metadata_json: '{}'}}" --feedback
```

System control services:
```bash
ros2 service call /r3/pause r3_msgs/srv/SystemControl "{operation: 'pause'}"
ros2 service call /r3/resume r3_msgs/srv/SystemControl "{operation: 'resume'}"
ros2 service call /r3/stop r3_msgs/srv/SystemControl "{operation: 'stop'}"
```

Health checks:
```bash
ros2 service call /r3/health r3_msgs/srv/HealthCheck "{include_details: true, request_id: ''}"
ros2 service call /r3/asset_check r3_msgs/srv/AssetCheck "{include_details: true, request_id: ''}"
```

## 3. Telemetry

Topics:
```bash
ros2 topic echo /r3/status
ros2 topic echo /r3/events
ros2 topic echo /r3/validation
```
`/r3/status` -> `r3_msgs/msg/RuntimeStatus`  
`/r3/events` -> `r3_msgs/msg/SystemEvent`  
`/r3/validation` -> `r3_msgs/msg/CommandResult`

Optional file watcher:
```bash
python scripts/watch_live_status.py
```

In `tcp_json` mode with `status_output.destination: auto`, step status is written to file by default (no console flood on the main sim terminal).

## 4. Compatibility Input (Optional)

JSON terminal bridge:
```bash
python scripts/run_json_bridge.py
```

TCP JSON bridge (WSL → Windows):
```bash
python scripts/run_ros2_tcp_bridge.py --host <windows-host> --port 58000
```

## 5. Replay

```bash
python scripts/replay_log.py --log-dir reports/logs/<session_id> --headless --ros2-only
```

## 6. Benchmark

```bash
python scripts/run_benchmark.py --log-dir reports/logs/<session_id> --ros2-only
```

## 7. ROS2-only E2E Report

```bash
python scripts/run_ros2_e2e.py --log-dir reports/logs/<session_id> --strict
```

## 8. Switch Input Source

Edit `configs/runtime_config.yaml`:
- `runtime.mode: ros2_only` (default)
- `runtime.input_source: ros2_action` (default)
- `runtime.input_source: local_terminal` (legacy/compat)
- `runtime.input_source: tcp_json` (TCP JSON adapter)
- `runtime.compat.enable_adapters: true` (required for local adapters)
- `runtime.compat.enable_degraded_fallback: false` (default; keep ROS2-only strict)
- `runtime.compat.degraded_input_source: local_terminal | tcp_json` (used only when degraded fallback is enabled)

## 9. Troubleshooting

- Startup blocked: check `reports/startup_health_report.json`
- Command rejected: check `validation.jsonl`
- Safety block/clamp: check `safety.jsonl` and `events.jsonl`
- IK failure: check `ik.jsonl` and `events.jsonl`

