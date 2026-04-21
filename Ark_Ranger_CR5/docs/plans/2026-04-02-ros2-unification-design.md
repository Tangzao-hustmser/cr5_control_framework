# ROS2-Only Unification Design (Ark Ranger CR5)

**Date:** 2026-04-02

## Goal
Unify all runtime command ingress, control, observability, and lifecycle management on ROS2 for Ark_Ranger_CR5 while preserving the existing R3 pipeline (validation, safety, execution, logging). JSON r3.v1 remains as a debug/compat bridge only. Ark-LCM/ark_patch are downgraded to explicit, opt-in compatibility paths and removed from the default execution chain.

## Architecture
The Windows-side Isaac Sim process hosts the ROS2 Action Server `/r3/command`, ROS2 system Services (`/r3/pause`, `/r3/resume`, `/r3/reset`, `/r3/stop`, `/r3/health`, `/r3/asset_check`), and ROS2 Topics (`/r3/events`, `/r3/status`, `/r3/validation`). The WSL2 Ubuntu 22.04 environment runs ROS2 Humble clients and tooling, communicating via DDS across the WSL/Windows boundary. The R3 pipeline remains intact and is invoked by the Action Server, which becomes the single main entrypoint. JSON input (terminal or legacy) is moved to a standalone JSON¡úROS2 bridge node.

## Components
- `ros2_ws/src/r3_msgs`: ROS2 interface package defining typed msgs/srvs/actions for r3.v1.
- `src/ros2/r3_action_server.py`: `/r3/command` action server and goal lifecycle handling.
- `src/ros2/r3_services.py`: runtime control and health/asset checks as services.
- `src/ros2/r3_publishers.py`: structured event/status/validation topics aligned with JSONL logs.
- `src/isaac_node.py`: refactored to expose `handle_command()`; no adapter-driven scheduling in the main path.
- `src/r3/adapters.py` + `src/ros2/r3_json_bridge.py`: JSON compatibility bridge (explicit opt-in).
- `ark_patch.py` / `run_registry.py`: compatibility-only guardrails, disabled by default.

## Data Flow
ROS2 Action goal ¡ú `r3_action_server` ¡ú `ArkIsaacSimNode.handle_command()` ¡ú R3 validator ¡ú Safety filter ¡ú `active_command` ¡ú `ArkIsaacBridge.step()` ¡ú Structured logs + timeline index ¡ú ROS2 Topics (`events/status/validation`). System operations are routed through ROS2 Services, but share the same validation and logging path via the unified command handler. JSON input uses the bridge: JSON line ¡ú `LocalInputAdapter` parsing ¡ú `Command.action` goal ¡ú same pipeline. `command_id` and `timestamp_ms` are the common correlation keys across JSONL logs, ROS2 topics, and rosbag playback.

## Error Handling
All validation and safety failures use stable R3 fault codes (`R3-E1xxx`). Action results map: validation failure or safety block ¡ú `abort`; success or safety pass/clamp ¡ú `succeed`. System services return success/false with fault code and message. ROS2 health/asset checks return JSON-serialized reports identical in structure to startup checks; startup still blocks on `BLOCKED` status. ROS2 initialization failures (missing `rclpy` or `r3_msgs`) raise explicit errors and surface through health checks.

## Testing Strategy
- Unit tests for msg<->dict mapping utilities (schema coverage, defaults, fault propagation).
- Adapter tests remain to validate JSON shortcut parsing for compatibility.
- E2E workflow: run Isaac Sim with ROS2-only mode, send Action goals from WSL, verify JSONL logs, publish topics, and replay+benchmark outputs; save report into `reports/`.
- Regression: keep legacy JSON bridge and LCM compat disabled by default but runnable for rollback validation.
