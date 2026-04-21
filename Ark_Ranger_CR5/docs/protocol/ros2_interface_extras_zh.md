# ROS2 接口补充说明（2026-04）

本文件说明 `ros2_ws/src/r3_msgs` 当前已启用的接口边界，确保运行时通信、调度、服务和可观测性都走 ROS2 主链路。

## 1. 已启用接口

- `msg/Command.msg`
- `msg/CommandResult.msg`
- `msg/RuntimeStatus.msg`
- `msg/SafetyDecision.msg`
- `msg/ValidationIssue.msg`
- `msg/SystemEvent.msg`
- `srv/SystemControl.srv`
- `srv/HealthCheck.srv`
- `srv/AssetCheck.srv`
- `srv/CheckReport.srv`
- `action/Command.action`

## 2. 运行时映射

- 命令主入口：`/r3/command` (`r3_msgs/action/Command`)
- 系统控制：`/r3/pause` `/r3/resume` `/r3/reset` `/r3/stop` (`r3_msgs/srv/SystemControl`)
- 健康检查：`/r3/health` (`r3_msgs/srv/HealthCheck`)
- 资产检查：`/r3/asset_check` (`r3_msgs/srv/AssetCheck`)
- 事件流：`/r3/events` (`r3_msgs/msg/SystemEvent`)
- 状态流：`/r3/status` (`r3_msgs/msg/RuntimeStatus`)
- 验证/安全流：`/r3/validation` (`r3_msgs/msg/CommandResult`)

## 3. 兼容层位置

- `r3.v1` JSON 输入仅作为兼容桥接（`run_json_bridge.py`、`tcp_json`）。
- 兼容层不再是主流程入口，默认配置下不会启用 adapter/LCM 路径。

## 4. 对齐原则

- `command_id` 与 `timestamp_ms` 在 Action、Service 响应、Topic 事件、JSONL 日志中保持可追溯。
- `timeline_index.jsonl` 通过 `event_id`（`evt-<timestamp_ms>-<seq>-<command_id>`）与 `/r3/events` 对齐。
- `/r3/events` 的 `SystemEvent` 额外提供 `timeline_index` 与 `timeline_json`，可直接关联 `timeline_index.jsonl`。
