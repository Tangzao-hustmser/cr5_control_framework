# Ark Ranger CR5 仿真指南（新手可直接上手）

## 1. 这份指南解决什么问题

如果你第一次接触这个项目，按本文可以完成一整轮：
启动仿真 -> 发送命令 -> 观察执行 -> 停止并拿到日志 -> 回放 -> 生成指标。

## 2. 先理解“命令发在哪”

命令不是在另一个脚本里发，也不是改配置文件发。
命令是发在运行 `run_simulation.py` 的那个终端里。

当你看到下面这种提示时，说明可以发命令：

```text
r3-json>
```

你要做的是：在 `r3-json>` 后面输入一行命令（JSON 或关节角快捷命令），按回车。

## 3. 启动前准备（Windows）

建议开三个终端：

- 终端 A：运行 Registry
- 终端 B：运行仿真并输入命令（`r3-json>`）
- 终端 C：查看实时机械臂状态（可选，但推荐）

### 3.1 终端 A：启动 Registry

```powershell
cd E:\isaccsim\isaac-sim-standalone-5.1.0-windows-x86_64\Ark_Ranger_CR5
..\python.bat run_registry.py
```

保持这个终端不要关闭。

### 3.2 终端 B：进入项目并可选跑门禁

```powershell
cd E:\isaccsim\isaac-sim-standalone-5.1.0-windows-x86_64\Ark_Ranger_CR5
python scripts/run_gates.py
```

### 3.3 终端 C（可选）：准备状态监视

先进入项目目录，后续会用到：

```powershell
cd E:\isaccsim\isaac-sim-standalone-5.1.0-windows-x86_64\Ark_Ranger_CR5
```

## 4. 启动仿真

在终端 B 执行：

```powershell
python scripts/run_simulation.py
```

如果出现 `ModuleNotFoundError: No module named 'isaacsim'`，改用 Isaac 根目录命令：

```powershell
cd E:\isaccsim\isaac-sim-standalone-5.1.0-windows-x86_64
.\python.bat Ark_Ranger_CR5\scripts\run_simulation.py
```

启动后会自动做：
- 资产一致性检查：`reports/asset_consistency_report.json`
- 健康检查：`reports/startup_health_report.json`

如果健康状态是 `BLOCKED`，系统会阻断启动。

## 5. 怎么发一条命令（重点）

### 5.1 发命令的动作

1. 等待终端出现 `r3-json>`。
2. 复制一条“单行命令”（JSON 或关节角快捷命令）。
3. 粘贴到 `r3-json>` 后面。
4. 按回车发送。

### 5.2 一条 `pose` 命令示例（可直接粘贴）

```json
{"command_type":"pose","payload":{"frame":"world","position_m":[0.45,0.15,0.8],"orientation_wxyz":[1,0,0,0]}}
```

发送后命令会被执行。默认本地交互模式下，状态不会持续刷在 `r3-json>` 终端，避免影响你继续输入。

如果你要实时看机械臂状态，请在终端 C 执行：

```powershell
python scripts/watch_live_status.py
```

你会看到类似输出：

```text
[LiveStatus][step=000360] cmd=cmd-xxxx EE xyz=(0.402, 0.001, 0.553) | arm=[...] | grip=0.0004
```

这说明命令已经进到执行链路，且状态在独立终端显示，不会打断 `r3-json>` 输入。

### 5.3 关节角输入控制

你现在可以直接在 `r3-json>` 输入关节角来控制机械臂姿态：

- 角度制（推荐，单位是度）：`arm`
- 弧度制：`arm_rad`

示例（6 轴机器人）：

```text
arm 0 -30 45 0 0 0
```

```text
arm_rad 0 -0.52 0.79 0 0 0
```

说明：
- 输入顺序就是关节顺序 `j1 j2 j3 j4 j5 j6`。
- `arm` 会自动把角度（deg）换算成弧度（rad）后执行。
- 关节数量或范围不合法会被校验器拒绝并返回故障码。

## 6. 命令模板（你可以照着改参数）

### 6.1 常用简化模板（本地终端推荐）

`pose`：

```json
{"command_type":"pose","payload":{"frame":"world","position_m":[X,Y,Z],"orientation_wxyz":[W,X,Y,Z]}}
```

`arm`：

```json
{"command_type":"arm","payload":{"joint_positions_rad":[j1,j2,j3,j4,j5,j6]}}
```

`arm` 快捷输入（角度）：

```text
arm j1 j2 j3 j4 j5 j6
```

`arm_rad` 快捷输入（弧度）：

```text
arm_rad j1 j2 j3 j4 j5 j6
```

`gripper`：

```json
{"command_type":"gripper","payload":{"mode":"open"}}
```

`system`：

```json
{"command_type":"system","payload":{"operation":"pause"}}
```

### 6.2 完整协议模板（ROS2/外部总线推荐）

```json
{
  "protocol_version": "r3.v1",
  "command_id": "cmd-demo-001",
  "timestamp_ms": 1710000000000,
  "source": "local_terminal",
  "command_type": "pose",
  "payload": {
    "frame": "world",
    "position_m": [0.40, 0.00, 0.55],
    "orientation_wxyz": [1, 0, 0, 0]
  },
  "metadata": {
    "operator": "demo"
  }
}
```

## 7. 常用指令案例（可直接复制）

### 7.1 到一个目标点（pose）

```json
{"command_type":"pose","payload":{"frame":"world","position_m":[0.45,0.10,0.55],"orientation_wxyz":[1,0,0,0]}}
```

### 7.2 关节角控制（arm）

角度输入（推荐）：

```text
arm 0 -20 35 0 15 0
```

弧度输入：

```text
arm_rad 0 -0.35 0.61 0 0.26 0
```

JSON 输入（与快捷输入等价）：

```json
{"command_type":"arm","payload":{"joint_positions_rad":[0,-0.35,0.61,0,0.26,0]}}
```

### 7.3 夹爪开/合

```json
{"command_type":"gripper","payload":{"mode":"open"}}
```

```json
{"command_type":"gripper","payload":{"mode":"close"}}
```

### 7.4 暂停/恢复/停止

```json
{"command_type":"system","payload":{"operation":"pause"}}
```

```json
{"command_type":"system","payload":{"operation":"resume"}}
```

```json
{"command_type":"system","payload":{"operation":"stop"}}
```

## 8. 如何结束本次仿真

推荐发送停止命令：

```json
{"command_type":"system","payload":{"operation":"stop"}}
```

结束时终端会打印日志会话目录，例如：

```text
[Simulation] Log session: .../reports/logs/20260329T160057Z
```

记下这个 `<session_id>`，后续回放和指标都用它。

## 9. 回放与指标（闭环后处理）

### 9.1 回放

```powershell
python scripts/replay_log.py --log-dir reports/logs/<session_id> --headless
```

关注输出字段：
- `replayed_commands`
- `dropped_commands`
- `avg_state_delta`
- `max_state_delta`

### 9.2 指标计算与基线对比

```powershell
python scripts/run_benchmark.py --log-dir reports/logs/<session_id>
```

默认输出：
- `reports/benchmark_report.json`

你也可以按 session 单独输出：

```powershell
python scripts/run_benchmark.py --log-dir reports/logs/<session_id> --output reports/benchmark_report_<session_id>.json
```

## 10. 日志怎么查“命令之后发生了什么”

同一 session 目录下重点看：
- `commands.jsonl`：收到了什么原始命令
- `validation.jsonl`：校验是否通过，失败码是什么
- `safety.jsonl`：是 pass / clamp / block
- `ik.jsonl`：IK 成功还是失败
- `state.jsonl`：执行后的状态
- `events.jsonl`：异常/故障/系统事件
- `timeline_index.jsonl`：统一时间轴索引

## 11. 新手最容易踩的坑

1. 在 PowerShell 提示符（`PS ...>`）下发 JSON，而不是在 `r3-json>` 下发。
2. JSON 不是单行，或者多了中文引号、尾逗号。
3. `run_registry.py` 终端被关掉，导致通信异常。
4. 用了系统 `python` 启动，没用 Isaac 的 `python.bat`。
5. 期待状态行出现在 `r3-json>` 终端。现在默认是“输入终端和状态终端分离”，请用 `watch_live_status.py` 看实时状态。

## 12. 最短可执行路径（3 分钟）

1. 终端 A 启动 `run_registry.py`。
2. 终端 B 启动 `run_simulation.py`。
3. 终端 C 启动 `python scripts/watch_live_status.py`（可选）。
4. 在 `r3-json>` 粘贴一条 `pose`。
5. 再粘贴一条 `system.stop`。
6. 用打印出的 session 跑 `replay_log.py`。
7. 跑 `run_benchmark.py` 生成报告。
