# Ark Ranger CR5 仿真完整指南（Windows + WSL 已验证）

本文面向第一次接手 `Ark_Ranger_CR5` 的同学，目标是让你在不了解历史背景的情况下，也能独立完成：

1. 环境准备
2. 启动仿真
3. 发送命令
4. 观测执行
5. 停机与回放
6. 生成 benchmark 报告

项目长期目标是 **ROS2-only 主流程**。  
当前仓库默认配置为 **Windows + WSL 友好模式**（`tcp_json` 兼容入口），用于规避 Windows 侧 `rclpy` 依赖问题。

> 本文流程按实际联调记录验证通过（2026-04-06）：  
> `WSL bridge -> /r3/command -> Windows tcp_json listener -> Isaac 执行`

---

## 0. 本次联调总结（问题 -> 原因 -> 修复）

1. 启动报 `Startup blocked by health check`
- 原因：预检失败（依赖或资产检查未通过）
- 修复：查看 `reports/startup_health_report.json` 与 `reports/asset_consistency_report.json`，按报告修复后再启动

2. `ros2 action send_goal` 报 `The passed action type is invalid` 或 `UnsupportedTypeSupport`
- 原因：`r3_msgs` 未正确 source / 动态库路径未加载
- 修复：在 WSL 里 `source /opt/ros/humble/setup.bash`，并补齐 `AMENT_PREFIX_PATH`、`PYTHONPATH`、`LD_LIBRARY_PATH`

3. `ModuleNotFoundError: rclpy._rclpy_pybind11`
- 原因：落在 conda Python（例如 3.13），与 ROS2 Humble 的 Python 3.10 ABI 不匹配
- 修复：`conda deactivate`，桥接脚本使用 `/usr/bin/python3`

4. `bash: windows-ip: No such file or directory`
- 原因：把 `<windows-ip>` 占位符原样执行，`< >` 被 shell 当重定向
- 修复：先求真实 IP 再传参，不要带尖括号

5. `bridge_sent` 但机器人不动
- 结论：`bridge_sent` 只表示“桥已发到 TCP”，不表示 Isaac 已执行成功
- 排查：看 Windows 侧 `reports/logs/<session_id>/commands.jsonl`、`validation.jsonl`、`safety.jsonl` 与 `events.jsonl`

---

## 1. 项目在做什么

`Ark_Ranger_CR5` 是一个基于 Isaac Sim 的移动底盘 + 机械臂仿真项目，运行时采用 R3 管线：

1. `协议校验`（validator）
2. `安全过滤`（safety）
3. `执行`（Isaac 控制）
4. `日志与可观测性`（JSONL + ROS2 topics）

命令主入口为 ROS2 Action：

- `/r3/command` (`r3_msgs/action/Command`)

系统控制与检查入口为 ROS2 Service：

- `/r3/pause` `/r3/resume` `/r3/reset` `/r3/stop`
- `/r3/health`
- `/r3/asset_check`

---

## 2. 关键目录速览

- `scripts/`：启动、回放、benchmark、验收脚本
- `src/isaac_node.py`：主执行节点（R3 执行链）
- `src/ros2/`：Action/Service/Topic 运行时组件
- `configs/runtime_config.yaml`：运行模式和入口配置
- `ros2_ws/src/r3_msgs/`：ROS2 消息定义
- `reports/`：运行日志与报告输出

---

## 3. 运行模式（先搞清）

### 3.1 当前默认模式（推荐直接用）

`configs/runtime_config.yaml` 默认值：

- `runtime.mode: compat`
- `runtime.input_source: tcp_json`
- `runtime.compat.enable_adapters: true`
- `runtime.compat.enable_degraded_fallback: false`

含义：

- Windows 侧主入口是 TCP JSON（供 WSL ROS2 桥接）
- 绕开 Windows 侧 `rclpy` 缺失导致的阻断
- `status_output.destination: auto` 时，`tcp_json` 默认写状态到文件，不在主终端刷 step 日志

### 3.2 目标模式（后续可切）

若 Windows 侧 ROS2 依赖齐全，可切到：

- `runtime.input_source: ros2_action`
- `runtime.mode: ros2_only`（可选）

---

## 4. 环境准备（Windows + WSL Ubuntu 22.04）

### 4.1 Windows（Isaac Sim）

```powershell
cd E:\isaccsim\isaac-sim-standalone-5.1.0-windows-x86_64\Ark_Ranger_CR5
..\python.bat scripts\run_simulation.py
```

跨端通信建议：

```powershell
set ROS_DOMAIN_ID=0
set ROS_LOCALHOST_ONLY=0
```

### 4.2 WSL（Ubuntu 22.04 + ROS2 Humble）

```bash
conda deactivate
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

构建并 source `r3_msgs`：

```bash
cd /mnt/e/isaccsim/isaac-sim-standalone-5.1.0-windows-x86_64/Ark_Ranger_CR5/ros2_ws
colcon build
source install/setup.bash
```

如遇 `r3_msgs` 类型支持导入错误，补充：

```bash
export AMENT_PREFIX_PATH=/mnt/e/isaccsim/isaac-sim-standalone-5.1.0-windows-x86_64/Ark_Ranger_CR5/ros2_ws/install/r3_msgs:$AMENT_PREFIX_PATH
export PYTHONPATH=/mnt/e/isaccsim/isaac-sim-standalone-5.1.0-windows-x86_64/Ark_Ranger_CR5/ros2_ws/install/r3_msgs/local/lib/python3.10/dist-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/mnt/e/isaccsim/isaac-sim-standalone-5.1.0-windows-x86_64/Ark_Ranger_CR5/ros2_ws/install/r3_msgs/lib:/mnt/e/isaccsim/isaac-sim-standalone-5.1.0-windows-x86_64/Ark_Ranger_CR5/ros2_ws/install/r3_msgs/local/lib/python3.10/dist-packages/r3_msgs:$LD_LIBRARY_PATH
```

快速自检：

```bash
/usr/bin/python3 -c "import rclpy; print('rclpy ok')"
/usr/bin/python3 -c "from r3_msgs.action import Command; print('r3_msgs typesupport OK')"
```

---

## 5. 第一次启动（标准流程）

在 Windows 启动：

```powershell
cd E:\isaccsim\isaac-sim-standalone-5.1.0-windows-x86_64\Ark_Ranger_CR5
..\python.bat scripts\run_simulation.py
```

启动前会生成：

- `reports/asset_consistency_report.json`
- `reports/startup_health_report.json`

状态解释：

- `NORMAL`：正常启动
- `DEGRADED`：降级启动（仅显式开启 `runtime.compat.enable_degraded_fallback: true` 时）
- `BLOCKED`：阻断启动，先修复报告问题

---

## 6. 发送命令（ROS2 主入口）

### 6.1 Action：发送运动命令

在 WSL：

```bash
ros2 action send_goal /r3/command r3_msgs/action/Command "{command: {protocol_version: 'r3.v1', command_id: 'cmd-demo-001', timestamp_ms: 1710000000000, source: 'ros2_cli', command_type: 'pose', payload_json: '{\"frame\":\"world\",\"position_m\":[0.40,0.00,0.55],\"orientation_wxyz\":[1,0,0,0]}', metadata_json: '{}'}}" --feedback
```

可将 `command_type` 改为 `arm/gripper/system`，并同步修改 `payload_json`。

注意：

- 上面的 JSON 字符串必须作为 `ros2 action send_goal` 的参数整体传入
- 不要单独执行 `{command: ...}`，否则会出现 `command not found`

### 6.1.1 推荐：使用短命令脚本（避免手敲长 JSON）

仓库已提供 `scripts/send_r3_goal.py`，建议优先使用：

```bash
/usr/bin/python3 /mnt/e/isaccsim/isaac-sim-standalone-5.1.0-windows-x86_64/Ark_Ranger_CR5/scripts/send_r3_goal.py pose --x 0.45 --y 0.15 --z 0.80 --feedback
```

更多示例：

```bash
/usr/bin/python3 /mnt/e/isaccsim/isaac-sim-standalone-5.1.0-windows-x86_64/Ark_Ranger_CR5/scripts/send_r3_goal.py arm --joints 0 -0.5 1.0 0 0.5 0 --feedback
/usr/bin/python3 /mnt/e/isaccsim/isaac-sim-standalone-5.1.0-windows-x86_64/Ark_Ranger_CR5/scripts/send_r3_goal.py gripper --mode open
/usr/bin/python3 /mnt/e/isaccsim/isaac-sim-standalone-5.1.0-windows-x86_64/Ark_Ranger_CR5/scripts/send_r3_goal.py system --operation stop --reason manual
```

如果你坚持用 `ros2 action send_goal` 原生命令，也可以写“最小字段版”：

```bash
ros2 action send_goal /r3/command r3_msgs/action/Command "{command: {command_type: 'pose', payload_json: '{\"frame\":\"world\",\"position_m\":[0.45,0.15,0.8],\"orientation_wxyz\":[1,0,0,0]}'}}" --feedback
```

### 6.2 Service：系统控制

```bash
ros2 service call /r3/pause r3_msgs/srv/SystemControl "{operation: 'pause', reason: 'manual'}"
ros2 service call /r3/resume r3_msgs/srv/SystemControl "{operation: 'resume', reason: 'manual'}"
ros2 service call /r3/reset r3_msgs/srv/SystemControl "{operation: 'reset', reason: 'manual'}"
ros2 service call /r3/stop r3_msgs/srv/SystemControl "{operation: 'stop', reason: 'manual'}"
```

### 6.3 Service：运行时检查

```bash
ros2 service call /r3/health r3_msgs/srv/HealthCheck "{include_details: true, request_id: ''}"
ros2 service call /r3/asset_check r3_msgs/srv/AssetCheck "{include_details: true, request_id: ''}"
```

---

## 7. 观察系统状态

### 7.1 ROS2 Topics（实时）

```bash
ros2 topic echo /r3/status
ros2 topic echo /r3/events
ros2 topic echo /r3/validation
```

### 7.2 JSONL（权威日志）

每次仿真生成一个 session 目录：

- `reports/logs/<session_id>/`

关键文件：

- `commands.jsonl`
- `validation.jsonl`
- `safety.jsonl`
- `ik.jsonl`
- `state.jsonl`
- `events.jsonl`
- `timeline_index.jsonl`

### 7.3 分屏观察（推荐）

主仿真终端保持清爽，在第二个 Windows 终端观察状态：

```powershell
..\python.bat scripts\watch_live_status.py --file reports\live_status\current_status.json
```

---

## 8. 停机、回放、benchmark

### 8.1 停机

```bash
ros2 service call /r3/stop r3_msgs/srv/SystemControl "{operation: 'stop', reason: 'manual'}"
```

### 8.2 回放

```powershell
..\python.bat scripts\replay_log.py --log-dir reports\logs\<session_id> --headless --ros2-only
```

### 8.3 benchmark

```powershell
..\python.bat scripts\run_benchmark.py --log-dir reports\logs\<session_id> --ros2-only
```

### 8.4 ROS2-only E2E

```powershell
..\python.bat scripts\run_ros2_e2e.py --log-dir reports\logs\<session_id> --strict
```

---

## 9. WSL 桥接实操（已验证）

适用场景：

- Windows 侧无法直接稳定运行 `rclpy`
- 需要在 WSL 使用 ROS2 CLI 控制 Windows 上的 Isaac

步骤：

1. Windows 启动仿真（监听 TCP，默认 `58000`）
2. WSL 终端 A 启动桥接：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
WIN_IP=$(ip route | awk '/default/ {print $3; exit}')
/usr/bin/python3 /mnt/e/isaccsim/isaac-sim-standalone-5.1.0-windows-x86_64/Ark_Ranger_CR5/scripts/run_ros2_tcp_bridge.py --host "$WIN_IP" --port 58000
```

3. WSL 终端 B 验证 Action 已注册：

```bash
ros2 action list -t | grep /r3/command
```

4. WSL 终端 B 发送命令（见第 6 节）

补充：

- 若 `ip route` 拿到的地址不可达，再尝试 `/etc/resolv.conf` 的 `nameserver`
- 命令里不要写 `<windows-ip>` 这种占位符字面值

---

## 10. 兼容输入（仅临时）

若需要手工 JSON 输入，可用：

```powershell
..\python.bat scripts\run_json_bridge.py
```

它会把 `r3-json>` 转为 ROS2 Action/Service 请求。  
该方式仅用于兼容迁移，不作为主流程。

---

## 11. 常见问题与排查

### 11.1 启动即 `BLOCKED`

看：

- `reports/startup_health_report.json`
- `reports/asset_consistency_report.json`

优先修复：

- 配置缺字段
- URDF/USD 路径错误
- `rclpy` 或 `r3_msgs` 不可用

### 11.2 `The passed action type is invalid`

常见原因：

- 没有 source 到包含 `r3_msgs` 的环境
- `r3_msgs` 没成功构建

先执行：

```bash
ros2 interface show r3_msgs/action/Command
```

### 11.3 `UnsupportedTypeSupport` 或 `libr3_msgs__rosidl_generator_py.so` 缺失

处理：

1. 确认已 `source install/setup.bash`
2. 必要时补齐 `AMENT_PREFIX_PATH`、`PYTHONPATH`、`LD_LIBRARY_PATH`
3. 用 `/usr/bin/python3 -c "from r3_msgs.action import Command"` 验证

### 11.4 `rclpy._rclpy_pybind11` 缺失

典型原因：conda Python 与 ROS2 ABI 不匹配。  
处理：`conda deactivate`，并用 `/usr/bin/python3` 运行桥接。

### 11.5 `bridge_sent` 但机器人不动

这是桥接层成功，不代表 Isaac 执行成功。  
进一步检查：

1. Windows 仿真是否仍在运行
2. `reports/logs/<session_id>/commands.jsonl` 是否有 `command.received`
3. `validation.jsonl` 是否 `ok=false`
4. `safety.jsonl` 是否 `block`
5. `events.jsonl` 是否有故障码（如 `R3-E14xx`/`R3-E15xx`）

### 11.6 快速链路自检

```powershell
..\python.bat scripts\run_gates.py --ros2-only
```

---

## 12. 新人 10 分钟演练清单

1. Windows 启动仿真
2. WSL 终端 A 启动桥接
3. WSL 终端 B 发送 `pose` Action
4. `ros2 topic echo /r3/status` 观察状态
5. 调用 `/r3/stop`
6. 回放 `replay_log.py --ros2-only`
7. 生成 `run_benchmark.py --ros2-only` 报告
8. 运行 `run_ros2_e2e.py --strict` 生成验收报告

---

## 13. 实机迁移入口

若要将该链路迁移到真实机器人，请继续阅读：

- `docs/real_robot_migration_guide_zh.md`
