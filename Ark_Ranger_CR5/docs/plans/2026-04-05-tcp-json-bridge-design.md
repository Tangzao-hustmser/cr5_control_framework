# TCP JSON 旁路设计（Windows Isaac Sim ? WSL ROS2）

## 背景
Windows 侧 Isaac Sim 使用 Python 3.11，ROS2 Humble 的 `rclpy` 依赖 Python 3.10，导致同进程 ROS2 Action Server 无法启动。需要一个不依赖 Windows 侧 `rclpy` 的命令入口，同时保持 R3 校验/安全/执行/日志链路不变。

## 备选方案
- 方案1：Windows 进程内 TCP JSON 服务器 + WSL ROS2→TCP 桥接（推荐）
- 方案2：仅提供 TCP JSON，手动 `nc` 发送
- 方案3：独立 Windows 网关进程（复杂度高、维护成本大）

## 选型结论
采用方案1。复用 `r3.v1` JSON 行协议，Windows 侧仅增加 TCP 适配器，WSL 侧提供 ROS2 Action/Service 到 TCP 的桥接节点。实现成本低、风险可控、与现有日志与回放链路兼容。

## 组件与职责
- `TcpJsonInputAdapter`（Windows）：监听 TCP，按行解析 JSON/快捷命令，推入现有 adapter 队列。
- `run_ros2_tcp_bridge.py`（WSL）：提供 `/r3/command` Action Server 和 `/r3/pause|resume|reset|stop` Service，将请求序列化为 r3.v1 JSON 并发送到 TCP。
- `run_simulation.py`：在 ROS2 依赖缺失时降级运行，并按配置切换输入源（内存级）。

## 数据流
WSL ROS2 Action/Service → TCP JSON → Windows `TcpJsonInputAdapter` → 校验/安全/执行/日志 → JSONL 记录。TCP 侧仅保证“已发送”，执行结果通过 JSONL/状态输出观察。

## 错误处理
- TCP 行超长：直接丢弃并生成 `R3-E1001` adapter error。
- JSON 解析失败：生成 `R3-E1001`，进入拒绝事件与 validation 记录。
- TCP 发送失败：Action result 标记 `bridge_send_failed`。

## 配置
新增 `runtime.input_source: tcp_json` 与 `runtime.adapters.tcp_json`，并提供 `runtime.compat.degraded_input_source` 选择降级目标（默认 `local_terminal`）。

## 测试路径
- Windows 启动仿真并显示 TCP 监听地址。
- WSL 运行 `run_ros2_tcp_bridge.py`，用 ROS2 Action/Service 发送命令。
- `nc` 发送单行 JSON 验证执行与日志。
- 断线重连验证适配器持续监听。
