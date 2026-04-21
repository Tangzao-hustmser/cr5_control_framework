# Ark Ranger CR5 实机迁移完整方案（风险优化版）

本文是 `Ark_Ranger_CR5` 从 Isaac 仿真迁移到 CR5+AG95 实机的执行方案。  
本版以“风险优先”重构：先消除高风险，再推进功能。

---

## 1. 迁移终态目标（必须达成）

1. 保持 ROS2 主入口不变：`/r3/command` Action 与 `/r3/*` Services 对外不变。  
2. 保持 R3 管线不变：校验 -> 安全 -> 执行 -> 日志。  
3. 执行层从 Isaac 切换为实机执行器，且 sim/real 通过同一运行时调用。  
4. 安全链路设备化：急停、保护停、远端确认、驱动故障进入 R3 事件与 fault_code。  
5. 日志链路一致：`command_id` 在 Action、设备回执、JSONL、rosbag 全链路可追踪。  
6. 可回退：任意阶段都可一键回到仿真执行链。

---

## 2. 设计边界与硬约束

1. 迁移阶段不改 `r3_msgs` 协议字段，避免上层入口漂移。  
2. 不将实机逻辑硬编码进 `src/isaac_node.py`，必须做执行器抽象。  
3. 不把“机器人能动”当完成标准，必须包含安全停机与故障闭环。  
4. 实机控制机采用原生 Ubuntu 22.04 + ROS2 Humble，不以 WSL 为生产路径。  
5. 满足设备文档硬约束：CR5 控制柜安全 I/O、AG95 24V 与初始化、AG95 命令节流 >=50ms。

---

## 3. 风险优先优化（核心）

### 3.1 风险台账（必须逐项闭环）

| 风险ID | 风险描述 | 触发信号 | 强制门禁 | 缓解动作 | 闭环标准 |
|---|---|---|---|---|---|
| R01 | 驱动路线不清（ROS2原生 vs ROS1过渡） | 同一功能出现双套实现 | 未决策禁止开发驱动代码 | 先产出 ADR，冻结一条主路线 | ADR 落地且配置中只有一条主路线 |
| R02 | 执行层与 Isaac 强耦合 | 实机代码需要 import Isaac 组件 | 未抽象执行器禁止接入实机驱动 | 引入 `executor_base` 抽象 | sim/real 可无侵入切换 |
| R03 | 安全回路未接入软件门禁 | 急停触发但系统仍可下发动作 | 未通过安全联调禁止动作测试 | 接入用户急停/保护停/远端确认状态 | 风险状态下命令必 block |
| R04 | AG95 上电未初始化导致失控 | 重启后首条命令失败或异常 | 未加初始化状态机禁止抓取测试 | 强制 init handshake + 状态确认 | 每次上电后自动初始化成功 |
| R05 | 供电/接线不满足手册约束 | AG95 力不足、控制间歇失效 | 未通过供电检查禁止性能测试 | 校验 24V 与末端电流能力 | 抓取力与响应稳定 |
| R06 | 命令风暴导致夹爪异常 | 高频下发导致状态抖动/堵塞 | 未加节流禁止连续控制测试 | 强制 command rate limiter（>=50ms） | 连续命令无丢失无阻塞 |
| R07 | 网络配置与模式错误 | TCP/RS485/CAN 连接偶发失败 | 未通过连通测试禁止联调 | 固化网络参数与拨码 SOP | 启动后可稳定连接 |
| R08 | 故障码未映射到 R3 | 日志只有原始错误难定位 | 未建立映射禁止验收 | 建立设备 fault -> R3 fault_code 表 | 故障可定位可处理 |
| R09 | 日志回放不一致 | rosbag 与 JSONL 事件顺序错乱 | 未对齐 timeline 禁止发布 | 强制 `command_id/timestamp_ms` 对齐 | 回放可复现关键事件 |
| R10 | 回退链路不可用 | 失败后无法快速恢复 | 无回退脚本禁止实机放量 | 配置化 executor 切换 + 回退脚本 | 任意阶段可回 sim |

### 3.2 风险门禁规则（Stop-The-Line）

1. 任一高风险项（R01-R05）未闭环，禁止进入下一阶段。  
2. 任一安全门禁失败（R03），立即停止动作测试，仅保留诊断测试。  
3. 任一日志一致性门禁失败（R09），禁止出具“可发布”结论。  
4. 回退门禁失败（R10），禁止实机长时运行。

---

## 4. 目标架构（风险优化后）

1. 保持不变：
- `ros2_ws/src/r3_msgs/*`
- `src/ros2/r3_action_server.py`
- `src/ros2/r3_services.py`
- `src/ros2/r3_publishers.py`
- `src/r3/structured_logging.py`
- `src/r3/timeline.py`

2. 重构核心：
- 运行时绑定抽象执行器，而不是绑定 Isaac。
- 执行器分 `isaac_executor` 与 `real_robot_executor`。
- 实机执行器内部拆分 `cr5_executor` 与 `ag95_executor`。

3. 兼容链路降级：
- `tcp_json`、`run_ros2_tcp_bridge.py` 仅作为旁路，不作为主流程依赖。

---

## 5. 文件级修改点（按风险映射）

### 5.1 R02 执行器抽象化（先做）

- 修改：`src/isaac_node.py`
- 新增：`src/executors/executor_base.py`
- 新增：`src/executors/isaac_executor.py`
- 新增：`src/executors/real_robot_executor.py`

完成标准：
- 同一 `/r3/command` 在 sim/real 下均可执行；
- 实机路径不依赖 Isaac import。

### 5.2 R01+R07 CR5 实机执行器

- 新增：`src/executors/cr5_executor.py`
- 新增：`configs/hardware/cr5.yaml`

完成标准：
- `arm` 命令可执行并稳定回读；
- 网络/地址/连接模式参数可配置，不写死。

### 5.3 R04+R05+R06 AG95 实机执行器

- 新增：`src/executors/ag95_client.py`
- 新增：`src/executors/ag95_executor.py`
- 新增：`configs/hardware/ag95.yaml`

完成标准：
- 上电自动初始化；
- 命令节流 >=50ms；
- `gripper` 命令执行与反馈稳定。

### 5.4 R03 系统服务映射设备动作

- 修改：`src/ros2/r3_services.py`

完成标准：
- `/r3/pause /r3/resume /r3/reset /r3/stop` 与设备动作一致；
- 支持 `servo_on/servo_off/clear_error/home/emergency_stop`。

### 5.5 R03+R08 安全与故障映射

- 修改：`src/r3/safety.py`
- 修改：`src/r3/events.py`
- 新增：`configs/fault_map_real_robot.yaml`

完成标准：
- 设备风险态命令必 block；
- 驱动错误可映射为 R3 fault_code。

### 5.6 R03+R07+R08 健康检查设备化

- 修改：`src/r3/health.py`
- 修改：`src/ros2/r3_services.py`

完成标准：
- `/r3/health` 输出设备可连通、初始化、安全回路状态；
- 报告可直接指导排障。

### 5.7 R10 配置与回退机制

- 修改：`configs/runtime_config.yaml`
- 新增：`configs/runtime_real_robot.yaml`
- 新增：`scripts/switch_executor_profile.py`

完成标准：
- 可配置切换 `executor: isaac|real`；
- 失败时可快速回退至仿真链路。

### 5.8 R09 验收与回放一致性

- 修改：`scripts/run_gates.py`
- 修改：`scripts/run_ros2_e2e.py`
- 新增：`scripts/run_real_robot_gates.py`

完成标准：
- 自动报告包含连通、安全、执行、日志一致性四类结果；
- 回放关键事件序列一致。

---

## 6. 推进机制（怎么推进）

1. 风险优先推进顺序：R01 -> R02 -> R03 -> R04/R05/R06 -> R07/R08 -> R09 -> R10。  
2. 每个任务必须绑定风险ID与门禁条件。  
3. 每次提交必须附带“验证证据”：命令回执、日志片段、服务返回、故障注入结果。  
4. 每阶段结束必须完成一次“回退演练”。  
5. 未达到门禁条件，不进入下一阶段。

---

## 7. 可执行任务清单模板（风险版）

### 7.1 任务模板（复制即用）

| 字段 | 内容要求 |
|---|---|
| 任务ID | 例如 `Txx` |
| 目标 | 本任务要实现什么能力 |
| 关联风险ID | 至少一个，如 `R03` |
| 输入 | 文档、配置、设备条件 |
| 输出 | 代码、配置、报告 |
| 修改文件 | 绝对/相对路径列表 |
| 实施动作 | 需要执行的具体修改 |
| 验证动作 | 如何验证通过 |
| 门禁条件 | 未满足则禁止推进 |
| 回退动作 | 失败时如何回退 |
| 完成标准 | 可判定通过的客观条件 |

### 7.2 可执行任务清单（建议基线）

| ID | 任务目标 | 关联风险 | 输入 | 输出 | 修改点 | 门禁条件 | 完成标准 |
|---|---|---|---|---|---|---|---|
| T01 | 冻结驱动路线并产出 ADR | R01 | 手册+现有代码 | 决策文档 | `docs/plans/*` | ADR 未通过评审不得编码 | 路线唯一且可执行 |
| T02 | 执行器抽象层落地 | R02 | 当前执行链 | `executor_base` 与 runtime 重构 | `src/isaac_node.py` `src/executors/*` | sim 回归未通过不得接实机 | sim/real 可切换 |
| T03 | CR5 执行器接入 | R01 R07 | CR5 SDK/API | `cr5_executor.py` | `src/executors/cr5_executor.py` | 连通失败不得动作测试 | arm 可执行并回读 |
| T04 | AG95 客户端与执行器接入 | R04 R05 R06 | AG95 协议 | `ag95_client.py` `ag95_executor.py` | `src/executors/*` | 初始化失败不得抓取测试 | gripper 稳定控制 |
| T05 | 系统服务映射设备控制 | R03 | 现有 `/r3/*` 服务 | 服务增强 | `src/ros2/r3_services.py` | stop 无效禁止继续联调 | 服务与设备状态一致 |
| T06 | 安全门禁与故障码映射 | R03 R08 | 安全回路与驱动错误 | 安全策略+映射表 | `src/r3/safety.py` `configs/fault_map_real_robot.yaml` | block 逻辑失效禁止测试 | 风险态命令被拦截 |
| T07 | 健康检查设备化 | R03 R07 R08 | 设备状态源 | 健康报告增强 | `src/r3/health.py` | health 报告不可诊断不得推进 | 可定位问题并给出原因 |
| T08 | profile 切换与回退脚本 | R10 | runtime 配置 | 切换工具 | `configs/runtime_*` `scripts/switch_executor_profile.py` | 无回退能力禁止长测 | 30秒内可回 sim |
| T09 | 实机验收脚本与报告 | R09 | 验收项清单 | gates 报告 | `scripts/run_real_robot_gates.py` | 报告缺安全项不得结束 | 四类检查均可自动输出 |
| T10 | 全链路回归与归档 | R09 R10 | 命令集+日志 | 归档报告 | `reports/*` | 回放不一致不得收口 | 命令-执行-日志一致 |

---

## 8. 完成定义（DoD）

以下条件全部满足，才可判定实机迁移完成：

1. `/r3/command` 与 `/r3/*` 在实机链路稳定可用。  
2. `arm` 与 `gripper` 指令均可执行，且状态回传可靠。  
3. 急停/保护停/远端确认在软件层与设备层联动一致。  
4. `/r3/health` 报告可识别并解释设备级故障。  
5. JSONL 与 rosbag 回放一致，`command_id` 无断链。  
6. 回退脚本可随时将执行器切回仿真。  
7. 实机 gates 与 E2E 报告通过并归档。
