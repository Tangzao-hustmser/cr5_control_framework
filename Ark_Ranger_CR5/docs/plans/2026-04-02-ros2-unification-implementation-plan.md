# ROS2-Only Unification Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace all runtime command ingress with ROS2 Action/Service/Topic while preserving the R3 validation/safety/execution/logging pipeline and keeping JSON/LCM as explicit compatibility bridges.

**Architecture:** Isaac Sim (Windows) hosts the ROS2 Action Server `/r3/command`, system Services, and event/status/validation Topics; WSL2 ROS2 Humble clients communicate via DDS. `ArkIsaacSimNode` exposes `handle_command()` and publishes observable data; compatibility adapters/LCM are disabled by default.

**Tech Stack:** Python 3, rclpy (ROS2 Humble), ROS2 interfaces (ament), Isaac Sim, YAML/JSON.

---

### Task 1: Define ROS2 Interfaces (`r3_msgs`)

**Files:**
- Create: `ros2_ws/src/r3_msgs/package.xml`
- Create: `ros2_ws/src/r3_msgs/CMakeLists.txt`
- Create: `ros2_ws/src/r3_msgs/msg/Command.msg`
- Create: `ros2_ws/src/r3_msgs/msg/CommandResult.msg`
- Create: `ros2_ws/src/r3_msgs/msg/ValidationIssue.msg`
- Create: `ros2_ws/src/r3_msgs/msg/SafetyDecision.msg`
- Create: `ros2_ws/src/r3_msgs/msg/RuntimeStatus.msg`
- Create: `ros2_ws/src/r3_msgs/msg/SystemEvent.msg`
- Create: `ros2_ws/src/r3_msgs/srv/SystemControl.srv`
- Create: `ros2_ws/src/r3_msgs/srv/HealthCheck.srv`
- Create: `ros2_ws/src/r3_msgs/srv/AssetCheck.srv`
- Create: `ros2_ws/src/r3_msgs/action/Command.action`

**Step 1: Write interface definitions**

`ros2_ws/src/r3_msgs/msg/Command.msg`
```text
string protocol_version
string command_id
int64 timestamp_ms
string source
string command_type
string frame
float64[] position_m
float64[] orientation_wxyz
float64[] joint_positions_rad
string[] joint_names
string gripper_mode
float64 gripper_position_rad
float64 gripper_speed_ratio
float64 gripper_force_ratio
string system_operation
string system_reason
string metadata_json
```

`ros2_ws/src/r3_msgs/msg/CommandResult.msg`
```text
bool ok
string fault_code
string message
string normalized_json
r3_msgs/ValidationIssue[] issues
```

`ros2_ws/src/r3_msgs/msg/ValidationIssue.msg`
```text
string fault_code
string field
string message
```

`ros2_ws/src/r3_msgs/msg/SafetyDecision.msg`
```text
string decision
string[] reason_codes
string details_json
```

`ros2_ws/src/r3_msgs/msg/RuntimeStatus.msg`
```text
int64 timestamp_ms
int32 step_idx
string command_id
float64[] ee_xyz
float64[] arm_rad
float64 gripper_rad
```

`ros2_ws/src/r3_msgs/msg/SystemEvent.msg`
```text
string event_type
string command_id
string fault_code
string reason
string details_json
int64 timestamp_ms
```

`ros2_ws/src/r3_msgs/srv/SystemControl.srv`
```text
string command_id
int64 timestamp_ms
string source
string operation
string reason
---
bool ok
string fault_code
string message
```

`ros2_ws/src/r3_msgs/srv/HealthCheck.srv`
```text
bool include_details
---
string status
string report_json
```

`ros2_ws/src/r3_msgs/srv/AssetCheck.srv`
```text
bool include_details
---
string status
string report_json
```

`ros2_ws/src/r3_msgs/action/Command.action`
```text
r3_msgs/Command command
---
bool ok
string fault_code
string message
string normalized_json
r3_msgs/ValidationIssue[] issues
---
string command_id
string stage
r3_msgs/SafetyDecision safety
r3_msgs/RuntimeStatus status
string info
```

**Step 2: Add package.xml + CMakeLists.txt**

`ros2_ws/src/r3_msgs/package.xml`
```xml
<?xml version="1.0"?>
<package format="3">
  <name>r3_msgs</name>
  <version>0.1.0</version>
  <description>ROS2 interfaces for Ark Ranger CR5 R3 protocol.</description>
  <maintainer email="dev@example.com">Ark Ranger CR5</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <depend>builtin_interfaces</depend>
  <depend>std_msgs</depend>
</package>
```

`ros2_ws/src/r3_msgs/CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 3.5)
project(r3_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Command.msg"
  "msg/CommandResult.msg"
  "msg/ValidationIssue.msg"
  "msg/SafetyDecision.msg"
  "msg/RuntimeStatus.msg"
  "msg/SystemEvent.msg"
)

set(srv_files
  "srv/SystemControl.srv"
  "srv/HealthCheck.srv"
  "srv/AssetCheck.srv"
)

set(action_files
  "action/Command.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
)

ament_package()
```

**Step 3: Build interfaces (verification)**

Run (WSL or Windows ROS2 env):
`colcon build --packages-select r3_msgs`

Expected: build completes without errors.

**Step 4: Commit**

```bash
git add ros2_ws/src/r3_msgs
git commit -m "feat: add r3_msgs ROS2 interfaces"
```

---

### Task 2: ROS2 Message Utilities + Publishers

**Files:**
- Create: `src/ros2/__init__.py`
- Create: `src/ros2/r3_msg_utils.py`
- Create: `src/ros2/r3_publishers.py`
- Test: `tests/test_ros2_msg_utils.py`

**Step 1: Write failing test**

`tests/test_ros2_msg_utils.py`
```python
def test_command_msg_roundtrip_defaults():
    raw = {
        "command_type": "pose",
        "payload": {"frame": "world", "position_m": [0.1, 0.2, 0.3], "orientation_wxyz": [1, 0, 0, 0]},
    }
    msg = command_dict_to_msg(raw)
    out = command_msg_to_dict(msg, default_source="ros2")
    assert out["command_type"] == "pose"
    assert out["payload"]["position_m"] == [0.1, 0.2, 0.3]
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_ros2_msg_utils.py -v`
Expected: FAIL with "NameError: command_dict_to_msg not defined"

**Step 3: Implement utilities**

`src/ros2/r3_msg_utils.py`
```python
import json
import time
from typing import Any, Dict

from r3_msgs.msg import Command, ValidationIssue, SafetyDecision, RuntimeStatus
from src.r3.protocol import PROTOCOL_VERSION, generate_command_id


def _now_ms() -> int:
    return int(time.time() * 1000)


def _safe_json_load(text: str) -> Dict[str, Any]:
    if not text:
        return {}
    try:
        data = json.loads(text)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def command_dict_to_msg(raw: Dict[str, Any]) -> Command:
    msg = Command()
    msg.protocol_version = str(raw.get("protocol_version") or PROTOCOL_VERSION)
    msg.command_id = str(raw.get("command_id") or generate_command_id())
    msg.timestamp_ms = int(raw.get("timestamp_ms") or _now_ms())
    msg.source = str(raw.get("source") or "ros2")
    msg.command_type = str(raw.get("command_type") or "")
    payload = raw.get("payload") or {}
    msg.frame = str(payload.get("frame") or "")
    msg.position_m = [float(v) for v in payload.get("position_m", [])]
    msg.orientation_wxyz = [float(v) for v in payload.get("orientation_wxyz", [])]
    msg.joint_positions_rad = [float(v) for v in payload.get("joint_positions_rad", [])]
    msg.joint_names = [str(v) for v in payload.get("joint_names", [])]
    msg.gripper_mode = str(payload.get("mode") or "")
    msg.gripper_position_rad = float(payload.get("position_rad") or 0.0)
    msg.gripper_speed_ratio = float(payload.get("speed_ratio") or 0.0)
    msg.gripper_force_ratio = float(payload.get("force_ratio") or 0.0)
    msg.system_operation = str(payload.get("operation") or "")
    msg.system_reason = str(payload.get("reason") or "")
    msg.metadata_json = json.dumps(raw.get("metadata") or {}, ensure_ascii=False)
    return msg


def command_msg_to_dict(msg: Command, default_source: str = "ros2") -> Dict[str, Any]:
    payload = {}
    if msg.command_type == "pose":
        if msg.frame:
            payload["frame"] = msg.frame
        if msg.position_m:
            payload["position_m"] = list(msg.position_m)
        if msg.orientation_wxyz:
            payload["orientation_wxyz"] = list(msg.orientation_wxyz)
    elif msg.command_type == "arm":
        payload["joint_positions_rad"] = list(msg.joint_positions_rad)
        if msg.joint_names:
            payload["joint_names"] = list(msg.joint_names)
    elif msg.command_type == "gripper":
        payload["mode"] = msg.gripper_mode
        if msg.gripper_position_rad:
            payload["position_rad"] = float(msg.gripper_position_rad)
        if msg.gripper_speed_ratio:
            payload["speed_ratio"] = float(msg.gripper_speed_ratio)
        if msg.gripper_force_ratio:
            payload["force_ratio"] = float(msg.gripper_force_ratio)
    elif msg.command_type == "system":
        payload["operation"] = msg.system_operation
        if msg.system_reason:
            payload["reason"] = msg.system_reason

    return {
        "protocol_version": msg.protocol_version or PROTOCOL_VERSION,
        "command_id": msg.command_id or generate_command_id(),
        "timestamp_ms": int(msg.timestamp_ms or _now_ms()),
        "source": msg.source or default_source,
        "command_type": msg.command_type,
        "payload": payload,
        "metadata": _safe_json_load(msg.metadata_json),
    }
```

**Step 4: Run test to verify it passes**

Run: `python -m pytest tests/test_ros2_msg_utils.py -v`
Expected: PASS

**Step 5: Implement publishers**

`src/ros2/r3_publishers.py`
```python
import json
from typing import Any, Dict, List, Optional

from r3_msgs.msg import RuntimeStatus, SystemEvent, SafetyDecision, CommandResult, ValidationIssue


class R3Ros2Publishers:
    def __init__(self, node):
        self._events_pub = node.create_publisher(SystemEvent, "/r3/events", 20)
        self._status_pub = node.create_publisher(RuntimeStatus, "/r3/status", 20)
        self._validation_pub = node.create_publisher(CommandResult, "/r3/validation", 20)

    def publish_event(self, event) -> None:
        msg = SystemEvent()
        msg.event_type = event.event_type
        msg.command_id = event.command_id
        msg.fault_code = event.fault_code
        msg.reason = event.reason
        msg.details_json = json.dumps(event.details, ensure_ascii=False)
        msg.timestamp_ms = int(event.timestamp_ms)
        self._events_pub.publish(msg)

    def publish_status(self, status: Dict[str, Any]) -> None:
        msg = RuntimeStatus()
        msg.timestamp_ms = int(status.get("timestamp_ms", 0))
        msg.step_idx = int(status.get("step_idx", 0))
        msg.command_id = str(status.get("command_id", ""))
        msg.ee_xyz = [float(v) for v in status.get("ee_xyz", [])]
        msg.arm_rad = [float(v) for v in status.get("arm_rad", [])]
        msg.gripper_rad = float(status.get("gripper_rad", 0.0))
        self._status_pub.publish(msg)

    def publish_validation(self, command_id: str, ok: bool, issues: List[Dict[str, Any]], normalized: Optional[Dict[str, Any]]):
        msg = CommandResult()
        msg.ok = bool(ok)
        msg.fault_code = str(issues[0].get("fault_code", "")) if issues else ""
        msg.message = "validation"
        msg.normalized_json = json.dumps(normalized or {}, ensure_ascii=False)
        msg.issues = [ValidationIssue(fault_code=i["fault_code"], field=i["field"], message=i["message"]) for i in issues]
        self._validation_pub.publish(msg)
```

**Step 6: Commit**

```bash
git add src/ros2 tests/test_ros2_msg_utils.py
git commit -m "feat: add ROS2 msg utils and publishers"
```

---

### Task 3: ROS2 Action Server + Services

**Files:**
- Create: `src/ros2/r3_action_server.py`
- Create: `src/ros2/r3_services.py`
- Create: `src/ros2/r3_runtime.py`

**Step 1: Implement Action Server**

`src/ros2/r3_action_server.py`
```python
from rclpy.action import ActionServer
from r3_msgs.action import Command
from r3_msgs.msg import ValidationIssue, SafetyDecision, RuntimeStatus
from src.ros2.r3_msg_utils import command_msg_to_dict


class R3ActionServer:
    def __init__(self, node, isaac_node):
        self._node = node
        self._isaac = isaac_node
        self._server = ActionServer(
            node,
            Command,
            "/r3/command",
            execute_callback=self.execute_callback,
        )

    async def execute_callback(self, goal_handle):
        raw = command_msg_to_dict(goal_handle.request.command, default_source="ros2")
        result = self._isaac.handle_command(raw, source="ros2")

        feedback = Command.Feedback()
        feedback.command_id = result.command_id
        feedback.stage = result.stage
        feedback.info = result.message
        feedback.safety = SafetyDecision(
            decision=result.safety_decision or "",
            reason_codes=result.safety_reason_codes or [],
            details_json=result.safety_details_json or "{}",
        )
        if result.status_snapshot:
            status = result.status_snapshot
            feedback.status = RuntimeStatus(
                timestamp_ms=status.get("timestamp_ms", 0),
                step_idx=status.get("step_idx", 0),
                command_id=status.get("command_id", ""),
                ee_xyz=status.get("ee_xyz", []),
                arm_rad=status.get("arm_rad", []),
                gripper_rad=status.get("gripper_rad", 0.0),
            )
        goal_handle.publish_feedback(feedback)

        action_result = Command.Result()
        action_result.ok = result.ok
        action_result.fault_code = result.fault_code
        action_result.message = result.message
        action_result.normalized_json = result.normalized_json
        action_result.issues = [ValidationIssue(fault_code=i["fault_code"], field=i["field"], message=i["message"]) for i in result.issues]

        if result.ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return action_result
```

**Step 2: Implement Services**

`src/ros2/r3_services.py`
```python
import json
import time
from r3_msgs.srv import SystemControl, HealthCheck, AssetCheck
from src.r3.health import run_startup_health_check
from src.r3.asset_check import run_asset_consistency_check


class R3Services:
    def __init__(self, node, isaac_node, project_root, robot_cfg, runtime_cfg):
        self._node = node
        self._isaac = isaac_node
        self._project_root = project_root
        self._robot_cfg = robot_cfg
        self._runtime_cfg = runtime_cfg

        node.create_service(SystemControl, "/r3/pause", self._make_system_cb("pause"))
        node.create_service(SystemControl, "/r3/resume", self._make_system_cb("resume"))
        node.create_service(SystemControl, "/r3/reset", self._make_system_cb("reset"))
        node.create_service(SystemControl, "/r3/stop", self._make_system_cb("stop"))
        node.create_service(HealthCheck, "/r3/health", self._health_cb)
        node.create_service(AssetCheck, "/r3/asset_check", self._asset_cb)

    def _make_system_cb(self, op: str):
        def _cb(request, response):
            raw = {
                "protocol_version": "r3.v1",
                "command_id": request.command_id or f"cmd-sys-{int(time.time()*1000)}",
                "timestamp_ms": int(request.timestamp_ms or time.time() * 1000),
                "source": request.source or "ros2",
                "command_type": "system",
                "payload": {"operation": op, "reason": request.reason},
            }
            result = self._isaac.handle_command(raw, source="ros2")
            response.ok = result.ok
            response.fault_code = result.fault_code
            response.message = result.message
            return response
        return _cb

    def _health_cb(self, request, response):
        report = run_startup_health_check(self._robot_cfg, self._runtime_cfg)
        response.status = report.status
        response.report_json = json.dumps(report.to_dict(), ensure_ascii=False)
        return response

    def _asset_cb(self, request, response):
        report = run_asset_consistency_check(
            project_root=self._project_root,
            robot_config_path=self._robot_cfg["_path"],
            kinematics_config_path=f"{self._project_root}/configs/kinematics_config.yaml",
            report_output_path=f"{self._project_root}/reports/asset_consistency_report.json",
        )
        response.status = report.get("summary", {}).get("status", "UNKNOWN")
        response.report_json = json.dumps(report, ensure_ascii=False)
        return response
```

**Step 3: Implement runtime wrapper**

`src/ros2/r3_runtime.py`
```python
import rclpy
from rclpy.executors import SingleThreadedExecutor

from src.ros2.r3_action_server import R3ActionServer
from src.ros2.r3_publishers import R3Ros2Publishers
from src.ros2.r3_services import R3Services


class R3Ros2Runtime:
    def __init__(self, isaac_node, project_root, robot_cfg, runtime_cfg):
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("r3_isaacsim")
        self.publishers = R3Ros2Publishers(self._node)
        isaac_node.attach_ros2_publishers(self.publishers)
        self._action_server = R3ActionServer(self._node, isaac_node)
        self._services = R3Services(self._node, isaac_node, project_root, robot_cfg, runtime_cfg)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

    def spin_once(self):
        self._executor.spin_once(timeout_sec=0.0)

    def shutdown(self):
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
```

**Step 4: Commit**

```bash
git add src/ros2/r3_action_server.py src/ros2/r3_services.py src/ros2/r3_runtime.py
git commit -m "feat: add ROS2 action server and services"
```

---

### Task 4: Refactor `ArkIsaacSimNode` Entry Point

**Files:**
- Modify: `src/isaac_node.py`

**Step 1: Add command handler structure**

Add a result dataclass and a public handler that reuses existing validation/safety:
```python
@dataclass
class CommandHandleResult:
    ok: bool
    command_id: str
    stage: str
    fault_code: str
    message: str
    issues: List[Dict[str, Any]]
    normalized_json: str
    safety_decision: str
    safety_reason_codes: List[str]
    safety_details_json: str
    status_snapshot: Optional[Dict[str, Any]]
```

**Step 2: Modify `_process_message` to call `handle_command`**

```python
result = self.handle_command(raw, source=message.source)
```

**Step 3: Add ROS2 publisher hook methods**

```python
def attach_ros2_publishers(self, publishers):
    self.ros2_publishers = publishers
```

Call `publish_validation`, `publish_event`, and `publish_status` at the same places where logs are written.

**Step 4: Commit**

```bash
git add src/isaac_node.py
git commit -m "refactor: add handle_command entrypoint"
```

---

### Task 5: Compatibility Adapters + JSON→ROS2 Bridge

**Files:**
- Modify: `src/r3/adapters.py`
- Create: `src/ros2/r3_json_bridge.py`
- Create: `scripts/run_json_bridge.py`

**Step 1: Update adapters to be opt-in**

```python
compat = runtime_cfg.get("compat", {}).get("enable_adapters", False)
if not compat:
    return []
```

**Step 2: Implement JSON bridge**

`src/ros2/r3_json_bridge.py`
```python
import time
import rclpy
from rclpy.action import ActionClient
from r3_msgs.action import Command
from src.r3.adapters import LocalInputAdapter
from src.ros2.r3_msg_utils import command_dict_to_msg


def main():
    rclpy.init()
    node = rclpy.create_node("r3_json_bridge")
    client = ActionClient(node, Command, "/r3/command")
    adapter = LocalInputAdapter(prompt="r3-json> ", show_help=True)
    adapter.start()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        for msg in adapter.poll():
            raw = msg.raw
            if isinstance(raw, dict) and "__adapter_error__" in raw:
                print(raw["__adapter_error__"].get("message"))
                continue
            goal = Command.Goal(command=command_dict_to_msg(raw))
            client.send_goal_async(goal)
        time.sleep(0.01)
```

**Step 3: Add launcher script**

`scripts/run_json_bridge.py`
```python
from src.ros2.r3_json_bridge import main

if __name__ == "__main__":
    main()
```

**Step 4: Commit**

```bash
git add src/r3/adapters.py src/ros2/r3_json_bridge.py scripts/run_json_bridge.py
git commit -m "feat: add JSON to ROS2 compatibility bridge"
```

---

### Task 6: Runtime Entry + Config

**Files:**
- Modify: `scripts/run_simulation.py`
- Modify: `configs/runtime_config.yaml`
- Modify: `src/r3/health.py`

**Step 1: Set ROS2-only defaults**

`configs/runtime_config.yaml`
```yaml
runtime:
  input_source: "ros2_action"
  mode: "ros2_only"
  compat:
    enable_adapters: false
    enable_ark_lcm: false
```

**Step 2: Wire ROS2 runtime**

`run_simulation.py` creates `R3Ros2Runtime` after `ArkIsaacSimNode` and calls `ros2_runtime.spin_once()` each frame; shutdown on exit.

**Step 3: Update health checks**

`health.py` accepts `input_source == "ros2_action"` and verifies `rclpy` availability.

**Step 4: Commit**

```bash
git add scripts/run_simulation.py configs/runtime_config.yaml src/r3/health.py
git commit -m "feat: enable ros2-only runtime mode"
```

---

### Task 7: LCM Compatibility Guard

**Files:**
- Modify: `ark_patch.py`
- Modify: `run_registry.py`

**Step 1: Add explicit enable flag**

`ark_patch.py` should only inject paths/mocks when `ARK_ENABLE_COMPAT=1`; otherwise provide a no-op Node.

`run_registry.py` should exit unless `ARK_ENABLE_COMPAT=1` is set.

**Step 2: Commit**

```bash
git add ark_patch.py run_registry.py
git commit -m "chore: guard legacy LCM compatibility"
```

---

### Task 8: E2E ROS2 Path + Docs

**Files:**
- Create: `scripts/run_ros2_e2e.py`
- Modify: `scripts/run_gates.py`
- Modify: `docs/simulation_guide_zh.md`

**Step 1: Add ROS2 E2E script**

`run_ros2_e2e.py` sends a sample Action goal, waits for result, then runs replay+benchmark with current session logs.

**Step 2: Gate integration**

`run_gates.py` runs ROS2 E2E if `R3_E2E_ROS2=1` is set.

**Step 3: Update guide**

Add WSL2 bridge instructions, ROS2 Action/Service usage, and JSON bridge as compat.

**Step 4: Commit**

```bash
git add scripts/run_ros2_e2e.py scripts/run_gates.py docs/simulation_guide_zh.md
git commit -m "docs: ROS2-only workflow and E2E entry"
```

