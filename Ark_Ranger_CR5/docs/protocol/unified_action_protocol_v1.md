# Unified Action Protocol v1 (r3.v1)

This protocol is the only accepted command format for all command entries.

## 1. Command Envelope

Required top-level fields:
- `protocol_version` (string): must equal `r3.v1`
- `command_id` (string): unique command id; if absent at source layer, it is auto-generated before validation
- `timestamp_ms` (number): unix epoch in milliseconds
- `source` (string): command source (`local_terminal`, `ros2`, etc.)
- `command_type` (string): one of `pose | arm | gripper | system`
- `payload` (object): command body

Optional top-level fields:
- `metadata` (object)

Unknown fields are rejected with stable fault code `R3-E1102`.

## 2. Payload Dictionary

### `command_type: pose`
- `frame` (string, optional, default `world`)
- `position_m` (list[3] number): `[x, y, z]` in meters
- `orientation_wxyz` (list[4] number): quaternion `[w, x, y, z]`

### `command_type: arm`
- `joint_positions_rad` (list[N] number): N must equal configured arm joint count
- `joint_names` (list[N] string, optional): if present must match configured joint order exactly

### `command_type: gripper`
- `mode` (string): `open | close | hold | position`
- `position_rad` (number): required only when mode is `position`
- `speed_ratio` (number, optional)
- `force_ratio` (number, optional)

### `command_type: system`
- `operation` (string): `pause | resume | reset | stop`
- `reason` (string, optional)

## 3. Mode Semantics

- `pose`: request end-effector spatial target; solver produces arm joint target
- `arm`: request arm joints directly
- `gripper`: request gripper state machine (`open/close/hold/position`)
- `system`: runtime control operation

## 4. Error/Fault Code Dictionary (Stable)

- `R3-E1001` invalid JSON
- `R3-E1002` invalid message type (not object)
- `R3-E1003` unsupported protocol version
- `R3-E1101` missing required field
- `R3-E1102` unknown/illegal field
- `R3-E1103` field type error
- `R3-E1201` dimension mismatch
- `R3-E1202` value out of range
- `R3-E1203` invalid unit field
- `R3-E1204` invalid coordinate frame
- `R3-E1301` unsupported command type
- `R3-E1401` blocked by safety filter
- `R3-E1402` clamped by safety filter
- `R3-E1501` IK solve failed (retry/hold policy active)
- `R3-E1502` IK retry exceeded (fallback mode)
- `R3-E1601` gripper stall detected
- `R3-E1602` gripper target out of range
- `R3-E1701` adapter unavailable
- `R3-E1702` ROS2 dependency missing
- `R3-E1801` asset consistency failed
- `R3-E1901` startup blocked
- `R3-E1902` startup degraded
- `R3-E1999` unknown internal error

## 5. Example Messages

Pose:
```json
{
  "protocol_version": "r3.v1",
  "command_id": "cmd-001",
  "timestamp_ms": 1710000000000,
  "source": "local_terminal",
  "command_type": "pose",
  "payload": {
    "frame": "world",
    "position_m": [0.45, 0.10, 0.55],
    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]
  },
  "metadata": {"operator": "demo"}
}
```

Arm:
```json
{
  "protocol_version": "r3.v1",
  "command_id": "cmd-002",
  "timestamp_ms": 1710000000100,
  "source": "ros2",
  "command_type": "arm",
  "payload": {
    "joint_positions_rad": [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]
  }
}
```

Gripper:
```json
{
  "protocol_version": "r3.v1",
  "command_id": "cmd-003",
  "timestamp_ms": 1710000000200,
  "source": "local_terminal",
  "command_type": "gripper",
  "payload": {
    "mode": "position",
    "position_rad": 0.25,
    "speed_ratio": 0.8
  }
}
```

System:
```json
{
  "protocol_version": "r3.v1",
  "command_id": "cmd-004",
  "timestamp_ms": 1710000000300,
  "source": "local_terminal",
  "command_type": "system",
  "payload": {
    "operation": "pause",
    "reason": "manual safety pause"
  }
}
```
