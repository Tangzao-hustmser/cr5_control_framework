# Fault Codes and Events

## Fault Codes

See `docs/protocol/unified_action_protocol_v1.md` for full dictionary.

Primary families:
- `R3-E10xx`: parsing/protocol envelope
- `R3-E11xx`: schema fields
- `R3-E12xx`: dimensions/ranges/frames
- `R3-E14xx`: safety filtering
- `R3-E15xx`: IK fallback
- `R3-E16xx`: gripper control
- `R3-E17xx`: adapter/dependency
- `R3-E18xx`: asset consistency
- `R3-E19xx`: startup and unknown internal

## Event Types

- `command.received`
- `command.rejected`
- `command.normalized`
- `safety.pass`
- `safety.clamp`
- `safety.block`
- `ik.success`
- `ik.failure`
- `ik.fallback`
- `gripper.stall`
- `control.applied`
- `system.state_change`
- `startup.report`

## Event Publishing Interface

Events are published via `src/r3/events.py` `EventPublisher` and forwarded to:
- Structured event log (`events.jsonl`)
- Timeline index linker (`timeline_index.jsonl`)

Every event includes:
- `event_type`
- `command_id`
- `fault_code`
- `reason`
- `details`
- `timestamp_ms`
