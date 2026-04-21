# Ark Ranger CR5 R3 Hardening Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Deliver a unified, safe, observable command-to-execution pipeline covering R3-01 through R3-16 for Ark_Ranger_CR5.

**Architecture:** Build a new `src/r3` pipeline layer that sits between all command adapters and the existing control bridge. Every command is parsed, validated, normalized, safety-filtered, executed, and fully logged with deterministic fault codes. Add startup checks, replay, benchmark, and test gate scripts so each change produces measurable quality and safety evidence.

**Tech Stack:** Python 3, Isaac Sim runtime APIs, YAML/JSON, unittest/pytest-compatible tests.

---

### Task 1: Protocol + Fault/Event Foundation

**Files:**
- Create: `Ark_Ranger_CR5/docs/protocol/unified_action_protocol_v1.md`
- Create: `Ark_Ranger_CR5/src/r3/errors.py`
- Create: `Ark_Ranger_CR5/src/r3/events.py`
- Create: `Ark_Ranger_CR5/src/r3/protocol.py`
- Test: `Ark_Ranger_CR5/tests/test_protocol_validator.py`

**Step 1:** Write tests for valid/invalid protocol messages and stable fault codes.
**Step 2:** Run tests and confirm failures.
**Step 3:** Implement command schema dataclasses, dictionaries, and fault/event enums.
**Step 4:** Re-run tests and confirm pass.

### Task 2: Validation + Normalization + Adapter Abstraction

**Files:**
- Create: `Ark_Ranger_CR5/src/r3/validator.py`
- Create: `Ark_Ranger_CR5/src/r3/adapters.py`
- Modify: `Ark_Ranger_CR5/scripts/run_simulation.py`
- Modify: `Ark_Ranger_CR5/src/isaac_node.py`
- Test: `Ark_Ranger_CR5/tests/test_protocol_validator.py`

**Step 1:** Add tests for field/dimension/range/unit checks and canonical normalized object output.
**Step 2:** Implement validator + normalizer and adapter interface with local and ROS2 adapters.
**Step 3:** Wire all runtime command ingress through adapters + validator.
**Step 4:** Re-run targeted tests.

### Task 3: Health Check + Asset Consistency + Startup Decision

**Files:**
- Create: `Ark_Ranger_CR5/src/r3/health.py`
- Create: `Ark_Ranger_CR5/src/r3/asset_check.py`
- Create: `Ark_Ranger_CR5/scripts/run_asset_check.py`
- Modify: `Ark_Ranger_CR5/scripts/run_simulation.py`
- Test: `Ark_Ranger_CR5/tests/test_asset_checker.py`

**Step 1:** Add tests for missing dependency/config and mismatch-detection behavior.
**Step 2:** Implement dependency + config + asset check reporting with NORMAL/DEGRADED/BLOCKED.
**Step 3:** Enforce startup block for critical mismatches.
**Step 4:** Re-run tests.

### Task 4: Safety Filter + IK Fallback + Gripper Closed Loop

**Files:**
- Create: `Ark_Ranger_CR5/src/r3/safety.py`
- Create: `Ark_Ranger_CR5/src/r3/ik_policy.py`
- Create: `Ark_Ranger_CR5/src/r3/gripper.py`
- Modify: `Ark_Ranger_CR5/src/bridge/ark_bridge.py`
- Modify: `Ark_Ranger_CR5/src/isaac_node.py`
- Test: `Ark_Ranger_CR5/tests/test_safety_filter.py`
- Test: `Ark_Ranger_CR5/tests/test_ik_policy.py`

**Step 1:** Add tests for pass/clamp/block decisions and IK state transitions.
**Step 2:** Implement filters and fallback manager with reason codes.
**Step 3:** Integrate gripper open/close/hold/position loop and jam detection.
**Step 4:** Re-run tests.

### Task 5: Structured Logs + Timeline + Replay + Benchmark

**Files:**
- Create: `Ark_Ranger_CR5/src/r3/structured_logging.py`
- Create: `Ark_Ranger_CR5/src/r3/timeline.py`
- Create: `Ark_Ranger_CR5/src/r3/replay.py`
- Create: `Ark_Ranger_CR5/src/r3/benchmark.py`
- Create: `Ark_Ranger_CR5/scripts/replay_log.py`
- Create: `Ark_Ranger_CR5/scripts/run_benchmark.py`
- Test: `Ark_Ranger_CR5/tests/test_replay_benchmark.py`

**Step 1:** Add tests for command_id-linked logs, index lookup, replay determinism helpers, and metric calculations.
**Step 2:** Implement JSONL logger + index manager + replay and benchmark reporting.
**Step 3:** Re-run tests.

### Task 6: Test Gate + Runbook + Final Acceptance

**Files:**
- Create: `Ark_Ranger_CR5/scripts/run_gates.py`
- Create: `Ark_Ranger_CR5/docs/runbook.md`
- Create: `Ark_Ranger_CR5/docs/fault_codes.md`
- Create: `Ark_Ranger_CR5/docs/acceptance/final_integration_acceptance.md`
- Modify: `Ark_Ranger_CR5/configs/robot_config.yaml`

**Step 1:** Add gate script thresholds for unit/integration checks.
**Step 2:** Document startup, adapter switching, replay, and troubleshooting.
**Step 3:** Add final acceptance checklist mapping R3-01~R3-16.
**Step 4:** Run gate script and capture outcome.
