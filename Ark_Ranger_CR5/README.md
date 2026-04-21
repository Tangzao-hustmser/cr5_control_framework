# Ark Ranger CR5 (Isaac Sim)

Isaac Sim + ROS2-based simulation/control project for Ark Ranger CR5 + AG95.

## What this repo contains

- Unified command protocol (`R3`) parsing/validation/safety/runtime.
- Isaac execution bridge and simulation entry scripts.
- ROS2 message package (`ros2_ws/src/r3_msgs`) and bridge tools.
- Tests, runbook, protocol docs, migration docs.

## Directory layout

- `src/`: core runtime, protocol, safety, ROS2 runtime, Isaac bridge
- `scripts/`: run scripts, gate scripts, benchmark/replay, bridge tools
- `configs/`: runtime/robot/kinematics and benchmark configs
- `ros2_ws/src/r3_msgs/`: ROS2 interfaces (`msg/srv/action`)
- `models/`: robot/scene model assets
- `tests/`: unit/integration tests
- `docs/`: runbook, protocol, plans

## Prerequisites

- Isaac Sim standalone (Windows), use Isaac-provided Python (`..\\python.bat`)
- Optional ROS2 Humble (recommended in WSL Ubuntu 22.04)

## Quick start

```powershell
cd Ark_Ranger_CR5
..\python.bat scripts\run_simulation.py
```

Run baseline gates:

```powershell
..\python.bat scripts\run_gates.py
```

## Open-source publishing notes

- This repo should be published from `Ark_Ranger_CR5` only. Do not publish the whole Isaac Sim installation directory.
- Generated artifacts are excluded by `.gitignore` (for example `reports/`, `ros2_ws/build|install|log`).
- Vendor manuals under `docs/Robot_Product_Document/` are ignored by default; only publish if redistribution is allowed.
- `ark_framework` and `ark_types` currently contain embedded `.git` metadata. Before first commit, choose one:
  - Vendor mode: remove nested `.git` directories and commit source directly.
  - Submodule mode: convert them to formal Git submodules.

For a practical release checklist, see `OPEN_SOURCE_CHECKLIST_zh.md`.
