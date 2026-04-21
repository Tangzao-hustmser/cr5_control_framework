import argparse
import json
import os
import time


def _fmt(value: float, precision: int = 3) -> str:
    return f"{float(value):.{precision}f}"


def _render_status(data):
    step_idx = data.get("step_idx", -1)
    command_id = data.get("command_id", "")
    xyz = data.get("ee_xyz", [0.0, 0.0, 0.0])
    arm = data.get("arm_rad", [])
    grip = data.get("gripper_rad", 0.0)

    arm_str = "[" + ", ".join(_fmt(v, 4) for v in arm) + "]"
    return (
        f"[LiveStatus][step={int(step_idx):06d}] cmd={command_id} "
        f"EE xyz=({_fmt(xyz[0])}, {_fmt(xyz[1])}, {_fmt(xyz[2])}) "
        f"| arm={arm_str} | grip={_fmt(grip, 4)}"
    )


def main():
    parser = argparse.ArgumentParser(description="Watch live robot status exported by run_simulation.")
    parser.add_argument(
        "--file",
        default=os.path.join("reports", "live_status", "current_status.json"),
        help="Path to live status json file",
    )
    parser.add_argument("--interval", type=float, default=0.5, help="Polling interval in seconds")
    parser.add_argument("--once", action="store_true", help="Print once when first status appears and exit")
    args = parser.parse_args()

    target = os.path.abspath(args.file)
    print(f"[Watcher] Waiting for live status file: {target}")

    last_signature = None
    try:
        while True:
            if os.path.exists(target):
                try:
                    with open(target, "r", encoding="utf-8") as fh:
                        data = json.load(fh)
                    signature = (
                        int(data.get("timestamp_ms", 0)),
                        int(data.get("step_idx", -1)),
                        str(data.get("command_id", "")),
                    )
                    if signature != last_signature:
                        print(_render_status(data))
                        last_signature = signature
                        if args.once:
                            break
                except Exception:
                    pass
            time.sleep(max(0.1, float(args.interval)))
    except KeyboardInterrupt:
        print("\n[Watcher] Stopped.")


if __name__ == "__main__":
    main()
