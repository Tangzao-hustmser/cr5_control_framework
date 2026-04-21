import argparse
import os
import sys

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if root not in sys.path:
    sys.path.insert(0, root)

from src.r3.replay import ReplayEngine


def main():
    parser = argparse.ArgumentParser(description="Replay R3 logs.")
    parser.add_argument("--log-dir", required=True, help="Session log directory containing *.jsonl files")
    parser.add_argument("--headless", action="store_true", help="Replay in no-render mode")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier")
    args = parser.parse_args()

    engine = ReplayEngine(args.log_dir)
    result = engine.replay(callback=None, headless=args.headless, speed=args.speed)
    print("Replay finished")
    print(result.to_dict())


if __name__ == "__main__":
    main()
