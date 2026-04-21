import os
import subprocess
import sys


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def run(cmd):
    print(f"[Gate] Run: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=ROOT)
    if result.returncode != 0:
        raise SystemExit(result.returncode)


def main():
    run([sys.executable, "scripts/run_asset_check.py"])
    run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py", "-v"])
    print("[Gate] PASS")


if __name__ == "__main__":
    main()
