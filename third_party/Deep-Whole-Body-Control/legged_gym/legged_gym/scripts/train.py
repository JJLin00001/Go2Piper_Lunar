"""Isaac Sim 5.1 entry point; original Gym script is in legacy_isaacgym/."""
from pathlib import Path
import runpy
import sys
if __name__ == "__main__":
    target = Path(__file__).resolve().parents[3] / "isaaclab/launch.py"
    sys.argv[1:1] = ["--mode", "train"]
    runpy.run_path(str(target), run_name="__main__")
