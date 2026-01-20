import json
import os
import argparse
from pathlib import Path

import matplotlib.pyplot as plt

# pyplot으로 map log 간단히 시각화

def load_json(p: Path):
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)


def find_latest_run(base_dir: Path) -> Path:
    runs = [d for d in base_dir.iterdir() if d.is_dir()]
    if not runs:
        raise FileNotFoundError(f"No run folders in {base_dir}")
    runs.sort(key=lambda d: d.stat().st_mtime, reverse=True)
    return runs[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="planner_logs", help="log base dir")
    ap.add_argument("--run", default="", help="specific run folder name under base")
    args = ap.parse_args()

    base = Path(args.base)
    if args.run:
        run_dir = base / args.run
    else:
        run_dir = find_latest_run(base)

    req = load_json(run_dir / "request.json")
    resp = load_json(run_dir / "response.json")

    ox = req.get("ox", [])
    oy = req.get("oy", [])

    sx, sy = req["sx"], req["sy"]
    gx, gy = req["gx"], req["gy"]

    px = resp.get("x", [])
    py = resp.get("y", [])
    ok = resp.get("ok", False)

    plt.figure()
    if ox and oy:
        plt.scatter(ox, oy, s=3)   # obstacles

    plt.scatter([sx], [sy], s=60, marker="o")
    plt.scatter([gx], [gy], s=60, marker="x")

    if ok and px and py:
        plt.plot(px, py)
        plt.title(f"Path ok=true, n={len(px)}, cost={resp.get('cost', None)}")
    else:
        plt.title(f"Path ok={ok}, error={resp.get('error', '')}")

    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.tight_layout()
    plt.show()

    print(f"[viz] run_dir = {run_dir}")


if __name__ == "__main__":
    main()
