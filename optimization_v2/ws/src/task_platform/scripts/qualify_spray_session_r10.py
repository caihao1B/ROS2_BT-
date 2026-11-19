"""Run or review a consecutive A1 selection, returning the independent physical acceptance result."""
import argparse
import json
from pathlib import Path

from analyze_spray_precision_r10 import evaluate
from common import dump
from spray_precision_session_r10 import run, terrain_version, worker


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    drive = commands.add_parser("run", help="Drive the real Gazebo vehicle in original point order")
    drive.add_argument("--first-index", type=int, default=21)
    drive.add_argument("--count", type=int, default=4)
    drive.add_argument("--seconds", type=int, default=300, help="Task budget in simulation seconds")
    drive.add_argument("--rtf", type=float, default=.55)
    review = commands.add_parser("score", help="Review a completed local precision-candidate session")
    review.add_argument("session", type=Path)
    commands.add_parser("prepare", help="Build this version's worker and terrain node")
    args = parser.parse_args()
    if args.command == "prepare":
        print(worker(), terrain_version()[0])
        return 0
    if args.command == "run":
        session = Path(run(args.first_index, args.count, args.seconds, args.rtf))
    else:
        session = args.session.resolve()
    spec = json.loads((session/"spec.json").read_text())
    if spec.get("case") != "precision_natural" or not spec.get("candidate_selected_point_ids"):
        raise ValueError("Expected an A1 precision session with explicitly selected original points")
    report = evaluate(session, "candidate")
    target = session/"precision_work_report.json"
    dump(target, report)
    print(json.dumps({k:report[k] for k in ("session","selected_count","completed_count","elapsed_sim_s",
                                            "error","measured_subset_pass","formal_credit")}, indent=2))
    print(target)
    return 0 if report["measured_subset_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
