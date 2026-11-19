#!/usr/bin/env python3
"""Summarize consecutive equal-work spray cycles and plot independently measured B1 stops."""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / "validation/r10_continuation_20261119"


def read(path):
    return json.loads(path.read_text())


def equal_work(a, b):
    """Compare only a continuous common route, from the same starting point and with both doses confirmed."""
    if a["scene_comparison_sha256"] != b["scene_comparison_sha256"]:
        raise ValueError("Timing comparison requires the same physical scene")
    paired = []
    previous_before, previous_after = a["first_nav_t"], b["first_nav_t"]
    for x, y in zip(a["points"], b["points"]):
        if x["id"] != y["id"] or not (x["completed"] and y["completed"]):
            break
        if x["ledger_state"] != "SUCCEEDED" or y["ledger_state"] != "SUCCEEDED" or x["uncertain"] or y["uncertain"]:
            raise ValueError("Completed service lacks a confirmed ledger outcome")
        before, after = x["succeeded_t"], y["succeeded_t"]
        paired.append({"point": x["id"], "before_sim_s": round(before-previous_before, 3),
                       "after_sim_s": round(after-previous_after, 3)})
        previous_before, previous_after = before, after
    # Both fixtures spawn on the first point. Timing of actual driving begins after servicing that point.
    travel = paired[1:]
    return {"before_session": a["session"], "after_session": b["session"],
            "common_confirmed_prefix": paired, "consecutive_travel_and_service_cycles": len(travel),
            "before_travel_and_service_s": round(sum(p["before_sim_s"] for p in travel), 3),
            "after_travel_and_service_s": round(sum(p["after_sim_s"] for p in travel), 3)}


def full_work_time(current, history):
    """From confirmed first dose to the last shared consecutive dose, on the fixed full point order."""
    ids = history["r03_reference"]["points"]
    actual = current["points"]
    if (len(ids) != 56 or [p["id"] for p in actual] != ids
            or current["selected_count"] != 56 or current["total_original"] != 56):
        raise ValueError("Full timing comparison requires all 56 original point IDs in order")
    comparisons = []
    for previous in history["sessions"]:
        reference = previous["point_cycles"]
        shared = []
        for old, new in zip(reference, actual):
            if old["id"] != new["id"]:
                raise ValueError("Original and candidate point ordering differs")
            if not old["confirmed"] or not new["completed"]:
                break
            if new["ledger_state"] != "SUCCEEDED" or new["uncertain"]:
                raise ValueError("Unconfirmed physical dose at a claimed full-route point")
            shared.append((old, new))
        count = len(shared)
        before = after = None
        if count >= 2:
            before = round(shared[-1][0]["end_sim_s"] - shared[0][0]["end_sim_s"], 3)
            after = round(shared[-1][1]["succeeded_t"] - shared[0][1]["succeeded_t"], 3)
        comparisons.append({"control_session": previous["session"], "shared_consecutive_points": count,
              "travel_and_service_cycles": max(0, count-1),
              "last_shared_point": shared[-1][1]["id"] if shared else None,
              "control_sim_s": before, "current_sim_s": after,
              "difference_current_minus_control_s": round(after-before, 3) if after is not None else None})
    return {"current_session": current["session"], "compared_sessions": comparisons,
            "clock": "Simulation seconds elapsed between the first and last confirmed dose in the common prefix. Includes all intervening waits.",
            "scope": "Identical original point order. Runs can differ in startup, observation scheduling, control versions and recovery events."}


def inspect(a1, b1):
    runs = a1["runs"]
    control = next(r for r in runs if r["variant"] == "baseline")
    pairs = [equal_work(control, run) for run in runs if run is not control and run["points"] and control["points"]
             and run["points"][0]["id"] == control["points"][0]["id"]]
    stops = b1["trajectory"]
    if len(stops) != 4 or len({s["segment"] for s in stops}) != 4:
        raise ValueError("Expected the four separate B1 candidate gear segments")
    return {"schema": "R10-VEHICLE-CHECKPOINT/1", "a1_equal_work": pairs,
            "b1_stop_states": {s["segment"]: s["state"] for s in stops},
            "b1_physical_verified": sum(s["state"] == "PHYSICALLY_VERIFIED" for s in stops),
            "b1_expected": len(stops), "full_cluster_credit": False}


def plot(a1, b1, target):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import font_manager
    for family in ("Noto Sans CJK SC", "Microsoft YaHei", "SimHei", "Noto Sans CJK JP"):
        if any(font.name == family for font in font_manager.fontManager.ttflist):
            matplotlib.rcParams["font.family"] = family
            break
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.lines import Line2D

    control = next(r for r in a1["runs"] if r["variant"] == "baseline")
    rows = [r for r in a1["runs"] if [p["id"] for p in r["points"]] == [p["id"] for p in control["points"]]]
    fig, (ax, xy, yaw) = plt.subplots(3, figsize=(10.2, 9.1), layout="constrained",
                                    gridspec_kw={"height_ratios": [1.35, 1, 1]})
    colors = ["#465b72", "#087c72", "#d49a21", "#8a528b", "#68873a"]
    ids = [p["id"].removeprefix("point_") for p in control["points"]]
    x = np.arange(len(ids))
    width = .78 / len(rows)
    lines = []
    for i, run in enumerate(rows):
        color = colors[i % len(colors)]
        xs = x + width*(i - (len(rows)-1)/2)
        for j, point in enumerate(run["points"]):
            measured = point.get("cycle_sim_s")
            if measured is None:
                ax.scatter(xs[j], 0, marker="x", color=color, zorder=4, clip_on=False)
                continue
            bar = ax.bar(xs[j], measured, width=width*.95, color=color, edgecolor=color)
            if not point["completed"]:
                bar[0].set_facecolor("white")
                bar[0].set_hatch("///")
        label = "原版" if i == 0 else f"修复版 {i}"
        lines.append(Line2D([], [], color=color, lw=7, label=f"{label} ({run['completed_count']}/{run['selected_count']})"))
    ax.legend(handles=lines, loc="upper left", fontsize=9, ncols=3)
    ax.set_xticks(x, ids)
    ax.set_ylabel("仿真秒")
    ax.set_title("A1　从导航接收目标到确认喷完的耗时（× 表示该点未完成）", loc="left", fontweight="bold")
    ax.margins(y=.28)

    stops = b1["trajectory"]
    for panel, component, multiplier, limit, title, units in (
        (xy, "xy_error_m", 100, 10, "B1　停车位置与目标的距离", "位置误差（cm）"),
        (yaw, "yaw_error_signed_rad", 1, .12, "B1　停车时的车身航向误差", "航向误差绝对值（rad）"),
    ):
        times = np.arange(len(stops))
        all_measured = []
        panel.axhline(limit, color="#a13f38", ls="--", lw=1.3, label="原验收上限")
        for field, color, marker, offset, label in (("estimated", "#345f8f", "o", -.085, "在线定位估计"),
                                                    ("physical", "#c1662d", "D", .085, "独立物理观测")):
            values = [((s.get("stop_observation") or {}).get(field) or {}).get(component) for s in stops]
            measured = [(t+offset, abs(v)*multiplier) for t, v in zip(times, values) if v is not None]
            all_measured.extend(v[1] for v in measured)
            panel.scatter([v[0] for v in measured], [v[1] for v in measured], c=color, marker=marker, s=50,
                          label=label, zorder=3)
        descriptions = {"PARKING_OR_TRACKING_REJECTED": "未通过", "PHYSICALLY_VERIFIED": "已核验",
                        "NOT_STARTED": "未启动", "ATTEMPTED": "行驶中止"}
        labels = [f"第{i+1}段 · {('前进' if s['direction'] > 0 else '倒车')}\n{descriptions[s['state']]}"
                  for i,s in enumerate(stops)]
        panel.set_xticks(times, labels)
        panel.set_xlim(-.45, len(stops)-.55)
        if any(v > 6*limit for v in all_measured):
            panel.set_yscale("symlog", linthresh=limit*1.5)
        panel.set_ylim(0, max([limit, *all_measured])*1.22)
        panel.set_title(title, loc="left", fontweight="bold")
        panel.set_ylabel(units)
        panel.legend(loc="upper right", fontsize=8, ncols=3)
    for panel in (ax, xy, yaw):
        panel.grid(axis="y", alpha=.17)
        panel.spines[["top","right"]].set_visible(False)
    fig.suptitle("R10　Gazebo 整车实测", x=.015, ha="left", fontsize=17, fontweight="bold")
    fig.savefig(target, dpi=145)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--evidence", type=Path, default=EVIDENCE)
    p.add_argument("--figure", type=Path, help="Regenerate the measurement figure")
    p.add_argument("--write-summary", type=Path, help="Write the equal-work comparison")
    args = p.parse_args()
    a1 = read(args.evidence / "a1/comparison.json")
    b1 = read(args.evidence / "b1/physical_connector_report.json")
    summary = inspect(a1, b1)
    full = args.evidence/"a1/full_selection.json"
    if full.exists():
        history = read(ROOT/"validation/closeout_20261119/spray_time_budget.json")
        summary["a1_full_original_prefix"] = full_work_time(read(full), history)
    attempts = sorted((args.evidence/"b1").glob("*/physical_connector_report.json"))
    summary["b1_attempt_history"] = []
    seen = set()
    for report in [*(read(path) for path in attempts), b1]:
        if report["session"] in seen:
            continue
        seen.add(report["session"])
        summary["b1_attempt_history"].append({"session": report["session"], "plan_revision": report["plan_revision"],
            "verified": report["verified_parking_stops"], "selected": report["selected_sections"],
            "sim_s": report["sim_s"], "mission_terminal": report["terminal"], "physical_route_pass": report["passed"]})
    summary["b1_attempt_history"].sort(key=lambda r: r["session"])
    if args.figure:
        plot(a1, b1, args.figure)
    serialized = json.dumps(summary, indent=2, allow_nan=False)
    if args.write_summary:
        args.write_summary.write_text(serialized + "\n")
    print(serialized)


if __name__ == "__main__":
    main()
