#!/usr/bin/env python3
"""Reconcile R03 A1 nominal time with observed R10 service cycles (offline only)."""
import argparse
import gzip
import hashlib
import json
import math
import sqlite3
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "optimization_v2/three_tasks/editions/R03/mission.json"
DEFAULT_CAPTURE = ROOT / "validation/closeout_20261119/spray_time_observations.json"
PHASES = ("navigation", "precision", "tool", "wait_recovery_handoff")
PRECISION = {"NAV_SUCCESS", "GLOBAL_APPROACH_STOP_FOR_PRECISION", "LOCAL_DOCK_GOAL",
             "LOCAL_DOCK_PROPOSAL_REFUSED", "DOCKING_SETTLED_CHECK", "DOCKING_STABLE_OFFSET",
             "DOCKING_OBSERVATION_CONTINUED", "LOCAL_DOCK_NAV_SUCCESS", "LOCAL_DOCK_NAV_FAILED",
             "LOCAL_DOCK_STOP_FOR_GLOBAL_OBSERVATION"}
HOLD = {"BT_INTERRUPTS_NAV", "NEAR_GOAL_PROGRESS_REPLAN", "NAV_TERMINAL_FAILURE", "NAV_REJECTED",
        "STOPPED_RELOCALIZATION_REQUEST", "LATE_LOCALIZATION_HOLD_DISPATCHED", "LOCAL_DOCK_INTERRUPTED",
        "LOCALIZATION_REVALIDATED_RESUME", "RESIDUAL_RETRY", "TASK_STOP", "SUBSET_FINISHED"}


def read(path):
    raw = path.read_bytes()
    return json.loads(gzip.decompress(raw) if path.suffix == ".gz" else raw)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()



def row_digest(rows):
    return sha(json.dumps(rows, separators=(",", ":"), sort_keys=True).encode())


def fmt(value):
    return round(value, 4)


def r03(path):
    raw = path.read_bytes()
    doc = json.loads(raw)
    plan = next(x for x in doc["task_plans"]["spray"] if x["cluster"] == "A1")
    nominal, times, points, action = Counter(), {}, [], 0
    for seg in plan["segments"]:
        nominal[seg["kind"]] += seg["nominal_seconds"]
        action += seg.get("action_seconds", 0)
        if seg["kind"] == "POINT_SERVICE":
            points.append(seg["point_id"])
            times[points[-1]] = 60 + 1.25 * (sum(nominal.values()) + action)
    total = 60 + 1.25 * (sum(nominal.values()) + action)
    if len(set(points)) != plan["point_count"] or not math.isclose(total, plan["estimated_seconds"]):
        raise ValueError("R03 segment count or time formula disagrees with the source plan")
    return {"mission_sha256": sha(raw), "mission_version": doc["version"], "cluster_version": plan["version"],
            "points": points, "nominal_by_kind_s": {k: fmt(v) for k, v in nominal.items()},
            "nominal_service_action_s": fmt(action), "length_m": fmt(plan["motion_m"]),
            "budget_sim_s": plan["budget_s"], "total_estimate_s": fmt(total),
            "tail_after_last_point_estimate_s": fmt(total - times[points[-1]]),
            "estimate_through_point_s": {k: fmt(v) for k, v in times.items()}}


def capture(directory, plan):
    spec, mission = read(directory / "spec.json"), read(directory / "mission.json")
    if (spec["task"], spec["cluster"], spec["case"], spec["point_limit"]) != ("spray", "A1", "nominal", 0):
        raise ValueError("Expected the full original nominal A1 selection, with point_limit=0")
    cluster = next(c for c in mission["clusters"] if c["id"] == "A1")
    if (cluster["source_version"] != plan["cluster_version"] or
            mission["source_plan_version"] != plan["mission_version"] or
            [x["id"] for x in cluster["points"]] != plan["points"] or
            spec["sim_seconds"] != plan["budget_sim_s"]):
        raise ValueError("Ordered service obligations, source version or budget differ from R03")
    result_file = directory / "mission_result.json"
    result = read(result_file) if result_file.exists() else None
    cutoff = (result if result is not None else read(directory / "live.json"))["t"]
    # Select a completed JSONL prefix. A live writer can leave an unfinished last line.
    raw = (directory / "mission_events.jsonl").read_bytes()
    if not raw.endswith(b"\n"):
        raw = raw[:raw.rfind(b"\n") + 1]
    events = []
    event_counts = Counter()
    for line in raw.splitlines():
        event = json.loads(line)
        name = event["event"]
        if event["t"] > cutoff:
            continue
        event_counts[name] += 1
        if name not in PRECISION | HOLD | {"NAV_GOAL", "TOOL_RESULT", "NAV_LIFECYCLE_ACTIVE_VERIFIED"}:
            continue
        fields = {k: event[k] for k in ("t", "event", "point", "state", "remaining", "replayed", "uncertainty") if k in event}
        events.append(fields)
    database = (directory / "ledger/ledger.sqlite").resolve()
    with sqlite3.connect(database.as_uri() + "?mode=ro", uri=True) as conn:
        entries = list(conn.execute("SELECT t,key,status FROM transitions WHERE t<=? ORDER BY id", (cutoff,)))
    transitions = [[t, key.rsplit(":", 1)[-1], status] for t, key, status in entries]
    closed = result is not None
    source = {"mission_event_log_sha256_at_read": sha(raw), "mission_sha256": sha((directory / "mission.json").read_bytes()),
              "spec_sha256": sha((directory / "spec.json").read_bytes()), "runtime_sha256": spec["runtime_sha256"],
              "bound_spray_executor_sha256": spec["implementation_bindings"]["optimization_v2/ws/src/task_platform/scripts/mission_spray_r10.py"],
              "selected_events_sha256": row_digest(events), "ledger_rows_sha256": row_digest(transitions)}
    if closed:
        source["mission_result_sha256"] = sha(result_file.read_bytes())
        if result["selected_points"] != len(plan["points"]):
            raise ValueError("Final result selected a different number of points")
    return {"session": spec["session"], "closed": closed, "end_sim_s": cutoff, "error": result.get("error") if closed else None,
            "mission_report_passed": result.get("passed") if closed else None,
            "wall_seconds_mission_report": result.get("wall_seconds") if closed else None,
            "report_completed_ids": [r["point"] for r in result["completed"]] if closed else None,
            "source": source, "mission_version": mission["version"], "speed_cap_mps": mission["speed_cap_mps"],
            "yaw_cap_rps": mission["yaw_cap_rps"], "event_counts": dict(sorted(event_counts.items())),
            "events": events, "ledger": transitions}


def distribution(values):
    if not values:
        return {"n": 0, "median_s": None, "p90_s": None, "mean_s": None}
    ordered = sorted(values)

    def quantile(p):
        index = p * (len(ordered) - 1)
        lo, hi = math.floor(index), math.ceil(index)
        return fmt(ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo))

    return {"n": len(values), "median_s": quantile(.5), "p90_s": quantile(.9), "mean_s": fmt(sum(values) / len(values))}


def analyze(sample, plan):
    events, end, points = sample["events"], sample["end_sim_s"], plan["points"]
    for stream in ([e["t"] for e in events], [t for t, _, _ in sample["ledger"]]):
        if any(not math.isfinite(t) or t < 0 or t > end for t in stream) or stream != sorted(stream):
            raise ValueError("Observed times must be monotonic and within the sample")
    goals, success, markers = {}, {}, []
    for e in events:
        t, typ, point = e["t"], e["event"], e.get("point")
        if typ == "NAV_GOAL":
            goals.setdefault(point, t)
            markers.append((t, 0, "navigation"))
        elif typ in PRECISION:
            markers.append((t, 0, "precision"))
        elif typ in HOLD:
            markers.append((t, 0, "wait_recovery_handoff"))
        elif typ == "TOOL_RESULT":
            if e["state"] == "SUCCEEDED" and not e["replayed"] and not e["uncertainty"] and e["remaining"] <= 1e-9:
                if point in success:
                    raise ValueError("Duplicate non-replayed success for " + point)
                success[point] = t
            markers.append((t, 0, "wait_recovery_handoff"))
    if not goals:
        raise ValueError("No accepted NAV_GOAL has been observed")
    if sorted(goals, key=points.index) != list(goals) or not set(success).issubset(goals):
        raise ValueError("Observed point order or success linkage differs from R03")
    if sample["closed"] and list(success) != sample["report_completed_ids"]:
        raise ValueError("Mission result completed IDs disagree with the first confirmed tool results")
    ledger_succeeded = [p for _, p, s in sample["ledger"] if s == "SUCCEEDED"]
    if sample["closed"] and ledger_succeeded != list(success):
        raise ValueError("Persistent ledger and mission successes differ")
    active = None
    for t, point, state in sample["ledger"]:
        if point not in points:
            raise ValueError("Unexpected job key in ledger")
        if state == "RUNNING":
            if active is not None:
                raise ValueError("Overlapping tool reservations")
            active = point
            markers.append((t, 1, "tool_open"))
        else:
            if active != point:
                raise ValueError("Unmatched terminal ledger transition")
            active = None
            markers.append((t, 1, "tool_close"))
    if sample["closed"] and active is not None:
        raise ValueError("Final ledger still contains an open reservation")
    stage = "wait_recovery_handoff"
    tool_open, last = False, 0
    intervals = []
    for t, _, mark in sorted(markers, key=lambda m: (m[0], m[1])):
        intervals.append((last, t, "tool" if tool_open else stage))
        if mark == "tool_open":
            tool_open = True
        elif mark == "tool_close":
            tool_open = False
            stage = "wait_recovery_handoff"
        else:
            stage = mark
        last = t
    intervals.append((last, end, "tool" if tool_open else stage))

    def durations(start, stop):
        return {phase: fmt(sum(max(0, min(stop, b) - max(start, a)) for a, b, mode in intervals if mode == phase))
                for phase in PHASES}

    order = list(goals)
    first = goals[order[0]]
    previous = first
    rows = []
    for i, point in enumerate(order):
        stop = success.get(point, goals[order[i + 1]] if i + 1 < len(order) else end)
        if not previous <= goals[point] <= stop <= end:
            raise ValueError("Point-cycle boundary order is invalid")
        phases = durations(previous, stop)
        if abs(stop - previous - sum(phases.values())) > .001:
            raise ValueError("Phase durations do not cover cycle")
        rows.append({"id": point, "first_nav_sim_s": goals[point], "cycle_start_sim_s": previous, "end_sim_s": stop,
                     "confirmed": point in success, "elapsed_s": fmt(stop - previous), "phase_s": phases})
        previous = stop
    completed = [r for r in rows if r["confirmed"]]
    prefix = 0
    while prefix < len(points) and points[prefix] in success:
        prefix += 1
    prefix_t = success[points[prefix-1]] if prefix else None
    estimate = plan["estimate_through_point_s"][points[prefix-1]] if prefix else None
    remaining = [p for p in points if p not in success]
    budget = plan["budget_sim_s"]
    final_stop = next((e["t"] for e in events if e["event"] == "TASK_STOP"), None)
    lifecycle = next((e["t"] for e in events if e["event"] == "NAV_LIFECYCLE_ACTIVE_VERIFIED"), None)
    return {"session": sample["session"], "closed": sample["closed"], "error": sample["error"],
            "original_mission_report_passed": sample["mission_report_passed"],
            "mission_version": sample["mission_version"], "speed_cap_mps": sample["speed_cap_mps"],
            "yaw_cap_rps": sample["yaw_cap_rps"], "observed_through_sim_s": end,
            "mission_report_wall_s": sample["wall_seconds_mission_report"],
            "lifecycle_active_sim_s": lifecycle, "startup_to_first_accepted_goal_s": first,
            "confirmed_points": len(success), "selected_points": len(points), "without_accepted_goal": [p for p in points if p not in goals],
            "attempted_unconfirmed": [p for p in goals if p not in success], "remaining_obligations": len(remaining),
            "ledger_outcomes": dict(Counter(s for _, _, s in sample["ledger"] if s != "RUNNING")),
            "accepted_global_goals": sample["event_counts"].get("NAV_GOAL", 0),
            "local_dock_goals": sample["event_counts"].get("LOCAL_DOCK_GOAL", 0),
            "bt_interrupts_global_nav": sample["event_counts"].get("BT_INTERRUPTS_NAV", 0),
            "stopped_relocalization_requests": sample["event_counts"].get("STOPPED_RELOCALIZATION_REQUEST", 0),
            "phase_totals_from_first_goal_s": durations(first, end),
            "completed_cycle_stats": {"elapsed": distribution([x["elapsed_s"] for x in completed]),
                                     **{k: distribution([x["phase_s"][k] for x in completed]) for k in PHASES}},
            "unassigned_after_last_closed_cycle_s": fmt(end - previous), "observed_stop_cleanup_s": None if final_stop is None else fmt(end - final_stop),
            "prefix_comparison": {"same_contiguous_confirmed_count": prefix, "r03_estimated_through_same_point_s": estimate,
                                  "measured_through_same_point_s": prefix_t, "measured_minus_estimated_s": None if not prefix else fmt(prefix_t-estimate)},
            "budget_arithmetic": {"mean_s_per_selected_point_after_observed_startup_before_reserve": fmt((budget-first)/len(points)),
                                  "signed_time_left_at_cutoff_s": fmt(budget-end),
                                  "required_mean_s_per_remaining_point_at_cutoff_before_reserve":
                                      None if not remaining else fmt(max(0, budget-end)/len(remaining))}, "point_cycles": rows}


def common_prefix(sessions, point_ids, budget):
    confirmations = [{r["id"]: r for r in s["point_cycles"] if r["confirmed"]} for s in sessions]
    prefix = []
    for point in point_ids:
        if not all(point in mapping for mapping in confirmations):
            break
        prefix.append(point)
    aligned = []
    for summary, mapping in zip(sessions, confirmations):
        rows = [mapping[point] for point in prefix]
        observed_t = rows[-1]["end_sim_s"] if rows else None
        remaining = len(point_ids) - len(rows)
        need = None
        if remaining and observed_t is not None:
            # Arithmetic scenarios only. An actual return/stop reserve must be validated locally.
            need = {str(reserve): fmt(max(0, budget - observed_t - reserve) / remaining) for reserve in (0, 60)}
        aligned.append({"session": summary["session"], "same_points": len(rows),
                        "at_last_confirmed_sim_s": observed_t, "remaining_points_at_checkpoint": remaining,
                        "required_mean_s_per_remaining_point_by_hypothetical_reserve_s": need,
                        "completed_cycle_stats": distribution([r["elapsed_s"] for r in rows]),
                        "phase_totals_s": {k: fmt(sum(r["phase_s"][k] for r in rows)) for k in PHASES}})
    return aligned


def history_index(samples, sessions):
    rows = []
    for sample, s in zip(samples, sessions):
        rows.append({"session": s["session"], "closed": s["closed"], "mission_version": s["mission_version"],
                     "mission_sha256": sample["source"]["mission_sha256"], "spec_sha256": sample["source"]["spec_sha256"],
                     "runtime_sha256": sample["source"]["runtime_sha256"],
                     "bound_spray_executor_sha256": sample["source"]["bound_spray_executor_sha256"],
                     "mission_result_sha256": sample["source"].get("mission_result_sha256"),
                     "original_mission_report_passed": s["original_mission_report_passed"],
                     "confirmed_points": s["confirmed_points"], "selected_points": s["selected_points"],
                     "end_sim_s": s["observed_through_sim_s"], "elapsed_completed_cycle": s["completed_cycle_stats"]["elapsed"],
                     "same_prefix": s["prefix_comparison"], "error": s["error"]})
    return {"schema": "spray-time-history-index-v1", "run_count": len(rows), "sessions": rows}



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    src = parser.add_mutually_exclusive_group()
    src.add_argument("--run", action="append", type=Path, help="Full A1 run directory, repeat for multiple runs")
    src.add_argument("--observations", type=Path, default=DEFAULT_CAPTURE, help="Compact observations from --capture")
    output = parser.add_mutually_exclusive_group()
    output.add_argument("--capture", action="store_true", help="Emit compact timestamp evidence instead of the calculated report")
    output.add_argument("--history-index", action="store_true", help="Emit a concise versioned index of the input sessions")
    parser.add_argument("--plan", type=Path, default=PLAN, help="Frozen R03 mission.json")
    args = parser.parse_args()
    plan = r03(args.plan)
    if args.run:
        data = {"schema": "spray-time-observations-v1", "plan_sha256": plan["mission_sha256"],
                "samples": [capture(d, plan) for d in args.run]}
    else:
        data = read(args.observations)
    if data["schema"] != "spray-time-observations-v1" or data["plan_sha256"] != plan["mission_sha256"]:
        raise ValueError("Observation schema or R03 file digest differs")
    for sample in data["samples"]:
        if (sample["source"]["selected_events_sha256"] != row_digest(sample["events"]) or
                sample["source"]["ledger_rows_sha256"] != row_digest(sample["ledger"])):
            raise ValueError("Compact event or ledger digest differs")
    sessions = [analyze(s, plan) for s in data["samples"]]
    report = {"schema": "spray-time-budget-v1", "clock": "simulation_seconds", "stage_status_change": False,
              "r03_reference": plan, "sessions": sessions, "common_confirmed_prefix": common_prefix(sessions, plan["points"], plan["budget_sim_s"])}
    if args.capture:
        output_doc = data
    elif args.history_index:
        output_doc = history_index(data["samples"], sessions)
    else:
        output_doc = report
    print(json.dumps(output_doc, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
