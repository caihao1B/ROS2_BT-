"""Compare two real last-selected-point runs using only exported records and the standard library."""
import argparse
import copy
import gzip
import hashlib
import json
from pathlib import Path


def load_records(case, name):
    path = case / name
    if path.exists():
        return [json.loads(line) for line in path.read_text().splitlines()]
    with gzip.open(str(path) + ".gz", "rt") as stream:
        return [json.loads(line) for line in stream]


def read_json(case, name):
    return json.loads((case / name).read_text())


def evidence(case):
    specs = read_json(case, "spec.json")
    mission = read_json(case, "mission_result.json")
    runtime = read_json(case, "runtime.json")
    wire = read_json(case, "operator_attention_receipt.json")
    decisions = load_records(case, "decisions.jsonl")
    events = load_records(case, "mission_events.jsonl")
    pair = specs["candidate_selected_point_ids"]
    first_attempt = next(index for index, event in enumerate(events)
                         if event["event"] == "NAV_GOAL" and event["point"] == pair[-1])
    blocked = events[first_attempt:]
    aborts = [e for e in blocked if e["event"] == "NAV_TERMINAL_FAILURE"]
    replans = [e for e in blocked if e["event"] == "RETRY_DECISION" and e["action"] == "REPLAN"]
    human = next((d for d in decisions if d["action"] == "HUMAN"
                  and d["reason_code"] == "BOUNDED_REPLAN_EXHAUSTED" and d["manual_latched"]
                  and d["t"] >= blocked[0]["t"]), None)
    if not human:
        raise ValueError(f'{specs["session"]}: last point did not produce a native manual latch')
    raw = human["evidence"]
    proof = all(raw.get(key) is True for key in ("stopped", "tool_stopped", "nav_idle", "localized",
                                               "sensors_healthy", "bindings_valid", "reference_valid"))
    physical_actions = [e for e in blocked if e["event"] == "NAV_GOAL"]
    disk = case / "operator_handoff.json"
    closure = read_json(case, disk.name) if disk.exists() else {}
    ack = closure.get("shutdown", {})
    refs = {}
    for name in ("mission_events.jsonl", "decisions.jsonl", "operator_attention_wire.jsonl",
                 "spec.json", "mission_result.json", "operator_attention_receipt.json",
                 "runtime.json", "operator_handoff.json"):
        file = case / name
        if not file.exists():
            file = case / (name + ".gz")
        if file.exists():
            data = gzip.decompress(file.read_bytes()) if file.suffix == ".gz" else file.read_bytes()
            refs[name] = hashlib.sha256(data).hexdigest()
    return dict(session=specs["session"], point=pair[-1], selected=pair,
                original=mission["original_points"], unselected=len(mission["not_selected"]),
                completed=[p["point"] for p in mission["completed"]], mission_passed=mission["passed"],
                terminal=mission.get("terminal"), error=mission.get("error"), sim_s=mission.get("t"),
                native=dict(time=human["t"], dispatch_id=human["dispatch_id"],
                            reason=human["reason_code"], physical_and_sources_qualified=proof,
                            has_alternative=raw["alternative"], replans=raw["replans"],
                            replan_limit=human["thresholds"]["max_replans"]),
                actions=[e["t"] for e in physical_actions],
                aborts=[dict(t=e["t"], status=e["status"], error_code=e["error_code"]) for e in aborts],
                replans=[e["t"] for e in replans], any_goal_after_human=any(e["t"] > human["t"] for e in physical_actions),
                wire=wire, disk_verified=ack.get("shutdown_verified", False),
                handoff_credit=closure.get("credit_added_by_this_handoff"),
                residual=closure.get("residual"), automatic_resume=closure.get("automatic_resume"),
                double_stop_checks=ack.get("checks", {}),
                observer_exit=next(p["exit"] for p in runtime["processes"] if p["role"] == "observer"),
                runtime_clean=runtime["passed"], source_hashes=refs)


def compare(before, after):
    unchanged = all(x["selected"] == ["point_11_20", "point_10_20"] and x["original"] == 56
                    and x["unselected"] == 54 and x["completed"] == ["point_11_20"]
                    and x["mission_passed"] is False for x in (before, after))
    def native(row):
        n = row["native"]
        return (len(row["actions"]) == len(row["aborts"]) == len(row["replans"]) == 2
                and all(e["status"] == 6 and e["error_code"] != 0 for e in row["aborts"])
                and n["replans"] == n["replan_limit"] == 2 and n["physical_and_sources_qualified"]
                and n["has_alternative"] is False and row["any_goal_after_human"] is False)
    gates = dict(selection_and_accounting_unchanged=unchanged, original_native_action_reconstructed=native(before),
                 original_worker_missed_latch="WAIT_DEADLINE" in before["error"] and before["wire"]["message_count"] == 0,
                 fresh_real_native_action_reconstructed=native(after),
                 operator_observed_both_messages=all(after["wire"].get(k) is True for k in
                     ("passed", "initial_request_received", "closure_received", "closure_matches_durable_case_file"))
                 and after["wire"]["message_count"] >= 2,
                 terminal_preserves_failed_task=after["terminal"] == "HUMAN_REQUIRED" and not after["mission_passed"],
                 independently_closed=after["disk_verified"] and bool(after["double_stop_checks"])
                 and all(after["double_stop_checks"].values()) and after["observer_exit"] == 0 and after["runtime_clean"],
                 retains_work_obligation=after["handoff_credit"] == 0 and after["residual"] == 3
                 and after["automatic_resume"] is False)
    return dict(schema="SPRAY-LAST-POINT-LATCH-COMPARISON/1", passed=all(gates.values()),
                formal_credit=False, checks=gates, initial=before, revised=after)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("initial", "revised", "out"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    result = compare(evidence(args.initial), evidence(args.revised))
    changes = {
        "missing_first_operator_notice": ("wire", "initial_request_received"),
        "missing_final_operator_notice": ("wire", "closure_received"),
        "different_persisted_notice": ("wire", "closure_matches_durable_case_file"),
        "missing_chassis_stop": ("double_stop_checks", "measured_chassis_stop"),
        "missing_tool_stop": ("double_stop_checks", "independent_tool_stopped"),
        "missing_stop_ack": ("double_stop_checks", "explicit_stop_work_ack"),
    }
    # Negative controls redact one observed fact in memory. The robot and the source records are untouched.
    before = result["initial"]
    outcomes = {}
    for name, (section, field) in changes.items():
        masked = copy.deepcopy(result["revised"])
        masked[section][field] = False
        outcomes[name] = compare(before, masked)["passed"] is False
    result["redacted_evidence_negative_controls_rejected"] = outcomes
    result["passed"] = result["passed"] and all(outcomes.values())
    args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(result["checks"], sort_keys=True))
    raise SystemExit(0 if result["passed"] else 2)
