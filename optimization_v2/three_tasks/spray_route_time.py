#!/usr/bin/env python3
"""Compare A1 visit orders with the original point set, endpoints and driving limits."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from statistics import median

import numpy as np
from shapely.geometry import LineString, shape

PROJECT = Path(__file__).resolve().parents[2]
MISSION = PROJECT / "optimization_v2/three_tasks/editions/R10/spray/mission.json"
HISTORY = PROJECT / "validation/closeout_20261119/spray_time_budget.json"
CANDIDATE = PROJECT / "optimization_v2/three_tasks/candidates/A1_time_order.json"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def moves(size):
    """All 2-opt reversals and single-point insertions, fixing both actual service endpoints."""
    identity = np.arange(size)
    result = [identity]
    for i in range(1, size - 2):
        for j in range(i + 1, size - 1):
            order = identity.copy()
            order[i:j + 1] = order[i:j + 1][::-1]
            result.append(order)
    for i in range(1, size - 1):
        for j in range(1, size - 1):
            if abs(i - j) > 1:
                result.append(np.insert(np.delete(identity, i), j, i))
    return np.array(result)


def model(mission, cluster):
    points = np.array([p["target"] for p in cluster["points"]]) + np.array(mission["map_offset"])
    if not np.allclose(points[0], cluster["start"][:2]):
        raise ValueError("Expected the first service point at the original starting pose")
    offsets = points[None, :, :] - points[:, None, :]
    dist = np.linalg.norm(offsets, axis=2)
    heading = np.arctan2(offsets[:, :, 1], offsets[:, :, 0])
    diff = heading[None, :, :] - heading[:, :, None]
    angles = np.abs(np.arctan2(np.sin(diff), np.cos(diff)))
    initial_yaw = heading[0] - cluster["start"][2]
    start_turn = np.abs(np.arctan2(np.sin(initial_yaw), np.cos(initial_yaw)))
    radius = max(math.hypot(*p) for p in mission["vehicle"]["footprint"]) + mission["footprint_margin_m"]
    # Test the entire swept disk, including all possible vehicle headings, against the original prior.
    prior = shape(mission["allowed_prior"])
    edge_limit = max(dist[i, i + 1] for i in range(len(points) - 1))
    allowed = np.zeros_like(dist, dtype=bool)
    for i, start in enumerate(points):
        for j in range(i + 1, len(points)):
            if dist[i, j] <= edge_limit + 1e-8:
                edge = LineString((start, points[j]))
                allowed[j, i] = allowed[i, j] = prior.covers(edge) and edge.distance(prior.boundary) >= radius

    def measure(routes):
        first, last = routes[:, :-1], routes[:, 1:]
        length = dist[first, last].sum(axis=1)
        turn = start_turn[routes[:, 1]] + angles[routes[:, :-2], routes[:, 1:-1], routes[:, 2:]].sum(axis=1)
        return length, turn, allowed[first, last].all(axis=1)

    def cost(routes):
        length, turn, _ = measure(routes)
        nominal = length / mission["speed_cap_mps"] + turn / mission["yaw_cap_rps"]
        invalid_edges = (~allowed[routes[:, :-1], routes[:, 1:]]).sum(axis=1)
        return nominal + 100000.0 * invalid_edges

    def report(order):
        length, turn, valid = measure(order[None, :])
        if not valid[0]:
            raise ValueError("Service edge exceeds the accepted geometric envelope")
        return dict(points=len(order), open_motion_m=round(float(length[0]), 6),
                    sum_abs_heading_change_rad=round(float(turn[0]), 6),
                    translation_and_turn_proxy_s=round(float(cost(order[None, :])[0]), 6),
                    r03_depot_return_m=round(float(dist[order[-1], 0]), 6),
                    minimum_centerline_boundary_distance_m=round(min(
                        LineString((points[i], points[j])).distance(prior.boundary)
                        for i, j in zip(order, order[1:])), 6))
    return points, allowed, cost, report, radius, edge_limit


def solve(points, cost, permutations):
    original = np.arange(len(points))
    # Include simple row/column sweeps of the irregular field along with the frozen route.
    seeds = [original]
    for axis in (0, 1):
        for levels in (sorted(set(points[:, axis])), sorted(set(points[:, axis]), reverse=True)):
            for reverse_first in (True, False):
                order = []
                for i, level in enumerate(levels):
                    group = np.flatnonzero(points[:, axis] == level)
                    row = sorted(group, key=lambda k: points[k, 1-axis], reverse=bool(i % 2) != reverse_first)
                    order.extend(k for k in row if k not in (0, len(points)-1))
                seeds.append(np.array([0, *order, len(points)-1]))
    # Finite penalties let infeasible stripe seeds repair invalid edges. Only a fully valid
    # connected path can be returned. The original admissible route remains a fallback.
    best = original
    for order in seeds:
        score = cost(order[None, :])[0]
        while True:
            variants = order[permutations]
            scores = cost(variants)
            idx = int(np.argmin(scores))
            if scores[idx] >= score - 1e-8:
                break
            order, score = variants[idx], scores[idx]
        if score < cost(best[None, :])[0] - 1e-8:
            best = order
    return best


def observation_summary(history, original, points):
    """Measured navigation phases grouped by incoming turn for original 3 m legs only."""
    by_id = {pid: i for i, pid in enumerate(original)}
    groups = {"straight": [], "quarter_turn": [], "other": []}
    session_ids = []
    for run in history["sessions"]:
        if not run["closed"]:
            continue
        session_ids.append(run["session"])
        for row in run["point_cycles"]:
            i = by_id[row["id"]]
            if i < 2 or not row["confirmed"]:
                continue
            previous, current = points[i-1] - points[i-2], points[i] - points[i-1]
            if not math.isclose(float(np.linalg.norm(current)), 3.0):
                continue
            angle = abs(math.remainder(math.atan2(*current[::-1]) - math.atan2(*previous[::-1]), math.tau))
            group = "straight" if angle < .01 else "quarter_turn" if abs(angle - math.pi/2) < .01 else "other"
            groups[group].append(row["phase_s"]["navigation"])
    return {"sessions": session_ids, "navigation_on_confirmed_original_3m_legs": {
            group: {"n": len(seconds), "median_sim_s": round(median(seconds), 4) if seconds else None}
            for group, seconds in groups.items()},
            "scope": "Descriptive historical observations. Repeated points and different recovery histories are included."}


def build():
    mission = json.loads(MISSION.read_text())
    cluster = next(c for c in mission["clusters"] if c["id"] == "A1")
    original = [p["id"] for p in cluster["points"]]
    if len(set(original)) != len(original):
        raise ValueError("Duplicate point ID")
    points, allowed, cost, report, radius, longest = model(mission, cluster)
    variants = moves(len(points))
    solution = solve(points, cost, variants)
    if sorted(solution.tolist()) != list(range(len(points))) or (solution[0], solution[-1]) != (0, len(points)-1):
        raise ValueError("Point obligation or depot endpoint changed")
    baseline, candidate = report(np.arange(len(points))), report(solution)
    return {"schema": "A1-TIME-ORDER/1", "cluster": "A1", "source_mission_version": mission["version"],
            "source_mission_sha256": digest(MISSION), "source_history_sha256": digest(HISTORY),
            "method": "Distance/speed plus absolute heading change/yaw speed; fixed-endpoint 2-opt and Or-opt with deterministic row/column seeds.",
            "scope": "Offline open path through the last executive service point. Depot return is a separate R03 metric. No work or mission credit.",
            "model_limits": "Kinematic ranking only. Rotation and translation can overlap. Obstacles, localization, docking, tool feedback and startup are not represented.",
            "motion_limits": {"speed_mps": mission["speed_cap_mps"], "yaw_rps": mission["yaw_cap_rps"],
                              "required_centerline_clearance_m": round(radius, 6),
                              "maximum_allowed_edge_m": round(float(longest), 6), "admissible_undirected_edges": int(allowed.sum() // 2)},
            "original": baseline, "candidate": candidate,
            "route_proxy_reduction_s": round(baseline["translation_and_turn_proxy_s"]-candidate["translation_and_turn_proxy_s"], 6),
            "unchanged_original_point_records": True, "ordered_ids": [original[i] for i in solution],
            "measured_reference": observation_summary(json.loads(HISTORY.read_text()), original, points)}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--write", type=Path, help="Write the regenerated candidate")
    p.add_argument("--check", action="store_true", help="Verify the committed candidate against current inputs")
    args = p.parse_args()
    result = build()
    if args.write:
        args.write.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    if args.check and result != json.loads(CANDIDATE.read_text()):
        raise SystemExit("Candidate differs from the result on the bound inputs")
    print(json.dumps({k: result[k] for k in ("original", "candidate", "route_proxy_reduction_s", "measured_reference")}, indent=2))
