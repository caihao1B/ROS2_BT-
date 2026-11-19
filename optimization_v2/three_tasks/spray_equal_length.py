"""Choose an A1 point order with the least turning under the original travel-distance cap."""
import itertools
import math

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp
from scipy.sparse import lil_matrix

from spray_route_time import MISSION, digest, model


def heading_change(a, b):
    return abs(math.remainder(b - a, 2 * math.pi))


def angle(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def minimum_turn_route(points, allowed, initial_yaw, length_cap):
    """A vertex chooses one pair of roads. Neighbor agreement and cut constraints form one path."""
    n = len(points)
    choices = []
    for vertex in range(n):
        degree = 1 if vertex in (0, n - 1) else 2
        choices.extend((vertex, peers) for peers in itertools.combinations(np.flatnonzero(allowed[vertex]), degree))
    constraints = []
    low, high = [], []

    def add(row, lower, upper):
        constraints.append(row)
        low.append(lower)
        high.append(upper)

    for vertex in range(n):
        add({k: 1. for k, (i, _) in enumerate(choices) if i == vertex}, 1., 1.)
    for u, v in np.argwhere(np.triu(allowed, 1)):
        # Both endpoints of an edge must select it together.
        add({k: 1. if i == u else -1. for k, (i, peers) in enumerate(choices)
             if (i == u and v in peers) or (i == v and u in peers)}, 0., 0.)
    add({k: sum(math.dist(points[i], points[j]) for j in peers) / 2
         for k, (i, peers) in enumerate(choices)}, 0., length_cap)
    objective = []
    for i, peers in choices:
        outgoing = angle(points[i], points[peers[-1]])
        incoming = initial_yaw if i == 0 else angle(points[peers[0]], points[i])
        objective.append(0. if i == n - 1 else heading_change(incoming, outgoing))

    cut_count = 0
    while True:
        matrix = lil_matrix((len(constraints), len(choices)))
        for row, terms in enumerate(constraints):
            for col, value in terms.items():
                matrix[row, col] = value
        result = milp(objective, integrality=np.ones(len(choices)), bounds=Bounds(0, 1),
                      constraints=LinearConstraint(matrix.tocsr(), low, high),
                      options={"mip_rel_gap": 0., "time_limit": 30.})
        if not result.success or result.mip_gap > 1e-8:
            raise ValueError(f"Cannot certify a complete minimum-turn route: {result.message}")
        adjacency = {i: peers for (i, peers), used in zip(choices, result.x) if used > .5}
        route, current = [], 0
        while True:
            route.append(current)
            if current == n - 1:
                break
            current = next(j for j in adjacency[current] if j not in route)
        if len(route) == n:
            return np.array(route), float(result.fun), cut_count
        # The endpoint component is a path. Every other component is an isolated
        # cycle and needs at least two edges to the rest of any complete path.
        unvisited = set(adjacency) - set(route)
        while unvisited:
            stack = [min(unvisited)]
            cycle = set()
            while stack:
                node = stack.pop()
                if node not in cycle:
                    cycle.add(node)
                    stack.extend(adjacency[node])
            unvisited -= cycle
            add({k: sum(j not in cycle for j in peers) for k, (i, peers) in enumerate(choices)
                 if i in cycle and any(j not in cycle for j in peers)}, 2., np.inf)
            cut_count += 1


def build_plan(mission, mission_path=MISSION):
    cluster = next(c for c in mission["clusters"] if c["id"] == "A1")
    records = cluster["points"]
    points, allowed, _, report, radius, _ = model(mission, cluster)
    original_order = np.arange(len(points))
    original = report(original_order)
    cap = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    route, lower_bound, cuts = minimum_turn_route(points, allowed, cluster["start"][2], cap)
    candidate = report(route)
    if (candidate["open_motion_m"] > original["open_motion_m"] or
            not math.isclose(lower_bound, candidate["sum_abs_heading_change_rad"], abs_tol=1e-6)):
        raise ValueError("Route reconstruction differs from the optimizer")
    ids = [records[i]["id"] for i in route]
    ingress = [cluster["start"][2], *(angle(points[i], points[j]) for i, j in zip(route, route[1:]))]
    return {"schema": "A1-EQUAL-DISTANCE-TURN-ORDER/1", "source_mission_sha256": digest(mission_path),
            "source_mission_version": mission["version"], "cluster": "A1", "ordered_ids": ids,
            "ingress_yaw_rad": dict(zip(ids, ingress)), "original": original, "candidate": candidate,
            "geometry": {"checked_edges": len(ids) - 1, "minimum_required_clearance_m": round(radius, 6),
                         "admissible_edges": int(allowed.sum() // 2)},
            "certificate": {"solver": "scipy.optimize.milp / HiGHS", "mip_relative_gap": 0.,
                            "turn_lower_bound_rad": round(lower_bound, 6), "subtour_cuts": cuts,
                            "scope": "Fixed service endpoints, original swept-disk prior and longest allowed edge, "
                                     "total length capped at original; measured initial yaw, no final yaw requirement."},
            "scope": "Travel order and incoming Nav2 heading only. No new dose, physical completion or time credit."}
