from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from coverage_replanner.common import (
    GridData,
    bbox_from_points,
    interp_line,
    line_risk_cost,
    points_to_segment_dict,
    polyline_length,
    wrap_pi,
    yaw_from_points,
)


@dataclass(frozen=True)
class SweepRuntimeConfig:
    occ_thresh: int = 50
    sweep_spacing_m: float = 0.45
    sample_step_m: float = 0.10
    min_run_length_m: float = 0.60
    pose_step_m: float = 0.20
    blocked_zone_pad_m: float = 0.22
    row_segment_target_len_m: float = 5.0
    row_segment_min_len_m: float = 3.2
    tiny_segment_len_m: float = 0.80
    nominal_speed_mps: float = 0.35


# -----------------------------------------------------------------------------
# Basic geometry helpers
# -----------------------------------------------------------------------------


def _segment_points(seg: Dict[str, Any]) -> List[Tuple[float, float]]:
    poses = list(seg.get('poses') or [])
    out: List[Tuple[float, float]] = []
    for p in poses:
        try:
            out.append((float(p['x']), float(p['y'])))
        except Exception:
            continue
    return out


def _segment_kind(seg: Dict[str, Any]) -> str:
    return str(seg.get('kind', seg.get('meta', {}).get('type', 'row')))


def _segment_length(seg: Dict[str, Any]) -> float:
    return polyline_length(_segment_points(seg))


def _segment_heading(seg: Dict[str, Any], *, at_end: bool = False) -> float:
    pts = _segment_points(seg)
    if len(pts) < 2:
        return 0.0
    if at_end:
        return yaw_from_points(pts[-2], pts[-1])
    return yaw_from_points(pts[0], pts[1])


def _dedup_angles_deg(angles: Iterable[Tuple[str, float]], tol_deg: float = 8.0) -> List[Tuple[str, float]]:
    out: List[Tuple[str, float]] = []
    for name, ang in angles:
        ang = (float(ang) + 360.0) % 180.0
        keep = True
        for _n2, a2 in out:
            da = abs(((ang - a2 + 90.0) % 180.0) - 90.0)
            if da <= tol_deg:
                keep = False
                break
        if keep:
            out.append((str(name), float(ang)))
    return out


def dominant_heading_from_segments(segments: Sequence[Dict[str, Any]]) -> float:
    vx = 0.0
    vy = 0.0
    for seg in segments:
        pts = _segment_points(seg)
        if len(pts) < 2:
            continue
        for a, b in zip(pts[:-1], pts[1:]):
            dx = float(b[0]) - float(a[0])
            dy = float(b[1]) - float(a[1])
            L = math.hypot(dx, dy)
            if L <= 1e-6:
                continue
            # Heading modulo pi: use doubled-angle accumulation.
            ang = math.atan2(dy, dx)
            vx += L * math.cos(2.0 * ang)
            vy += L * math.sin(2.0 * ang)
    if abs(vx) < 1e-9 and abs(vy) < 1e-9:
        return 0.0
    ang2 = 0.5 * math.atan2(vy, vx)
    return math.degrees(ang2)


def pca_major_axis_deg(points: Sequence[Tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    mx = sum(float(p[0]) for p in points) / float(len(points))
    my = sum(float(p[1]) for p in points) / float(len(points))
    sxx = 0.0
    sxy = 0.0
    syy = 0.0
    for x, y in points:
        dx = float(x) - mx
        dy = float(y) - my
        sxx += dx * dx
        sxy += dx * dy
        syy += dy * dy
    if abs(sxy) < 1e-9 and abs(sxx - syy) < 1e-9:
        return 0.0
    # Principal axis of symmetric 2x2 covariance matrix.
    ang = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    return math.degrees(ang)


def bbox_major_axis_deg(bbox: Tuple[float, float, float, float]) -> float:
    x0, y0, x1, y1 = bbox
    dx = float(x1) - float(x0)
    dy = float(y1) - float(y0)
    return 0.0 if dx >= dy else 90.0


def path_heading_jump_density_segments(segments: Sequence[Dict[str, Any]]) -> float:
    jumps = 0.0
    total = 0.0
    last_yaw: Optional[float] = None
    for seg in segments:
        pts = _segment_points(seg)
        if len(pts) < 2:
            continue
        yaw0 = yaw_from_points(pts[0], pts[1])
        if last_yaw is not None:
            jumps += abs(wrap_pi(yaw0 - last_yaw))
        total += polyline_length(pts)
        last_yaw = yaw_from_points(pts[-2], pts[-1])
    return jumps / max(1e-6, total)


def connector_ratio_segments(segments: Sequence[Dict[str, Any]]) -> float:
    total = 0.0
    turn = 0.0
    for seg in segments:
        L = _segment_length(seg)
        total += L
        if _segment_kind(seg) != 'row':
            turn += L
    return turn / max(1e-6, total)


def tiny_segment_ratio(segments: Sequence[Dict[str, Any]], tiny_len_m: float) -> float:
    if not segments:
        return 0.0
    bad = 0
    for seg in segments:
        if _segment_length(seg) <= float(tiny_len_m):
            bad += 1
    return float(bad) / float(len(segments))


def repeat_cell_ratio(segments: Sequence[Dict[str, Any]], cell_size_m: float = 0.25) -> float:
    if not segments:
        return 0.0
    seen = set()
    total_hits = 0
    repeat_hits = 0
    step = max(0.10, 0.5 * float(cell_size_m))
    for seg in segments:
        pts = _segment_points(seg)
        if len(pts) < 2:
            continue
        dense = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            line = interp_line(a, b, step)
            if len(dense) >= 1 and line:
                line = line[1:]
            dense.extend(line)
        for x, y in dense:
            ix = int(round(float(x) / max(1e-3, cell_size_m)))
            iy = int(round(float(y) / max(1e-3, cell_size_m)))
            total_hits += 1
            key = (ix, iy)
            if key in seen:
                repeat_hits += 1
            else:
                seen.add(key)
    return float(repeat_hits) / float(max(1, total_hits))


# -----------------------------------------------------------------------------
# World/local frame rotation
# -----------------------------------------------------------------------------


def _rot_to_local(x: float, y: float, origin: Tuple[float, float], angle_deg: float) -> Tuple[float, float]:
    th = math.radians(float(angle_deg))
    c = math.cos(th)
    s = math.sin(th)
    dx = float(x) - float(origin[0])
    dy = float(y) - float(origin[1])
    return c * dx + s * dy, -s * dx + c * dy


def _rot_to_world(u: float, v: float, origin: Tuple[float, float], angle_deg: float) -> Tuple[float, float]:
    th = math.radians(float(angle_deg))
    c = math.cos(th)
    s = math.sin(th)
    return float(origin[0]) + c * u - s * v, float(origin[1]) + s * u + c * v


def _bbox_rotated_extents(bbox: Tuple[float, float, float, float], origin: Tuple[float, float], angle_deg: float) -> Tuple[float, float, float, float]:
    x0, y0, x1, y1 = bbox
    corners = [(x0, y0), (x0, y1), (x1, y0), (x1, y1)]
    loc = [_rot_to_local(x, y, origin, angle_deg) for x, y in corners]
    us = [p[0] for p in loc]
    vs = [p[1] for p in loc]
    return min(us), min(vs), max(us), max(vs)


# -----------------------------------------------------------------------------
# Obstacle / free-space checks
# -----------------------------------------------------------------------------


def _blocked_zone_hit(x: float, y: float, blocked_zones: Sequence[Dict[str, Any]], pad_m: float) -> bool:
    for z in blocked_zones:
        try:
            zx = float(z.get('x', 0.0))
            zy = float(z.get('y', 0.0))
            rr = float(z.get('r', z.get('radius', 0.0))) + float(pad_m)
            if math.hypot(float(x) - zx, float(y) - zy) <= rr:
                return True
        except Exception:
            continue
    return False


def point_is_free(
    grid: Optional[GridData],
    x: float,
    y: float,
    *,
    occ_thresh: int,
    blocked_zones: Sequence[Dict[str, Any]],
    blocked_zone_pad_m: float,
    allow_unknown: bool = False,
) -> bool:
    if _blocked_zone_hit(float(x), float(y), blocked_zones, blocked_zone_pad_m):
        return False
    if grid is None:
        return True
    v = grid.cost_at_world(float(x), float(y))
    if v is None:
        return False
    if v == grid.unknown_cost:
        return bool(allow_unknown)
    return int(v) < int(occ_thresh)


def _line_is_free(
    grid: Optional[GridData],
    a: Tuple[float, float],
    b: Tuple[float, float],
    *,
    cfg: SweepRuntimeConfig,
    blocked_zones: Sequence[Dict[str, Any]],
) -> bool:
    pts = interp_line(a, b, max(0.05, float(cfg.sample_step_m)))
    for x, y in pts:
        if not point_is_free(
            grid, x, y,
            occ_thresh=int(cfg.occ_thresh),
            blocked_zones=blocked_zones,
            blocked_zone_pad_m=float(cfg.blocked_zone_pad_m),
            allow_unknown=False,
        ):
            return False
    return True


# -----------------------------------------------------------------------------
# Grid A* + LOS pruning for connectors
# -----------------------------------------------------------------------------


def astar_path_points(
    grid: Optional[GridData],
    start: Tuple[float, float],
    goal: Tuple[float, float],
    *,
    occ_thresh: int,
    blocked_zones: Sequence[Dict[str, Any]],
    blocked_zone_pad_m: float,
) -> Optional[List[Tuple[float, float]]]:
    if grid is None:
        return [tuple(start), tuple(goal)]
    s = grid.world_to_map(float(start[0]), float(start[1]))
    g = grid.world_to_map(float(goal[0]), float(goal[1]))
    if s is None or g is None:
        return None

    def nearest_free(rc: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        sx, sy = rc
        if point_is_free(
            grid, *grid.map_to_world(sx, sy),
            occ_thresh=occ_thresh,
            blocked_zones=blocked_zones,
            blocked_zone_pad_m=blocked_zone_pad_m,
            allow_unknown=False,
        ):
            return rc
        q = [(sx, sy)]
        seen = {(sx, sy)}
        while q:
            cx, cy = q.pop(0)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or ny < 0 or nx >= grid.width or ny >= grid.height:
                    continue
                if (nx, ny) in seen:
                    continue
                seen.add((nx, ny))
                wx, wy = grid.map_to_world(nx, ny)
                if point_is_free(
                    grid, wx, wy,
                    occ_thresh=occ_thresh,
                    blocked_zones=blocked_zones,
                    blocked_zone_pad_m=blocked_zone_pad_m,
                    allow_unknown=False,
                ):
                    return (nx, ny)
                q.append((nx, ny))
        return None

    s2 = nearest_free(s)
    g2 = nearest_free(g)
    if s2 is None or g2 is None:
        return None

    pq: List[Tuple[float, float, Tuple[int, int]]] = []
    heapq.heappush(pq, (0.0, 0.0, s2))
    came: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_cost: Dict[Tuple[int, int], float] = {s2: 0.0}
    visited = set()
    gx, gy = g2

    def heur(rc: Tuple[int, int]) -> float:
        return math.hypot(float(rc[0] - gx), float(rc[1] - gy))

    nbrs = (
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
    )
    while pq:
        _f, gcost, cur = heapq.heappop(pq)
        if cur in visited:
            continue
        if cur == g2:
            path_rc = [cur]
            while cur in came:
                cur = came[cur]
                path_rc.append(cur)
            path_rc.reverse()
            pts: List[Tuple[float, float]] = [tuple(start)]
            for mx, my in path_rc:
                pts.append(grid.map_to_world(mx, my))
            pts.append(tuple(goal))
            return _dedup_points(pts)
        visited.add(cur)
        cx, cy = cur
        for dx, dy, w in nbrs:
            nx = cx + dx
            ny = cy + dy
            if nx < 0 or ny < 0 or nx >= grid.width or ny >= grid.height:
                continue
            wx, wy = grid.map_to_world(nx, ny)
            if not point_is_free(
                grid, wx, wy,
                occ_thresh=occ_thresh,
                blocked_zones=blocked_zones,
                blocked_zone_pad_m=blocked_zone_pad_m,
                allow_unknown=False,
            ):
                continue
            ng = float(gcost) + float(w)
            key = (nx, ny)
            if ng + 1e-9 < float(g_cost.get(key, 1.0e18)):
                g_cost[key] = ng
                came[key] = cur
                heapq.heappush(pq, (ng + heur(key), ng, key))
    return None


def _dedup_points(points: Sequence[Tuple[float, float]], eps: float = 1e-9) -> List[Tuple[float, float]]:
    if not points:
        return []
    out = [tuple(points[0])]
    for p in points[1:]:
        if math.hypot(float(p[0]) - float(out[-1][0]), float(p[1]) - float(out[-1][1])) > eps:
            out.append((float(p[0]), float(p[1])))
    return out


def los_prune(
    points: Sequence[Tuple[float, float]],
    *,
    grid: Optional[GridData],
    cfg: SweepRuntimeConfig,
    blocked_zones: Sequence[Dict[str, Any]],
) -> List[Tuple[float, float]]:
    pts = _dedup_points(points)
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        best = i + 1
        for j in range(len(pts) - 1, i + 1, -1):
            if _line_is_free(grid, pts[i], pts[j], cfg=cfg, blocked_zones=blocked_zones):
                best = j
                break
        out.append(pts[best])
        i = best
    return _dedup_points(out)


def connector_points(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    *,
    grid: Optional[GridData],
    cfg: SweepRuntimeConfig,
    blocked_zones: Sequence[Dict[str, Any]],
) -> Tuple[List[Tuple[float, float]], str]:
    if math.hypot(float(goal[0]) - float(start[0]), float(goal[1]) - float(start[1])) <= 1e-6:
        return [], 'none'
    if _line_is_free(grid, start, goal, cfg=cfg, blocked_zones=blocked_zones):
        return interp_line(start, goal, cfg.pose_step_m), 'straight'
    astar_pts = astar_path_points(
        grid, start, goal,
        occ_thresh=int(cfg.occ_thresh),
        blocked_zones=blocked_zones,
        blocked_zone_pad_m=float(cfg.blocked_zone_pad_m),
    )
    if astar_pts and len(astar_pts) >= 2:
        pruned = los_prune(astar_pts, grid=grid, cfg=cfg, blocked_zones=blocked_zones)
        dense: List[Tuple[float, float]] = [pruned[0]]
        for a, b in zip(pruned[:-1], pruned[1:]):
            sub = interp_line(a, b, cfg.pose_step_m)
            if dense and sub:
                sub = sub[1:]
            dense.extend(sub)
        return _dedup_points(dense), 'astar_los'
    return interp_line(start, goal, cfg.pose_step_m), 'straight_fallback'


# -----------------------------------------------------------------------------
# Sweep / row generation
# -----------------------------------------------------------------------------


def _split_polyline_by_target(points: Sequence[Tuple[float, float]], target_len: float, min_len: float) -> List[List[Tuple[float, float]]]:
    pts = _dedup_points(points)
    if len(pts) < 2:
        return []
    total = polyline_length(pts)
    if total <= max(1.25 * float(target_len), 1.20 * float(min_len)):
        return [pts]
    chunks: List[List[Tuple[float, float]]] = []
    cur: List[Tuple[float, float]] = [pts[0]]
    acc = 0.0
    used = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        seg = interp_line(a, b, max(0.05, 0.5 * min(target_len, min_len)))
        for p0, p1 in zip(seg[:-1], seg[1:]):
            d = math.hypot(float(p1[0]) - float(p0[0]), float(p1[1]) - float(p0[1]))
            if not cur or math.hypot(float(cur[-1][0]) - float(p0[0]), float(cur[-1][1]) - float(p0[1])) > 1e-9:
                cur.append(p0)
            cur.append(p1)
            acc += d
            used += d
            remaining = total - used
            if acc >= target_len and remaining >= min_len:
                chunks.append(_dedup_points(cur))
                cur = [p1]
                acc = 0.0
    if len(cur) >= 2:
        if chunks and polyline_length(cur) < min_len:
            merged = _dedup_points(chunks[-1] + cur)
            chunks[-1] = merged
        else:
            chunks.append(_dedup_points(cur))
    return [c for c in chunks if polyline_length(c) >= min(0.5 * min_len, 0.4)]


def generate_sweep_plan(
    *,
    grid: Optional[GridData],
    bbox: Tuple[float, float, float, float],
    blocked_zones: Sequence[Dict[str, Any]],
    cluster_id: int,
    orientation_deg: float,
    cfg: SweepRuntimeConfig,
    backend_name: str,
) -> List[Dict[str, Any]]:
    origin = (0.5 * (float(bbox[0]) + float(bbox[2])), 0.5 * (float(bbox[1]) + float(bbox[3])))
    u0, v0, u1, v1 = _bbox_rotated_extents(bbox, origin, orientation_deg)

    segments: List[Dict[str, Any]] = []
    prev_end: Optional[Tuple[float, float]] = None
    prev_seg_id = None
    seg_i = 0
    forward = True
    v = float(v0) + 0.5 * float(cfg.sweep_spacing_m)
    while v <= float(v1) + 1e-6:
        runs_local: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
        start_u: Optional[float] = None
        last_u: Optional[float] = None
        u = float(u0)
        while u <= float(u1) + 1e-6:
            xw, yw = _rot_to_world(u, v, origin, orientation_deg)
            is_free = point_is_free(
                grid, xw, yw,
                occ_thresh=int(cfg.occ_thresh),
                blocked_zones=blocked_zones,
                blocked_zone_pad_m=float(cfg.blocked_zone_pad_m),
                allow_unknown=False,
            )
            if is_free and start_u is None:
                start_u = float(u)
            if is_free:
                last_u = float(u)
            if (not is_free) and start_u is not None and last_u is not None:
                if (last_u - start_u) >= float(cfg.min_run_length_m):
                    runs_local.append(((start_u, v), (last_u, v)))
                start_u = None
                last_u = None
            u += float(cfg.sample_step_m)
        if start_u is not None and last_u is not None and (last_u - start_u) >= float(cfg.min_run_length_m):
            runs_local.append(((start_u, v), (last_u, v)))

        if not forward:
            runs_local = [((b[0], b[1]), (a[0], a[1])) for a, b in reversed(runs_local)]

        for a_loc, b_loc in runs_local:
            a = _rot_to_world(a_loc[0], a_loc[1], origin, orientation_deg)
            b = _rot_to_world(b_loc[0], b_loc[1], origin, orientation_deg)
            row_pts = interp_line(a, b, cfg.pose_step_m)
            row_chunks = _split_polyline_by_target(row_pts, cfg.row_segment_target_len_m, cfg.row_segment_min_len_m)
            for chunk in row_chunks:
                if len(chunk) < 2:
                    continue
                if prev_end is not None:
                    conn_pts, path_type = connector_points(prev_end, chunk[0], grid=grid, cfg=cfg, blocked_zones=blocked_zones)
                    if len(conn_pts) >= 2 and polyline_length(conn_pts) > 1e-6:
                        segments.append(points_to_segment_dict(f'{backend_name}_turn_{seg_i:04d}', 'turn', cluster_id, conn_pts))
                        segments[-1].setdefault('meta', {})
                        segments[-1]['meta'].update({'path_type': path_type, 'backend': backend_name, 'from_seg': prev_seg_id})
                        seg_i += 1
                segments.append(points_to_segment_dict(f'{backend_name}_row_{seg_i:04d}', 'row', cluster_id, chunk))
                segments[-1].setdefault('meta', {})
                segments[-1]['meta'].update({'backend': backend_name, 'orientation_deg': float(orientation_deg)})
                prev_end = chunk[-1]
                prev_seg_id = segments[-1]['id']
                seg_i += 1
        forward = not forward
        v += float(cfg.sweep_spacing_m)
    return segments


# -----------------------------------------------------------------------------
# Candidate selection / scoring
# -----------------------------------------------------------------------------


def recommend_orientations(
    *,
    remaining_segments: Sequence[Dict[str, Any]],
    bbox: Tuple[float, float, float, float],
    robot_pose: Dict[str, Any],
    candidate_limit: int,
) -> List[Tuple[str, float]]:
    pts: List[Tuple[float, float]] = []
    for seg in remaining_segments:
        pts.extend(_segment_points(seg))
    angles: List[Tuple[str, float]] = [
        ('grid_x', 0.0),
        ('grid_y', 90.0),
    ]
    if pts:
        angles.insert(0, ('geom_plan', dominant_heading_from_segments(remaining_segments)))
        angles.insert(1, ('geom_major', pca_major_axis_deg(pts)))
    else:
        angles.insert(0, ('geom_major', bbox_major_axis_deg(bbox)))
    angles.insert(2, ('geom_robot', math.degrees(float(robot_pose.get('yaw', 0.0)))))
    dedup = _dedup_angles_deg(angles)
    return dedup[: max(1, int(candidate_limit))]


def _segment_blocked_metrics(
    segments: Sequence[Dict[str, Any]],
    *,
    cfg: SweepRuntimeConfig,
    blocked_zones: Sequence[Dict[str, Any]],
    blocked_zone_pad_m: float,
    near_extra_pad_m: float = 0.25,
) -> Dict[str, float]:
    if not segments or not blocked_zones:
        return {
            "blocked_overlap_len_m": 0.0,
            "blocked_near_len_m": 0.0,
            "blocked_hit_segments": 0.0,
        }

    step = max(0.05, min(0.20, float(cfg.sample_step_m)))
    overlap_len = 0.0
    near_len = 0.0
    hit_segments = 0

    def zone_distance(x: float, y: float) -> float:
        best = float("inf")
        for z in blocked_zones:
            try:
                zx = float(z.get("x", 0.0))
                zy = float(z.get("y", 0.0))
                rr = float(z.get("r", z.get("radius", 0.0))) + float(blocked_zone_pad_m)
            except Exception:
                continue
            best = min(best, math.hypot(float(x) - zx, float(y) - zy) - rr)
        return best

    for seg in segments:
        pts = _segment_points(seg)
        if len(pts) < 2:
            continue
        dense: List[Tuple[float, float]] = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            line = interp_line(a, b, step)
            if dense and line:
                line = line[1:]
            dense.extend(line)
        if len(dense) < 2:
            continue
        seg_overlap = 0.0
        seg_near = 0.0
        for a, b in zip(dense[:-1], dense[1:]):
            ds = math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
            if ds <= 1e-9:
                continue
            mx = 0.5 * (float(a[0]) + float(b[0]))
            my = 0.5 * (float(a[1]) + float(b[1]))
            dd = zone_distance(mx, my)
            if dd <= 0.0:
                seg_overlap += ds
            elif dd <= float(near_extra_pad_m):
                seg_near += ds
        overlap_len += seg_overlap
        near_len += seg_near
        if seg_overlap > 1e-3 or seg_near > 1e-3:
            hit_segments += 1

    return {
        "blocked_overlap_len_m": float(overlap_len),
        "blocked_near_len_m": float(near_len),
        "blocked_hit_segments": float(hit_segments),
    }


def score_plan_candidate(
    segments: Sequence[Dict[str, Any]],
    *,
    robot_pose: Dict[str, Any],
    grid: Optional[GridData],
    region: Dict[str, Any],
    cfg: SweepRuntimeConfig,
    switch_cost: float,
    blocked_zones: Optional[Sequence[Dict[str, Any]]] = None,
    blocked_zone_pad_m: Optional[float] = None,
    current_plan_segments: Optional[Sequence[Dict[str, Any]]] = None,
) -> Dict[str, float]:
    total_length = sum(_segment_length(seg) for seg in segments)
    turn_length = sum(_segment_length(seg) for seg in segments if _segment_kind(seg) != 'row')
    row_length = max(0.0, total_length - turn_length)
    connector_ratio = turn_length / max(1e-6, total_length)
    heading_jump_density = path_heading_jump_density_segments(segments)
    repeat_ratio = repeat_cell_ratio(segments, cell_size_m=max(0.15, cfg.pose_step_m))
    tiny_ratio = tiny_segment_ratio(segments, cfg.tiny_segment_len_m)
    risk_cost = 0.0
    count = 0
    for seg in segments:
        pts = _segment_points(seg)
        if len(pts) < 2:
            continue
        risk_cost += line_risk_cost(grid, pts[0], pts[-1], occ_thresh=int(cfg.occ_thresh))
        count += 1
    risk_cost = risk_cost / max(1, count)

    first_heading = 0.0
    for seg in segments:
        pts = _segment_points(seg)
        if len(pts) >= 2:
            yaw0 = yaw_from_points(pts[0], pts[1])
            first_heading = abs(wrap_pi(float(yaw0) - float(robot_pose.get('yaw', 0.0))))
            break

    blocked_zones = list(blocked_zones or [])
    blocked_pad = float(cfg.blocked_zone_pad_m if blocked_zone_pad_m is None else blocked_zone_pad_m)
    blocked_metrics = _segment_blocked_metrics(
        segments,
        cfg=cfg,
        blocked_zones=blocked_zones,
        blocked_zone_pad_m=blocked_pad,
    )
    cur_overlap = 0.0
    cur_near = 0.0
    if current_plan_segments:
        cur_metrics = _segment_blocked_metrics(
            list(current_plan_segments),
            cfg=cfg,
            blocked_zones=blocked_zones,
            blocked_zone_pad_m=blocked_pad,
        )
        cur_overlap = float(cur_metrics.get('blocked_overlap_len_m', 0.0))
        cur_near = float(cur_metrics.get('blocked_near_len_m', 0.0))
    blocked_overlap = float(blocked_metrics.get('blocked_overlap_len_m', 0.0))
    blocked_near = float(blocked_metrics.get('blocked_near_len_m', 0.0))
    blocked_hits = float(blocked_metrics.get('blocked_hit_segments', 0.0))
    current_blocked_exposure = max(0.0, cur_overlap) + 0.5 * max(0.0, cur_near)
    blocked_exposure = max(0.0, blocked_overlap) + 0.5 * max(0.0, blocked_near)
    blocked_relief = max(0.0, current_blocked_exposure - blocked_exposure)
    blocked_penalty = (
        8.0 * blocked_overlap
        + 2.5 * blocked_near
        + 2.0 * blocked_hits
        - 2.5 * blocked_relief
    )
    blocked_penalty = max(0.0, blocked_penalty)

    backlog = float(region.get('cleanup_backlog', 0.0))
    est_time = total_length / max(0.05, float(cfg.nominal_speed_mps))
    overlap_penalty = repeat_ratio * max(1.0, row_length)
    structure_penalty = (
        1.0 * heading_jump_density * max(1.0, total_length)
        + 0.8 * connector_ratio * max(1.0, total_length)
        + 1.5 * tiny_ratio * max(1.0, total_length)
    )
    total_cost = (
        1.0 * total_length
        + 1.0 * est_time
        + 2.0 * overlap_penalty
        + 3.0 * risk_cost
        + 1.5 * first_heading
        + 1.0 * backlog
        + 2.0 * structure_penalty
        + 1.0 * blocked_penalty
        + 0.5 * float(switch_cost)
    )
    return {
        'L': float(total_length),
        'T': float(est_time),
        'O': float(overlap_penalty),
        'R': float(risk_cost),
        'H': float(first_heading),
        'B': float(backlog),
        'S': float(structure_penalty),
        'Z': float(blocked_penalty),
        'C': float(switch_cost),
        'J': float(total_cost),
        'row_length_m': float(row_length),
        'turn_length_m': float(turn_length),
        'connector_ratio': float(connector_ratio),
        'heading_jump_density': float(heading_jump_density),
        'repeat_cell_ratio': float(repeat_ratio),
        'tiny_segment_ratio': float(tiny_ratio),
        'segment_count': float(len(segments)),
        'row_segments': float(sum(1 for s in segments if _segment_kind(s) == 'row')),
        'turn_segments': float(sum(1 for s in segments if _segment_kind(s) != 'row')),
        'blocked_overlap_len_m': float(blocked_overlap),
        'blocked_near_len_m': float(blocked_near),
        'blocked_hit_segments': float(blocked_hits),
        'blocked_exposure_m': float(blocked_exposure),
        'current_blocked_exposure_m': float(current_blocked_exposure),
        'blocked_relief_m': float(blocked_relief),
    }


__all__ = [
    'SweepRuntimeConfig',
    'astar_path_points',
    'bbox_major_axis_deg',
    'connector_points',
    'connector_ratio_segments',
    'dominant_heading_from_segments',
    'generate_sweep_plan',
    'path_heading_jump_density_segments',
    'pca_major_axis_deg',
    'recommend_orientations',
    'repeat_cell_ratio',
    'score_plan_candidate',
    'tiny_segment_ratio',
]
