import heapq
import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_points(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.atan2(float(b[1]) - float(a[1]), float(b[0]) - float(a[0]))


def polyline_length(pts: Sequence[Tuple[float, float]]) -> float:
    if len(pts) < 2:
        return 0.0
    s = 0.0
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        s += math.hypot(float(x1) - float(x0), float(y1) - float(y0))
    return s


def interp_line(a: Tuple[float, float], b: Tuple[float, float], step_m: float) -> List[Tuple[float, float]]:
    dist = math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
    if dist <= max(1e-6, step_m):
        return [a, b]
    n = max(1, int(math.ceil(dist / max(1e-3, step_m))))
    pts: List[Tuple[float, float]] = []
    for i in range(n + 1):
        t = float(i) / float(n)
        x = float(a[0]) * (1.0 - t) + float(b[0]) * t
        y = float(a[1]) * (1.0 - t) + float(b[1]) * t
        pts.append((x, y))
    return pts


def points_to_segment_dict(
    sid: str,
    kind: str,
    cluster_id: int,
    pts: Sequence[Tuple[float, float]],
    frame_id: str = 'map',
) -> Dict[str, Any]:
    out_pts: List[Dict[str, Any]] = []
    prev_yaw = 0.0
    for i, (x, y) in enumerate(pts):
        if i + 1 < len(pts):
            prev_yaw = yaw_from_points((x, y), pts[i + 1])
        qz = math.sin(prev_yaw * 0.5)
        qw = math.cos(prev_yaw * 0.5)
        out_pts.append({
            'i': int(i),
            'x': float(x),
            'y': float(y),
            'z': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': float(qz),
            'qw': float(qw),
            'yaw': float(prev_yaw),
            'frame_id': str(frame_id),
        })
    return {
        'id': str(sid),
        'kind': str(kind),
        'frame_id': str(frame_id),
        'cluster_id': int(cluster_id),
        'poses': out_pts,
    }


def load_plan_clusters(plan_path: str) -> Tuple[Dict[int, List[Dict[str, Any]]], Dict[int, Tuple[float, float, float, float]], Dict[int, int]]:
    with open(plan_path, 'r', encoding='utf-8') as f:
        plan = json.load(f)
    clusters: Dict[int, List[Dict[str, Any]]] = {}
    bboxes: Dict[int, Tuple[float, float, float, float]] = {}
    counts: Dict[int, int] = {}
    for s in plan.get('segments', []):
        cid = int(s.get('cluster_id', -1))
        poses = s.get('poses', [])
        if not poses:
            continue
        xs = [float(p['x']) for p in poses]
        ys = [float(p['y']) for p in poses]
        seg = {
            'seg_id': str(s.get('id', 'seg')),
            'cluster_id': cid,
            'kind': str(s.get('kind', s.get('meta', {}).get('type', 'segment'))),
            'start': (float(poses[0]['x']), float(poses[0]['y'])),
            'end': (float(poses[-1]['x']), float(poses[-1]['y'])),
            'length_m': polyline_length([(float(p['x']), float(p['y'])) for p in poses]),
        }
        clusters.setdefault(cid, []).append(seg)
        counts[cid] = counts.get(cid, 0) + 1
        bbox = bboxes.get(cid)
        seg_bbox = (min(xs), min(ys), max(xs), max(ys))
        if bbox is None:
            bboxes[cid] = seg_bbox
        else:
            bboxes[cid] = (
                min(bbox[0], seg_bbox[0]),
                min(bbox[1], seg_bbox[1]),
                max(bbox[2], seg_bbox[2]),
                max(bbox[3], seg_bbox[3]),
            )
    return clusters, bboxes, counts


@dataclass
class GridData:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    data: List[int]
    unknown_cost: int = 255
    lethal_cost: int = 253

    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        mx = int((float(x) - self.origin_x) / self.resolution)
        my = int((float(y) - self.origin_y) / self.resolution)
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return None
        return mx, my

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        x = self.origin_x + (float(mx) + 0.5) * self.resolution
        y = self.origin_y + (float(my) + 0.5) * self.resolution
        return x, y

    def cost_at_map(self, mx: int, my: int) -> Optional[int]:
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return None
        return int(self.data[my * self.width + mx])

    def cost_at_world(self, x: float, y: float) -> Optional[int]:
        ij = self.world_to_map(x, y)
        if ij is None:
            return None
        return self.cost_at_map(*ij)

    def point_is_free(self, x: float, y: float, occ_thresh: int = 50, allow_unknown: bool = False) -> bool:
        v = self.cost_at_world(x, y)
        if v is None:
            return False
        if v == self.unknown_cost:
            return bool(allow_unknown)
        return int(v) < int(occ_thresh)


class GridSource:
    def __init__(self):
        self.grid: Optional[GridData] = None

    def update_from_occgrid(self, msg) -> None:
        info = msg.info
        self.grid = GridData(
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            width=int(info.width),
            height=int(info.height),
            data=[int(v) for v in msg.data],
            unknown_cost=-1,
            lethal_cost=100,
        )

    def update_from_costmap(self, msg) -> None:
        meta = getattr(msg, 'metadata', getattr(msg, 'meta_data', None))
        if meta is None:
            return

        # nav2_msgs/CostmapMetaData in Humble exposes size_x/size_y, while some
        # older variants expose width/height. Avoid nested getattr(default=...) here,
        # because Python evaluates the default expression eagerly and can raise even
        # when size_x/size_y are present.
        size_x = getattr(meta, 'size_x', None)
        size_y = getattr(meta, 'size_y', None)
        if size_x is None:
            size_x = getattr(meta, 'width', None)
        if size_y is None:
            size_y = getattr(meta, 'height', None)
        if size_x is None or size_y is None:
            return

        self.grid = GridData(
            resolution=float(getattr(meta, 'resolution')),
            origin_x=float(meta.origin.position.x),
            origin_y=float(meta.origin.position.y),
            width=int(size_x),
            height=int(size_y),
            data=[int(v) for v in msg.data],
            unknown_cost=255,
            lethal_cost=253,
        )


def local_bbox_to_map_indices(grid: GridData, bbox: Tuple[float, float, float, float]) -> Optional[Tuple[int, int, int, int]]:
    mn = grid.world_to_map(float(bbox[0]), float(bbox[1]))
    mx = grid.world_to_map(float(bbox[2]), float(bbox[3]))
    if mn is None or mx is None:
        # clamp partially-outside boxes
        x0 = max(0, int((float(bbox[0]) - grid.origin_x) / grid.resolution))
        y0 = max(0, int((float(bbox[1]) - grid.origin_y) / grid.resolution))
        x1 = min(grid.width - 1, int((float(bbox[2]) - grid.origin_x) / grid.resolution))
        y1 = min(grid.height - 1, int((float(bbox[3]) - grid.origin_y) / grid.resolution))
        if x1 <= x0 or y1 <= y0:
            return None
        return x0, y0, x1, y1
    x0, y0 = mn
    x1, y1 = mx
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def occupancy_stats_in_bbox(
    grid: GridData,
    bbox: Tuple[float, float, float, float],
    occ_thresh: int = 50,
    free_unknown: bool = False,
) -> Dict[str, float]:
    idxs = local_bbox_to_map_indices(grid, bbox)
    if idxs is None:
        return {'occupied_ratio': 0.0, 'unknown_ratio': 0.0, 'free_ratio': 0.0, 'fragment_count': 0}
    x0, y0, x1, y1 = idxs
    occ = 0
    unk = 0
    free = 0
    mask: List[List[int]] = []
    for my in range(y0, y1 + 1):
        row: List[int] = []
        for mx in range(x0, x1 + 1):
            v = int(grid.cost_at_map(mx, my) or 0)
            if v == grid.unknown_cost:
                unk += 1
                row.append(1 if free_unknown else 0)
            elif v >= occ_thresh:
                occ += 1
                row.append(0)
            else:
                free += 1
                row.append(1)
        mask.append(row)
    tot = max(1, occ + unk + free)
    frag = connected_components(mask)
    return {
        'occupied_ratio': float(occ) / float(tot),
        'unknown_ratio': float(unk) / float(tot),
        'free_ratio': float(free) / float(tot),
        'fragment_count': int(frag),
    }


def connected_components(mask: Sequence[Sequence[int]]) -> int:
    if not mask:
        return 0
    h = len(mask)
    w = len(mask[0]) if h else 0
    seen = [[False for _ in range(w)] for _ in range(h)]
    cc = 0
    for y in range(h):
        for x in range(w):
            if not mask[y][x] or seen[y][x]:
                continue
            cc += 1
            q = [(x, y)]
            seen[y][x] = True
            while q:
                cx, cy = q.pop()
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx = cx + dx
                    ny = cy + dy
                    if 0 <= nx < w and 0 <= ny < h and mask[ny][nx] and not seen[ny][nx]:
                        seen[ny][nx] = True
                        q.append((nx, ny))
    return cc


def line_risk_cost(grid: Optional[GridData], a: Tuple[float, float], b: Tuple[float, float], occ_thresh: int = 50) -> float:
    if grid is None:
        return 0.0
    pts = interp_line(a, b, max(0.05, grid.resolution * 2.0))
    risk = 0.0
    for x, y in pts:
        v = grid.cost_at_world(x, y)
        if v is None:
            risk += 1.0
            continue
        if v == grid.unknown_cost:
            risk += 1.0
        elif v >= occ_thresh:
            risk += 2.0
        else:
            risk += max(0.0, float(v) / max(1.0, float(occ_thresh)))
    return risk / max(1, len(pts))


def point_risk_cost(grid: Optional[GridData], x: float, y: float, occ_thresh: int = 50, radius_cells: int = 2) -> float:
    if grid is None:
        return 0.0
    ij = grid.world_to_map(x, y)
    if ij is None:
        return 2.0
    mx, my = ij
    risk = 0.0
    cnt = 0
    for yy in range(max(0, my - radius_cells), min(grid.height - 1, my + radius_cells) + 1):
        for xx in range(max(0, mx - radius_cells), min(grid.width - 1, mx + radius_cells) + 1):
            cnt += 1
            v = int(grid.cost_at_map(xx, yy) or 0)
            if v == grid.unknown_cost:
                risk += 1.0
            elif v >= occ_thresh:
                risk += 2.0
            else:
                risk += max(0.0, float(v) / max(1.0, float(occ_thresh)))
    return risk / max(1, cnt)


def astar_cost(grid: Optional[GridData], start: Tuple[float, float], goal: Tuple[float, float], occ_thresh: int = 50) -> Tuple[float, bool]:
    if grid is None:
        return math.hypot(float(goal[0]) - float(start[0]), float(goal[1]) - float(start[1])), False
    s = grid.world_to_map(float(start[0]), float(start[1]))
    g = grid.world_to_map(float(goal[0]), float(goal[1]))
    if s is None or g is None:
        return math.hypot(float(goal[0]) - float(start[0]), float(goal[1]) - float(start[1])) * 1.5, False
    sx, sy = s
    gx, gy = g
    if (sx, sy) == (gx, gy):
        return 0.0, True

    # A* on a lightly downsampled grid would be faster, but the micro-map is small enough.
    pq: List[Tuple[float, float, Tuple[int, int]]] = []
    heapq.heappush(pq, (0.0, 0.0, (sx, sy)))
    best_g: Dict[Tuple[int, int], float] = {(sx, sy): 0.0}
    visited = set()

    def h(xx: int, yy: int) -> float:
        return math.hypot(float(xx - gx), float(yy - gy))

    nbrs = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    while pq:
        f, gcur, cur = heapq.heappop(pq)
        if cur in visited:
            continue
        visited.add(cur)
        if cur == (gx, gy):
            return float(gcur) * grid.resolution, True
        cx, cy = cur
        for dx, dy in nbrs:
            nx = cx + dx
            ny = cy + dy
            if nx < 0 or ny < 0 or nx >= grid.width or ny >= grid.height:
                continue
            v = int(grid.cost_at_map(nx, ny) or 0)
            if v == grid.unknown_cost or v >= occ_thresh:
                continue
            step = math.hypot(float(dx), float(dy))
            ng = float(gcur) + step * (1.0 + float(v) / max(1.0, float(occ_thresh)))
            key = (nx, ny)
            if ng < float(best_g.get(key, 1e18)):
                best_g[key] = ng
                heapq.heappush(pq, (ng + h(nx, ny), ng, key))
    eu = math.hypot(float(goal[0]) - float(start[0]), float(goal[1]) - float(start[1]))
    return eu * 2.0, False


def bbox_from_points(points: Iterable[Tuple[float, float]], pad: float = 0.0) -> Optional[Tuple[float, float, float, float]]:
    pts = list(points)
    if not pts:
        return None
    xs = [float(p[0]) for p in pts]
    ys = [float(p[1]) for p in pts]
    return (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)


def path_heading_jump_density(segments: Sequence[Dict[str, Any]]) -> float:
    jumps = 0.0
    length = 0.0
    last_yaw: Optional[float] = None
    for seg in segments:
        pts = [(float(p['x']), float(p['y'])) for p in seg.get('poses', [])]
        if len(pts) >= 2:
            yaw = yaw_from_points(pts[0], pts[min(1, len(pts) - 1)])
            if last_yaw is not None:
                jumps += abs(wrap_pi(yaw - last_yaw))
            length += polyline_length(pts)
            last_yaw = yaw_from_points(pts[-2], pts[-1]) if len(pts) >= 2 else last_yaw
    return jumps / max(1e-6, length)
