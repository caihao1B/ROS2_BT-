#!/usr/bin/env python3
"""
coverage_sim.dynamic_obstacles_node

A robust dynamic-obstacle generator for **ROS 2 + Gazebo Classic**.

Design goals (tailored to the thesis topic "Online Coverage Path Planning & Navigation in Agriculture"):
- Provide **typical agricultural dynamic obstacles**:
    A) Transient crossing   (Mode A: WAIT)
    B) Co-moving / parallel (Mode B: YIELD)
    C) Persistent blocking  (Mode C/D: DEFER / REPLAN)
- Obstacles are spawned via Gazebo ROS services:
    - SpawnEntity
    - SetEntityState
    - (optional) DeleteEntity
- Can be started **as a standalone module** (separate terminal) and safely restarted.

This node is backward-compatible with the original Milestone1 "legacy_line" mode,
but adds more scenarios and robustness.
"""

from __future__ import annotations

import json
import math
import os
import random
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

from gazebo_msgs.srv import SpawnEntity, SetEntityState
from gazebo_msgs.msg import EntityState

try:
    from gazebo_msgs.srv import DeleteEntity  # type: ignore

    _HAS_DELETE_ENTITY = True
except Exception:  # pragma: no cover
    DeleteEntity = None  # type: ignore
    _HAS_DELETE_ENTITY = False

# Optional: custom scenario file parsing
try:
    import yaml  # type: ignore

    _HAS_YAML = True
except Exception:  # pragma: no cover
    yaml = None  # type: ignore
    _HAS_YAML = False


# ------------------------------- math helpers -------------------------------

def _now() -> float:
    return time.monotonic()


def _hypot(x: float, y: float) -> float:
    return math.hypot(x, y)


def _unit(x: float, y: float) -> Tuple[float, float]:
    n = _hypot(x, y)
    if n < 1e-9:
        return (1.0, 0.0)
    return (x / n, y / n)


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _add(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
    return (a[0] + b[0], a[1] + b[1])


def _sub(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
    return (a[0] - b[0], a[1] - b[1])


def _mul(a: Tuple[float, float], s: float) -> Tuple[float, float]:
    return (a[0] * s, a[1] * s)


def _yaw_to_unit(yaw: float) -> Tuple[float, float]:
    return (math.cos(yaw), math.sin(yaw))


def _angle_diff(a: float, b: float) -> float:
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d


def _bbox_from_points(pts: List[Tuple[float, float]]) -> Optional[Tuple[float, float, float, float]]:
    if not pts:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


def _expand_bbox(b: Tuple[float, float, float, float], pad: float) -> Tuple[float, float, float, float]:
    xmin, ymin, xmax, ymax = b
    return (xmin - pad, ymin - pad, xmax + pad, ymax + pad)


def _line_bbox_endpoints(
    point: Tuple[float, float],
    direction: Tuple[float, float],
    bbox: Tuple[float, float, float, float],
    edge_margin: float = 0.0,
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Compute the intersection endpoints of an infinite line with an axis-aligned bbox.

    Returns (start, end) ordered along +direction (i.e., start is the "t_min" intersection).
    A small `edge_margin` will move the endpoints slightly *inside* the bbox along the line,
    to avoid spawning exactly on the wall / fence collision.

    If the line doesn't intersect twice, returns None.
    """
    px, py = point
    dx, dy = direction
    xmin, ymin, xmax, ymax = bbox

    eps = 1e-9
    hits: List[Tuple[float, float, float]] = []  # (t, x, y)

    # x = xmin / xmax
    if abs(dx) > eps:
        t = (xmin - px) / dx
        y = py + t * dy
        if ymin - 1e-6 <= y <= ymax + 1e-6:
            hits.append((t, xmin, y))
        t = (xmax - px) / dx
        y = py + t * dy
        if ymin - 1e-6 <= y <= ymax + 1e-6:
            hits.append((t, xmax, y))

    # y = ymin / ymax
    if abs(dy) > eps:
        t = (ymin - py) / dy
        x = px + t * dx
        if xmin - 1e-6 <= x <= xmax + 1e-6:
            hits.append((t, x, ymin))
        t = (ymax - py) / dy
        x = px + t * dx
        if xmin - 1e-6 <= x <= xmax + 1e-6:
            hits.append((t, x, ymax))

    if len(hits) < 2:
        return None

    # Deduplicate near-identical (corner) hits
    uniq: Dict[Tuple[int, int], Tuple[float, float, float]] = {}
    for t, x, y in hits:
        key = (int(round(x * 1000.0)), int(round(y * 1000.0)))
        # keep extreme t for same corner, doesn't matter much
        if key not in uniq or abs(t) > abs(uniq[key][0]):
            uniq[key] = (t, x, y)

    hits2 = sorted(uniq.values(), key=lambda v: v[0])
    if len(hits2) < 2:
        return None

    t0, x0, y0 = hits2[0]
    t1, x1, y1 = hits2[-1]

    # Move inside by edge_margin along the line
    ux, uy = _unit(dx, dy)
    start = (x0 + ux * edge_margin, y0 + uy * edge_margin)
    end = (x1 - ux * edge_margin, y1 - uy * edge_margin)
    return (start, end)


def _pgm_size(pgm_path: str) -> Optional[Tuple[int, int]]:
    """Read width/height from a PGM file header (P2 or P5)."""
    try:
        with open(pgm_path, "rb") as f:
            magic = f.readline().strip()
            if magic not in (b"P2", b"P5"):
                return None

            def _next_non_comment_line() -> bytes:
                line = f.readline()
                while line.startswith(b"#") or line.strip() == b"":
                    line = f.readline()
                return line

            dims = _next_non_comment_line().strip().split()
            if len(dims) != 2:
                return None
            w = int(dims[0])
            h = int(dims[1])
            return (w, h)
    except Exception:
        return None


def _load_map_yaml_bbox(map_yaml_path: str) -> Optional[Tuple[float, float, float, float]]:
    """Compute bbox from a standard nav2 map YAML (image + resolution + origin).

    Works without Pillow by parsing PGM header for size.
    """
    if not map_yaml_path:
        return None
    p = os.path.expanduser(map_yaml_path)
    if not os.path.exists(p):
        return None
    if not _HAS_YAML:
        return None

    try:
        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        res = float(data["resolution"])
        origin = data["origin"]
        ox = float(origin[0])
        oy = float(origin[1])
        img_rel = str(data["image"])
        img_path = img_rel
        if not os.path.isabs(img_rel):
            img_path = os.path.join(os.path.dirname(p), img_rel)
        if not os.path.exists(img_path):
            return None

        wh = _pgm_size(img_path)
        if wh is None:
            return None
        w_px, h_px = wh

        xmin = ox
        ymin = oy
        xmax = ox + w_px * res
        ymax = oy + h_px * res
        return (xmin, ymin, xmax, ymax)
    except Exception:
        return None


# ------------------------------- plan loading -------------------------------

@dataclass
class PlanInfo:
    bbox: Tuple[float, float, float, float]
    segments: List[List[Tuple[float, float]]] = field(default_factory=list)
    points: List[Tuple[float, float]] = field(default_factory=list)


def _load_plan(plan_path: str) -> Optional[PlanInfo]:
    if not plan_path:
        return None

    p = os.path.expanduser(plan_path)
    if not os.path.exists(p):
        return None

    try:
        with open(p, "r", encoding="utf-8") as f:
            plan = json.load(f)

        segs: List[List[Tuple[float, float]]] = []
        all_pts: List[Tuple[float, float]] = []

        for seg in plan.get("segments", []):
            pts: List[Tuple[float, float]] = []
            for pose in seg.get("poses", []):
                x = float(pose.get("x"))
                y = float(pose.get("y"))
                pts.append((x, y))
            if pts:
                segs.append(pts)
                all_pts.extend(pts)

        bbox = _bbox_from_points(all_pts)
        if bbox is None:
            return None

        return PlanInfo(bbox=bbox, segments=segs, points=all_pts)
    except Exception:
        return None


def _pick_anchor_and_tangent(plan: PlanInfo, seg_idx: int, frac: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """Pick a point on segment and tangent direction (unit)."""
    if not plan.segments:
        xmin, ymin, xmax, ymax = plan.bbox
        c = (0.5 * (xmin + xmax), 0.5 * (ymin + ymax))
        return c, (1.0, 0.0)

    seg_idx = max(0, min(seg_idx, len(plan.segments) - 1))
    pts = plan.segments[seg_idx]
    if len(pts) < 2:
        return pts[0], (1.0, 0.0)

    frac = _clamp(frac, 0.0, 1.0)
    i = int(round(frac * (len(pts) - 1)))
    i = max(0, min(i, len(pts) - 1))

    if i < len(pts) - 1:
        a = pts[i]
        b = pts[i + 1]
    else:
        a = pts[i - 1]
        b = pts[i]

    tx, ty = _unit(b[0] - a[0], b[1] - a[1])
    return pts[i], (tx, ty)


# ------------------------------ SDF generation ------------------------------

def _make_box_sdf(
    name: str,
    sx: float,
    sy: float,
    sz: float,
    mass: float = 1.0,
    kinematic: bool = True,
    rgba: Optional[Tuple[float, float, float, float]] = None,
) -> str:
    """Generate a Gazebo Classic SDF model for a box obstacle.

    Stability tricks:
    - Model origin at ground contact (z=0) by offsetting link pose by sz/2
    - gravity=false + kinematic=true avoids physics jitter when using SetEntityState
    """
    sx = max(1e-3, float(sx))
    sy = max(1e-3, float(sy))
    sz = max(1e-3, float(sz))
    mass = max(1e-6, float(mass))

    ixx = (1.0 / 12.0) * mass * (sy * sy + sz * sz)
    iyy = (1.0 / 12.0) * mass * (sx * sx + sz * sz)
    izz = (1.0 / 12.0) * mass * (sx * sx + sy * sy)

    link_pose_z = 0.5 * sz

    mat = ""
    if rgba is not None:
        r, g, b, a = rgba
        mat = (
            "<material>"
            f"<ambient>{r} {g} {b} {a}</ambient>"
            f"<diffuse>{r} {g} {b} {a}</diffuse>"
            "</material>"
        )

    kin_tag = "true" if kinematic else "false"

    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='link'>
      <pose>0 0 {link_pose_z:.6f} 0 0 0</pose>
      <kinematic>{kin_tag}</kinematic>
      <gravity>false</gravity>
      <inertial>
        <mass>{mass:.6f}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>
          <ixy>0.0</ixy><ixz>0.0</ixz><iyz>0.0</iyz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
        {mat}
      </visual>
    </link>
  </model>
</sdf>
"""


# ------------------------------- data models --------------------------------

SIZE_PROFILES = {
    "small": (0.20, 0.20, 0.30),
    "human": (0.50, 0.50, 0.90),
    "vehicle": (1.50, 1.00, 0.90),
}


@dataclass
class NoiseModel:
    wiggle_amp: float = 0.0     # meters
    wiggle_freq: float = 0.0    # Hz
    wiggle_phase: float = 0.0   # rad
    gauss_std: float = 0.0      # meters (position noise per tick)


@dataclass
class ObstacleAgent:
    name: str
    kind: str  # line | random_walk | circle | static
    behavior: str  # crossing | parallel | blocking | ...
    size_xyz: Tuple[float, float, float]
    speed: float
    z: float

    # line motion
    start: Tuple[float, float] = (0.0, 0.0)
    end: Tuple[float, float] = (0.0, 0.0)
    t: float = 0.0
    direction: int = 1  # +1 forward, -1 backward
    loop: str = "pingpong"  # pingpong | restart | stay
    moving: bool = True
    next_start_time: float = 0.0
    pause_start_min: float = 0.0
    pause_start_max: float = 0.0
    pause_end_min: float = 0.0
    pause_end_max: float = 0.0

    # random_walk extras
    step_m: float = 4.0
    bbox: Tuple[float, float, float, float] = (0.0, 0.0, 40.0, 30.0)

    # circle extras
    center: Tuple[float, float] = (0.0, 0.0)
    radius: float = 1.0
    omega: float = 0.0
    theta: float = 0.0

    # uncertainty
    wiggle_axis: Tuple[float, float] = (0.0, 0.0)  # unit axis for sinusoidal offset
    noise: NoiseModel = field(default_factory=NoiseModel)
    t_total: float = 0.0

    # runtime
    spawned: bool = False
    active: bool = True
    activate_not_before: float = 0.0
    parked: Tuple[float, float] = (0.0, 0.0)
    last_safe_warn_t: float = 0.0
    warn_next_time: float = 0.0


# ----------------------------- main node class ------------------------------

class DynamicObstacles(Node):
    """Standalone dynamic obstacle generator for Gazebo Classic."""

    def __init__(self) -> None:
        super().__init__("dynamic_obstacles")

        # --- legacy / compatibility params ---
        self.declare_parameter("mode", "low")  # low/high
        self.declare_parameter("seed", 7)  # RNG seed for reproducible scenarios
        self.declare_parameter("scenario", "legacy_line")  # legacy_line | suite | agri_suite | random | circle | yaml
        self.declare_parameter("num_obstacles", 1)

        self.declare_parameter("obstacle_name_prefix", "dyn_obstacle")
        self.declare_parameter("obstacle_size_xyz", [0.30, 0.30, 0.40])
        self.declare_parameter("obstacle_mass", 1.0)
        self.declare_parameter("obstacle_kinematic", True)
        self.declare_parameter("obstacle_speed_mps", 0.55)
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("spawn_z", 0.0)

        self.declare_parameter("plan_path", "")

        # legacy_line manual line override
        self.declare_parameter("use_manual_line", False)
        self.declare_parameter("line_start_xy", [0.0, 0.0])
        self.declare_parameter("line_end_xy", [2.0, 0.0])
        self.declare_parameter("spacing_m", 0.8)
        self.declare_parameter("line_loop", "restart")  # restart|pingpong

        # --- service robustness / auto-detect ---
        # These single-name params keep backward compatibility with old param files
        self.declare_parameter("spawn_service", "/spawn_entity")
        self.declare_parameter("set_state_service", "/gazebo/set_entity_state")
        self.declare_parameter("delete_service", "/delete_entity")

        # New: candidate lists (first available will be used)
        self.declare_parameter("spawn_service_candidates", ["/spawn_entity", "/gazebo/spawn_entity"])
        self.declare_parameter("set_state_service_candidates", ["/gazebo/set_entity_state", "/set_entity_state"])
        self.declare_parameter("delete_service_candidates", ["/delete_entity", "/gazebo/delete_entity"])

        self.declare_parameter("startup_wait_sec", 8.0)
        self.declare_parameter("cleanup_before_spawn", True)
        self.declare_parameter("spawn_retry_period_sec", 1.0)
        self.declare_parameter("spawn_call_timeout_sec", 6.0)

        # --- safe spawn / activation (avoid unavoidable spawn-time collisions) ---
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("safe_spawn_enable", True)
        self.declare_parameter("safe_spawn_arm_delay_sec", 2.0)
        self.declare_parameter("safe_spawn_pose_timeout_sec", 8.0)
        self.declare_parameter("safe_spawn_robot_clearance_m", 1.35)
        self.declare_parameter("safe_spawn_front_clearance_m", 1.80)
        self.declare_parameter("safe_spawn_front_half_angle_deg", 70.0)
        self.declare_parameter("safe_spawn_outside_pad_m", 1.20)
        self.declare_parameter("safe_spawn_log_every_sec", 3.0)

        # --- bounds (for "edge" spawning) ---
        self.declare_parameter("bounds_source", "manual")  # manual | plan | map_yaml
        self.declare_parameter("bbox", [0.0, 0.0, 40.0, 30.0])
        self.declare_parameter("bbox_pad_m", 0.0)
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("edge_margin_m", 0.25)

        # --- suite params (mixed cases, useful for milestone tests) ---
        self.declare_parameter("suite_level", "basic")  # basic|stress
        self.declare_parameter("suite_segment_indices", [0, 2, 6])
        self.declare_parameter("suite_static_enable", False)
        self.declare_parameter("suite_random_step_m", 5.0)
        self.declare_parameter("suite_circle_radius_m", 2.0)
        self.declare_parameter("suite_circle_speed_mps", 0.55)

        # --- agri_suite params (thesis-focused, edge-spawned A/B/C behaviors) ---
        self.declare_parameter("agri_crossing_num", 2)
        self.declare_parameter("agri_parallel_num", 1)
        self.declare_parameter("agri_blocking_num", 1)
        self.declare_parameter("agri_random_walk_num", 0)

        self.declare_parameter("agri_crossing_speed_scale", 1.0)
        self.declare_parameter("agri_parallel_speed_scale", 0.55)
        self.declare_parameter("agri_blocking_speed_scale", 0.8)

        self.declare_parameter("agri_block_leave", True)  # True: pingpong out; False: stay (persistent)
        self.declare_parameter("agri_block_dwell_min_sec", 15.0)
        self.declare_parameter("agri_block_dwell_max_sec", 35.0)
        self.declare_parameter("agri_outside_pause_min_sec", 1.0)
        self.declare_parameter("agri_outside_pause_max_sec", 4.0)

        self.declare_parameter("agri_initial_delay_max_sec", 6.0)

        self.declare_parameter("agri_size_profiles", ["small", "human", "vehicle"])
        self.declare_parameter("agri_size_weights", [3.0, 2.0, 1.0])  # small more frequent
        self.declare_parameter("size_small_xyz", list(SIZE_PROFILES["small"]))
        self.declare_parameter("size_human_xyz", list(SIZE_PROFILES["human"]))
        self.declare_parameter("size_vehicle_xyz", list(SIZE_PROFILES["vehicle"]))

        # motion uncertainty
        self.declare_parameter("enable_wiggle", True)
        self.declare_parameter("wiggle_amp_m_range", [0.00, 0.06])
        self.declare_parameter("wiggle_freq_hz_range", [0.00, 0.30])
        self.declare_parameter("gaussian_noise_std_m", 0.01)

        # --- yaml scenario file ---
        self.declare_parameter("scenario_file", "")

        # --- read core params ---
        self._mode = self.get_parameter("mode").get_parameter_value().string_value.lower().strip()
        self._scenario = self.get_parameter("scenario").get_parameter_value().string_value.lower().strip()
        self._seed = int(self.get_parameter("seed").value) if self.has_parameter("seed") else 7
        random.seed(self._seed)

        # pause defaults
        if self._mode == "high":
            self._mode_pause = (2.0, 5.0)
        else:
            self._mode_pause = (10.0, 30.0)

        # model settings
        self._mass = float(self.get_parameter("obstacle_mass").value)
        self._kinematic = bool(self.get_parameter("obstacle_kinematic").value)
        size_list = self.get_parameter("obstacle_size_xyz").value
        self._default_size_xyz = (float(size_list[0]), float(size_list[1]), float(size_list[2]))
        self._base_speed = float(self.get_parameter("obstacle_speed_mps").value)
        self._spawn_z = float(self.get_parameter("spawn_z").value)

        # safe-spawn / activation
        self._safe_spawn_enable = bool(self.get_parameter("safe_spawn_enable").value)
        self._safe_spawn_arm_delay_sec = float(self.get_parameter("safe_spawn_arm_delay_sec").value)
        self._safe_spawn_pose_timeout_sec = float(self.get_parameter("safe_spawn_pose_timeout_sec").value)
        self._safe_spawn_robot_clearance_m = float(self.get_parameter("safe_spawn_robot_clearance_m").value)
        self._safe_spawn_front_clearance_m = float(self.get_parameter("safe_spawn_front_clearance_m").value)
        self._safe_spawn_front_half_angle_rad = math.radians(float(self.get_parameter("safe_spawn_front_half_angle_deg").value))
        self._safe_spawn_outside_pad_m = float(self.get_parameter("safe_spawn_outside_pad_m").value)
        self._safe_spawn_log_every_sec = float(self.get_parameter("safe_spawn_log_every_sec").value)
        self._robot_pose_xyyaw: Optional[Tuple[float, float, float]] = None
        self._robot_pose_stamp_mono: float = -1.0e9
        if self._safe_spawn_enable:
            try:
                self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._cb_odom, 10)
            except Exception:
                self.get_logger().warn("[dynamic_obstacles] failed to subscribe odom for safe-spawn; will rely on pose timeout")

        # bounds
        self._bbox = self._resolve_bbox()

        # plan
        plan_path = self.get_parameter("plan_path").get_parameter_value().string_value
        self._plan = _load_plan(plan_path)
        if self._plan is None:
            # Fallback: use bbox as "plan bbox"
            self._plan = PlanInfo(bbox=self._bbox, segments=[], points=[])
            if plan_path:
                self.get_logger().warn(f"plan_path cannot be loaded: {plan_path} (fallback to bbox)")

        # build obstacles
        self._obstacles: List[ObstacleAgent] = self._build_obstacles()
        for o in self._obstacles:
            o.parked = self._compute_park_pose(o.start)
            if self._safe_spawn_enable:
                o.active = False
                o.activate_not_before = _now() + self._safe_spawn_arm_delay_sec
            else:
                o.active = True
                o.activate_not_before = 0.0

        # --- services: create candidates ---
        self._spawn_clients = self._create_service_clients(SpawnEntity, "spawn_service", "spawn_service_candidates")
        self._set_state_clients = self._create_service_clients(
            SetEntityState, "set_state_service", "set_state_service_candidates"
        )
        self._delete_clients = []
        if _HAS_DELETE_ENTITY:
            self._delete_clients = self._create_service_clients(DeleteEntity, "delete_service", "delete_service_candidates")  # type: ignore

        # spawn bookkeeping (async retry)
        self._pending_spawn_name: Optional[str] = None
        self._pending_spawn_future = None
        self._pending_spawn_sent_time = 0.0
        self._last_spawn_attempt_time = 0.0

        # set_state bookkeeping: avoid backlogs (one in-flight per entity)
        self._pending_set_futures: Dict[str, object] = {}

        self._warned_spawn = False
        self._warned_set = False

        # Timer loop
        hz = float(self.get_parameter("update_rate_hz").value)
        self._period = 1.0 / max(1e-3, hz)
        self._last_update = _now()
        self._timer = self.create_timer(self._period, self._on_timer)

        # Initial blocking spawn attempt (for launch timing jitter)
        self._initial_spawn_blocking()

        self.get_logger().info(
            f"[dynamic_obstacles] scenario={self._scenario} mode={self._mode} n={len(self._obstacles)} bbox={self._bbox}"
        )

    def _cb_odom(self, msg: Odometry) -> None:
        try:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)), 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z)))
            self._robot_pose_xyyaw = (x, y, yaw)
            self._robot_pose_stamp_mono = _now()
        except Exception:
            pass

    def _compute_park_pose(self, anchor_xy: Tuple[float, float]) -> Tuple[float, float]:
        xmin, ymin, xmax, ymax = self._bbox
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        vx = float(anchor_xy[0]) - cx
        vy = float(anchor_xy[1]) - cy
        ux, uy = _unit(vx, vy)
        # If anchor is near center, push to the south-west corner direction by default.
        if abs(vx) < 1e-6 and abs(vy) < 1e-6:
            ux, uy = _unit(-1.0, -1.0)
        span = max(xmax - xmin, ymax - ymin)
        pad = max(0.5, float(self._safe_spawn_outside_pad_m))
        return (anchor_xy[0] + ux * (span + pad), anchor_xy[1] + uy * (span + pad))

    def _safe_to_activate(self, obs: ObstacleAgent, now: float) -> bool:
        if not self._safe_spawn_enable:
            return True
        if now < float(obs.activate_not_before):
            return False
        # Fail-open after a long odom outage; do not deadlock obstacle activation forever.
        if self._robot_pose_xyyaw is None or (now - self._robot_pose_stamp_mono) > self._safe_spawn_pose_timeout_sec:
            return (now - float(obs.activate_not_before)) >= self._safe_spawn_pose_timeout_sec
        rx, ry, ryaw = self._robot_pose_xyyaw
        sx, sy = obs.start
        dx = float(sx) - float(rx)
        dy = float(sy) - float(ry)
        dist = math.hypot(dx, dy)
        if dist < self._safe_spawn_robot_clearance_m:
            return False
        if dist < self._safe_spawn_front_clearance_m:
            bearing = math.atan2(dy, dx)
            if abs(_angle_diff(bearing, ryaw)) <= self._safe_spawn_front_half_angle_rad:
                return False
        return True

    def _try_activate_obstacle(self, obs: ObstacleAgent, now: float) -> bool:
        if obs.active:
            return True
        if not self._safe_to_activate(obs, now):
            if now - float(obs.last_safe_warn_t) >= max(0.5, self._safe_spawn_log_every_sec):
                obs.last_safe_warn_t = now
                self.get_logger().info(f"[dynamic_obstacles] safe-spawn hold: {obs.name} waiting for robot clearance")
            return False
        obs.active = True
        obs.t = 0.0
        obs.direction = 1
        obs.moving = True
        obs.next_start_time = 0.0
        self._set_state_async(obs.name, obs.start[0], obs.start[1], obs.z)
        self.get_logger().info(f"[dynamic_obstacles] activated obstacle: {obs.name}")
        return True

    # ------------------------------- bbox resolve -------------------------------

    def _resolve_bbox(self) -> Tuple[float, float, float, float]:
        source = self.get_parameter("bounds_source").get_parameter_value().string_value.lower().strip()
        pad = float(self.get_parameter("bbox_pad_m").value)

        if source == "map_yaml":
            map_yaml = self.get_parameter("map_yaml_path").get_parameter_value().string_value
            b = _load_map_yaml_bbox(map_yaml)
            if b is not None:
                return _expand_bbox(b, pad)
            self.get_logger().warn(f"bounds_source=map_yaml but cannot load: {map_yaml} -> fallback to manual/plan")
            source = "manual"

        if source == "plan":
            plan_path = self.get_parameter("plan_path").get_parameter_value().string_value
            plan = _load_plan(plan_path)
            if plan is not None:
                return _expand_bbox(plan.bbox, pad)
            self.get_logger().warn("bounds_source=plan but plan not available -> fallback to manual")
            source = "manual"

        # manual default
        b = self.get_parameter("bbox").value
        if len(b) != 4:
            self.get_logger().warn("bbox param must be [xmin,ymin,xmax,ymax]; using default 0,0,40,30")
            b = [0.0, 0.0, 40.0, 30.0]
        xmin, ymin, xmax, ymax = (float(b[0]), float(b[1]), float(b[2]), float(b[3]))
        if xmax <= xmin or ymax <= ymin:
            self.get_logger().warn("Invalid bbox; using default 0,0,40,30")
            xmin, ymin, xmax, ymax = (0.0, 0.0, 40.0, 30.0)
        return _expand_bbox((xmin, ymin, xmax, ymax), pad)

    # ------------------------------ service helpers -----------------------------

    def _create_service_clients(self, srv_type, single_name_param: str, candidates_param: str):
        # candidates = explicit candidates list + legacy single-name param
        candidates: List[str] = []
        legacy = self.get_parameter(single_name_param).get_parameter_value().string_value
        if legacy:
            candidates.append(legacy)

        try:
            cand_list = list(self.get_parameter(candidates_param).value)
            for c in cand_list:
                if isinstance(c, str) and c and c not in candidates:
                    candidates.append(c)
        except Exception:
            pass

        # final fallback
        if not candidates:
            candidates = [legacy] if legacy else ["/"]

        clients = []
        for name in candidates:
            try:
                cli = self.create_client(srv_type, name)
                clients.append((name, cli))
            except Exception:
                continue
        return clients

    def _first_available_client(self, clients) -> Optional[Tuple[str, object]]:
        for name, cli in clients:
            try:
                if cli.wait_for_service(timeout_sec=0.0):
                    return (name, cli)
            except Exception:
                continue
        return None

    # ---------------------------- obstacle generation ---------------------------

    def _pause_range(self) -> Tuple[float, float]:
        return self._mode_pause

    def _choose_size_xyz(self) -> Tuple[float, float, float]:
        profiles = [str(x) for x in self.get_parameter("agri_size_profiles").value]
        weights = [float(x) for x in self.get_parameter("agri_size_weights").value]
        if not profiles:
            return self._default_size_xyz
        if len(weights) != len(profiles):
            weights = [1.0] * len(profiles)

        profile = random.choices(profiles, weights=weights, k=1)[0].lower().strip()
        if profile == "small":
            v = self.get_parameter("size_small_xyz").value
            return (float(v[0]), float(v[1]), float(v[2]))
        if profile == "human":
            v = self.get_parameter("size_human_xyz").value
            return (float(v[0]), float(v[1]), float(v[2]))
        if profile == "vehicle":
            v = self.get_parameter("size_vehicle_xyz").value
            return (float(v[0]), float(v[1]), float(v[2]))
        return self._default_size_xyz

    def _make_noise(self, wiggle_axis: Tuple[float, float]) -> Tuple[Tuple[float, float], NoiseModel]:
        enable = bool(self.get_parameter("enable_wiggle").value)
        amp_rng = [float(x) for x in self.get_parameter("wiggle_amp_m_range").value]
        freq_rng = [float(x) for x in self.get_parameter("wiggle_freq_hz_range").value]
        gauss = float(self.get_parameter("gaussian_noise_std_m").value)

        if not enable:
            return (wiggle_axis, NoiseModel(0.0, 0.0, 0.0, gauss))

        if len(amp_rng) != 2:
            amp_rng = [0.0, float(amp_rng[0]) if amp_rng else 0.0]
        if len(freq_rng) != 2:
            freq_rng = [0.0, float(freq_rng[0]) if freq_rng else 0.0]

        a0, a1 = min(amp_rng), max(amp_rng)
        f0, f1 = min(freq_rng), max(freq_rng)

        amp = random.uniform(a0, a1)
        freq = random.uniform(f0, f1)
        phase = random.uniform(-math.pi, math.pi)
        return (wiggle_axis, NoiseModel(wiggle_amp=amp, wiggle_freq=freq, wiggle_phase=phase, gauss_std=gauss))

    def _build_obstacles(self) -> List[ObstacleAgent]:
        sc = self._scenario

        if sc in ("legacy", "legacy_line", "line"):
            return self._build_legacy_line()

        if sc == "suite":
            return self._build_suite()

        if sc in ("agri_suite", "agri"):
            return self._build_agri_suite()

        if sc == "random":
            return self._build_random()

        if sc == "circle":
            return self._build_circle()

        if sc in ("yaml", "scenario_file"):
            return self._build_from_yaml()

        self.get_logger().warn(f"Unknown scenario '{sc}', fallback to legacy_line")
        return self._build_legacy_line()

    def _build_legacy_line(self) -> List[ObstacleAgent]:
        use_manual = bool(self.get_parameter("use_manual_line").value)

        if use_manual:
            s = self.get_parameter("line_start_xy").value
            e = self.get_parameter("line_end_xy").value
            start = (float(s[0]), float(s[1]))
            end = (float(e[0]), float(e[1]))
        else:
            # cross a mid-line of bbox
            xmin, ymin, xmax, ymax = self._bbox
            mid_y = 0.5 * (ymin + ymax)
            pad = 1.0
            start = (xmin - pad, mid_y)
            end = (xmax + pad, mid_y)

        spacing = float(self.get_parameter("spacing_m").value)
        loop = self.get_parameter("line_loop").get_parameter_value().string_value.lower().strip()
        if loop not in ("restart", "pingpong"):
            loop = "restart"

        n = max(1, int(self.get_parameter("num_obstacles").value))
        pause_min, pause_max = self._pause_range()

        dx, dy = (end[0] - start[0]), (end[1] - start[1])
        px, py = _unit(-dy, dx)

        prefix = self.get_parameter("obstacle_name_prefix").get_parameter_value().string_value
        obs: List[ObstacleAgent] = []
        for i in range(n):
            off = (i - (n - 1) / 2.0) * spacing
            s2 = _add(start, _mul((px, py), off))
            e2 = _add(end, _mul((px, py), off))
            axis = _unit(px, py)
            axis, noise = self._make_noise(axis)
            obs.append(
                ObstacleAgent(
                    name=f"{prefix}_{i:02d}",
                    kind="line",
                    behavior="legacy_line",
                    size_xyz=self._default_size_xyz,
                    speed=self._base_speed,
                    z=self._spawn_z,
                    start=s2,
                    end=e2,
                    loop=loop,
                    pause_start_min=pause_min,
                    pause_start_max=pause_max,
                    pause_end_min=pause_min,
                    pause_end_max=pause_max,
                    wiggle_axis=axis,
                    noise=noise,
                )
            )
        return obs

    def _build_suite(self) -> List[ObstacleAgent]:
        """Mixed cases (milestone test-suite).

        Keeps the original "suite" spirit, but uses bbox edges for start points when possible.
        """
        assert self._plan is not None

        prefix = self.get_parameter("obstacle_name_prefix").get_parameter_value().string_value
        pause_min, pause_max = self._pause_range()
        edge_margin = float(self.get_parameter("edge_margin_m").value)

        seg_indices = [int(x) for x in self.get_parameter("suite_segment_indices").value]
        if not seg_indices:
            seg_indices = [0, 2, 6]

        # Anchors / tangents
        a0, t0 = _pick_anchor_and_tangent(self._plan, seg_indices[0], frac=0.15)
        a1, t1 = _pick_anchor_and_tangent(self._plan, seg_indices[min(1, len(seg_indices) - 1)], frac=0.35)
        a2, t2 = _pick_anchor_and_tangent(self._plan, seg_indices[min(2, len(seg_indices) - 1)], frac=0.55)

        # Perpendiculars
        p0 = (-t0[1], t0[0])
        p1 = (-t1[1], t1[0])

        obs: List[ObstacleAgent] = []

        # 0) CROSS_PASS (edge-to-edge)
        ep = _line_bbox_endpoints(a0, p0, self._bbox, edge_margin=edge_margin)
        if ep is not None:
            s, e = ep
        else:
            s, e = (_add(a0, _mul(p0, 2.0)), _add(a0, _mul(p0, -2.0)))
        axis = _unit(*t0)  # lateral wiggle along rows
        axis, noise = self._make_noise(axis)
        obs.append(
            ObstacleAgent(
                name=f"{prefix}_cross_00",
                kind="line",
                behavior="crossing",
                size_xyz=self._default_size_xyz,
                speed=1.1 * self._base_speed,
                z=self._spawn_z,
                start=s,
                end=e,
                loop="pingpong",
                pause_start_min=pause_min,
                pause_start_max=pause_max,
                pause_end_min=pause_min,
                pause_end_max=pause_max,
                wiggle_axis=axis,
                noise=noise,
            )
        )

        # 1) ALONG_SLOW (edge-to-edge)
        ep = _line_bbox_endpoints(a0, t0, self._bbox, edge_margin=edge_margin)
        if ep is not None:
            s, e = ep
        else:
            s, e = (a0, _add(a0, _mul(t0, 8.0)))
        axis = _unit(-t0[1], t0[0])  # lateral wiggle across rows
        axis, noise = self._make_noise(axis)
        obs.append(
            ObstacleAgent(
                name=f"{prefix}_along_00",
                kind="line",
                behavior="parallel",
                size_xyz=self._default_size_xyz,
                speed=max(0.05, 0.55 * self._base_speed),
                z=self._spawn_z,
                start=s,
                end=e,
                loop="pingpong",
                pause_start_min=0.3 * pause_min,
                pause_start_max=0.6 * pause_max,
                pause_end_min=0.3 * pause_min,
                pause_end_max=0.6 * pause_max,
                wiggle_axis=axis,
                noise=noise,
            )
        )

        # 2) BLOCK_PINGPONG: from edge into anchor, wait long, back out
        ep = _line_bbox_endpoints(a1, p1, self._bbox, edge_margin=edge_margin)
        if ep is not None:
            s_edge, e_edge = ep
            start = random.choice([s_edge, e_edge])
        else:
            start = _add(a1, _mul(p1, 3.0))
        axis = _unit(*t1)
        axis, noise = self._make_noise(axis)
        obs.append(
            ObstacleAgent(
                name=f"{prefix}_block_00",
                kind="line",
                behavior="blocking",
                size_xyz=self._default_size_xyz,
                speed=max(0.1, self._base_speed),
                z=self._spawn_z,
                start=start,
                end=a1,
                loop="pingpong",
                pause_start_min=1.0,
                pause_start_max=3.0,
                pause_end_min=max(6.0, pause_max),
                pause_end_max=max(12.0, 1.5 * pause_max),
                wiggle_axis=axis,
                noise=noise,
            )
        )

        # 3) RANDOM_WALK (starts near anchor)
        step = max(0.5, float(self.get_parameter("suite_random_step_m").value))
        axis = (0.0, 0.0)
        axis, noise = self._make_noise(axis)
        obs.append(
            ObstacleAgent(
                name=f"{prefix}_rand_00",
                kind="random_walk",
                behavior="random_walk",
                size_xyz=self._default_size_xyz,
                speed=max(0.1, self._base_speed),
                z=self._spawn_z,
                start=a2,
                end=a2,
                loop="restart",
                pause_start_min=0.2 * pause_min,
                pause_start_max=0.5 * pause_max,
                pause_end_min=0.2 * pause_min,
                pause_end_max=0.5 * pause_max,
                step_m=step,
                bbox=self._bbox,
                wiggle_axis=axis,
                noise=noise,
            )
        )

        # 4) CIRCLE
        radius = max(0.2, float(self.get_parameter("suite_circle_radius_m").value))
        speed = max(0.0, float(self.get_parameter("suite_circle_speed_mps").value))
        omega = 0.0 if radius < 1e-6 else (speed / radius)
        xmin, ymin, xmax, ymax = self._bbox
        center = (0.5 * (xmin + xmax), 0.5 * (ymin + ymax))
        theta0 = random.uniform(-math.pi, math.pi)
        p0 = (center[0] + radius * math.cos(theta0), center[1] + radius * math.sin(theta0))
        axis = (0.0, 0.0)
        axis, noise = self._make_noise(axis)
        obs.append(
            ObstacleAgent(
                name=f"{prefix}_circle_00",
                kind="circle",
                behavior="circle",
                size_xyz=self._default_size_xyz,
                speed=speed,
                z=self._spawn_z,
                start=p0,
                end=p0,
                center=center,
                radius=radius,
                omega=omega,
                theta=theta0,
                wiggle_axis=axis,
                noise=noise,
            )
        )

        # optional static
        if bool(self.get_parameter("suite_static_enable").value):
            obs.append(
                ObstacleAgent(
                    name=f"{prefix}_static_00",
                    kind="static",
                    behavior="static",
                    size_xyz=self._default_size_xyz,
                    speed=0.0,
                    z=self._spawn_z,
                    start=_add(a1, _mul(t1, 2.5)),
                    end=_add(a1, _mul(t1, 2.5)),
                )
            )

        # stress level: add more crossers
        if self.get_parameter("suite_level").get_parameter_value().string_value.lower().strip() == "stress":
            base = obs[0]
            dx = base.end[0] - base.start[0]
            dy = base.end[1] - base.start[1]
            px, py = _unit(-dy, dx)
            spacing = float(self.get_parameter("spacing_m").value)
            for i in range(1, 6):
                off = i * spacing
                s2 = _add(base.start, _mul((px, py), off))
                e2 = _add(base.end, _mul((px, py), off))
                axis = _unit(*t0)
                axis, noise = self._make_noise(axis)
                obs.append(
                    ObstacleAgent(
                        name=f"{prefix}_cross_{i:02d}",
                        kind="line",
                        behavior="crossing",
                        size_xyz=self._default_size_xyz,
                        speed=1.0 * self._base_speed,
                        z=self._spawn_z,
                        start=s2,
                        end=e2,
                        loop="pingpong",
                        pause_start_min=pause_min,
                        pause_start_max=pause_max,
                        pause_end_min=pause_min,
                        pause_end_max=pause_max,
                        wiggle_axis=axis,
                        noise=noise,
                    )
                )

        return obs

    def _pick_random_anchor(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        assert self._plan is not None
        seg_indices = [int(x) for x in self.get_parameter("suite_segment_indices").value]
        if not seg_indices:
            seg_indices = [0]
        seg = random.choice(seg_indices)
        frac = random.uniform(0.1, 0.9)
        return _pick_anchor_and_tangent(self._plan, seg, frac=frac)

    def _build_agri_suite(self) -> List[ObstacleAgent]:
        """Thesis-focused agriculture suite.

        - All moving obstacles **start at map/bbox edges**.
        - Provides A/B/C behaviors with size variability and motion uncertainty.
        """
        assert self._plan is not None

        prefix = self.get_parameter("obstacle_name_prefix").get_parameter_value().string_value
        edge_margin = float(self.get_parameter("edge_margin_m").value)
        pause_min, pause_max = self._pause_range()

        n_cross = max(0, int(self.get_parameter("agri_crossing_num").value))
        n_para = max(0, int(self.get_parameter("agri_parallel_num").value))
        n_block = max(0, int(self.get_parameter("agri_blocking_num").value))
        n_rw = max(0, int(self.get_parameter("agri_random_walk_num").value))

        cross_scale = float(self.get_parameter("agri_crossing_speed_scale").value)
        para_scale = float(self.get_parameter("agri_parallel_speed_scale").value)
        block_scale = float(self.get_parameter("agri_blocking_speed_scale").value)

        block_leave = bool(self.get_parameter("agri_block_leave").value)
        dwell_min = float(self.get_parameter("agri_block_dwell_min_sec").value)
        dwell_max = float(self.get_parameter("agri_block_dwell_max_sec").value)
        outside_pause_min = float(self.get_parameter("agri_outside_pause_min_sec").value)
        outside_pause_max = float(self.get_parameter("agri_outside_pause_max_sec").value)

        initial_delay_max = float(self.get_parameter("agri_initial_delay_max_sec").value)

        obs: List[ObstacleAgent] = []

        # --- A) Crossing ---
        for i in range(n_cross):
            anchor, t = self._pick_random_anchor()
            d = (-t[1], t[0])  # perpendicular
            ep = _line_bbox_endpoints(anchor, d, self._bbox, edge_margin=edge_margin)
            if ep is None:
                continue
            s, e = ep
            if random.random() < 0.5:
                s, e = e, s  # randomize entry side

            size_xyz = self._choose_size_xyz()
            axis = _unit(*t)  # wiggle along rows (perpendicular to crossing dir)
            axis, noise = self._make_noise(axis)

            speed = max(0.05, self._base_speed * cross_scale * random.uniform(0.85, 1.15))

            agent = ObstacleAgent(
                name=f"{prefix}_A_cross_{i:02d}",
                kind="line",
                behavior="crossing",
                size_xyz=size_xyz,
                speed=speed,
                z=self._spawn_z,
                start=s,
                end=e,
                loop="pingpong",
                pause_start_min=pause_min,
                pause_start_max=pause_max,
                pause_end_min=pause_min,
                pause_end_max=pause_max,
                wiggle_axis=axis,
                noise=noise,
            )

            # start delay (avoid all obstacles moving at t=0)
            agent.moving = False
            agent.next_start_time = _now() + random.uniform(0.0, max(0.0, initial_delay_max))
            obs.append(agent)

        # --- B) Parallel / co-moving ---
        for i in range(n_para):
            anchor, t = self._pick_random_anchor()
            ep = _line_bbox_endpoints(anchor, t, self._bbox, edge_margin=edge_margin)
            if ep is None:
                continue
            s, e = ep
            if random.random() < 0.5:
                s, e = e, s

            size_xyz = self._choose_size_xyz()
            axis = _unit(-t[1], t[0])  # wiggle across rows
            axis, noise = self._make_noise(axis)

            speed = max(0.03, self._base_speed * para_scale * random.uniform(0.85, 1.15))

            agent = ObstacleAgent(
                name=f"{prefix}_B_para_{i:02d}",
                kind="line",
                behavior="parallel",
                size_xyz=size_xyz,
                speed=speed,
                z=self._spawn_z,
                start=s,
                end=e,
                loop="pingpong",
                pause_start_min=0.2 * pause_min,
                pause_start_max=0.6 * pause_max,
                pause_end_min=0.2 * pause_min,
                pause_end_max=0.6 * pause_max,
                wiggle_axis=axis,
                noise=noise,
            )
            agent.moving = False
            agent.next_start_time = _now() + random.uniform(0.0, max(0.0, initial_delay_max))
            obs.append(agent)

        # --- C) Blocking / persistent ---
        for i in range(n_block):
            anchor, t = self._pick_random_anchor()
            d = (-t[1], t[0])  # approach from side onto row
            ep = _line_bbox_endpoints(anchor, d, self._bbox, edge_margin=edge_margin)
            if ep is None:
                continue
            s_edge, e_edge = ep
            start = random.choice([s_edge, e_edge])

            size_xyz = self._choose_size_xyz()
            axis = _unit(*t)
            axis, noise = self._make_noise(axis)

            speed = max(0.05, self._base_speed * block_scale * random.uniform(0.85, 1.15))

            loop = "pingpong" if block_leave else "stay"
            agent = ObstacleAgent(
                name=f"{prefix}_C_block_{i:02d}",
                kind="line",
                behavior="blocking",
                size_xyz=size_xyz,
                speed=speed,
                z=self._spawn_z,
                start=start,
                end=anchor,
                loop=loop,
                pause_start_min=outside_pause_min,
                pause_start_max=outside_pause_max,
                pause_end_min=min(dwell_min, dwell_max),
                pause_end_max=max(dwell_min, dwell_max),
                wiggle_axis=axis,
                noise=noise,
            )
            agent.moving = False
            agent.next_start_time = _now() + random.uniform(0.0, max(0.0, initial_delay_max))
            obs.append(agent)

        # --- extra: Random walk that starts from an edge point then wanders ---
        for i in range(n_rw):
            xmin, ymin, xmax, ymax = self._bbox
            edge = random.choice(["left", "right", "bottom", "top"])
            if edge == "left":
                start = (xmin + edge_margin, random.uniform(ymin + edge_margin, ymax - edge_margin))
            elif edge == "right":
                start = (xmax - edge_margin, random.uniform(ymin + edge_margin, ymax - edge_margin))
            elif edge == "bottom":
                start = (random.uniform(xmin + edge_margin, xmax - edge_margin), ymin + edge_margin)
            else:
                start = (random.uniform(xmin + edge_margin, xmax - edge_margin), ymax - edge_margin)

            size_xyz = self._choose_size_xyz()
            axis = (0.0, 0.0)
            axis, noise = self._make_noise(axis)

            agent = ObstacleAgent(
                name=f"{prefix}_RW_{i:02d}",
                kind="random_walk",
                behavior="random_walk",
                size_xyz=size_xyz,
                speed=max(0.05, 0.8 * self._base_speed),
                z=self._spawn_z,
                start=start,
                end=start,
                loop="restart",
                pause_start_min=0.2 * pause_min,
                pause_start_max=0.5 * pause_max,
                pause_end_min=0.2 * pause_min,
                pause_end_max=0.5 * pause_max,
                step_m=max(0.5, float(self.get_parameter("suite_random_step_m").value)),
                bbox=self._bbox,
                wiggle_axis=axis,
                noise=noise,
            )
            agent.moving = False
            agent.next_start_time = _now() + random.uniform(0.0, max(0.0, initial_delay_max))
            obs.append(agent)

        return obs

    def _build_random(self) -> List[ObstacleAgent]:
        # simple random-walk group
        assert self._plan is not None
        prefix = self.get_parameter("obstacle_name_prefix").get_parameter_value().string_value
        pause_min, pause_max = self._pause_range()
        n = max(1, int(self.get_parameter("num_obstacles").value))
        step = max(0.5, float(self.get_parameter("suite_random_step_m").value))

        pts = self._plan.points
        if not pts:
            xmin, ymin, xmax, ymax = self._bbox
            pts = [(random.uniform(xmin, xmax), random.uniform(ymin, ymax)) for _ in range(max(10, n))]

        obs: List[ObstacleAgent] = []
        for i in range(n):
            p = pts[i % len(pts)]
            axis = (0.0, 0.0)
            axis, noise = self._make_noise(axis)
            obs.append(
                ObstacleAgent(
                    name=f"{prefix}_rand_{i:02d}",
                    kind="random_walk",
                    behavior="random_walk",
                    size_xyz=self._default_size_xyz,
                    speed=max(0.1, self._base_speed),
                    z=self._spawn_z,
                    start=p,
                    end=p,
                    loop="restart",
                    pause_start_min=0.2 * pause_min,
                    pause_start_max=0.6 * pause_max,
                    pause_end_min=0.2 * pause_min,
                    pause_end_max=0.6 * pause_max,
                    step_m=step,
                    bbox=self._bbox,
                    wiggle_axis=axis,
                    noise=noise,
                )
            )
        return obs

    def _build_circle(self) -> List[ObstacleAgent]:
        prefix = self.get_parameter("obstacle_name_prefix").get_parameter_value().string_value
        n = max(1, int(self.get_parameter("num_obstacles").value))
        radius = max(0.2, float(self.get_parameter("suite_circle_radius_m").value))
        speed = max(0.0, float(self.get_parameter("suite_circle_speed_mps").value))
        omega = 0.0 if radius < 1e-6 else (speed / radius)

        xmin, ymin, xmax, ymax = self._bbox
        center = (0.5 * (xmin + xmax), 0.5 * (ymin + ymax))

        obs: List[ObstacleAgent] = []
        for i in range(n):
            theta0 = (2.0 * math.pi * i / max(1, n))
            p0 = (center[0] + (radius + 0.6 * i) * math.cos(theta0), center[1] + (radius + 0.6 * i) * math.sin(theta0))
            axis = (0.0, 0.0)
            axis, noise = self._make_noise(axis)
            obs.append(
                ObstacleAgent(
                    name=f"{prefix}_circle_{i:02d}",
                    kind="circle",
                    behavior="circle",
                    size_xyz=self._default_size_xyz,
                    speed=speed,
                    z=self._spawn_z,
                    start=p0,
                    end=p0,
                    center=center,
                    radius=radius + 0.6 * i,
                    omega=omega,
                    theta=theta0,
                    wiggle_axis=axis,
                    noise=noise,
                )
            )
        return obs

    def _build_from_yaml(self) -> List[ObstacleAgent]:
        scenario_file = self.get_parameter("scenario_file").get_parameter_value().string_value
        if not scenario_file:
            self.get_logger().warn("scenario=yaml but scenario_file is empty -> fallback to agri_suite")
            self._scenario = "agri_suite"
            return self._build_agri_suite()

        if not _HAS_YAML:
            self.get_logger().warn("scenario=yaml but python yaml is not available -> fallback to agri_suite")
            self._scenario = "agri_suite"
            return self._build_agri_suite()

        p = os.path.expanduser(scenario_file)
        if not os.path.exists(p):
            self.get_logger().warn(f"scenario_file not found: {p} -> fallback to agri_suite")
            self._scenario = "agri_suite"
            return self._build_agri_suite()

        try:
            with open(p, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)  # type: ignore
        except Exception as e:
            self.get_logger().warn(f"Failed to load scenario_file: {p} ({e}) -> fallback to agri_suite")
            self._scenario = "agri_suite"
            return self._build_agri_suite()

        # Optional bbox override
        bbox = data.get("bbox")
        if isinstance(bbox, Sequence) and len(bbox) == 4:
            self._bbox = (float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))

        edge_margin = float(data.get("edge_margin_m", self.get_parameter("edge_margin_m").value))

        # Defaults
        default_speed = float(data.get("default_speed_mps", self._base_speed))
        default_loop = str(data.get("default_loop", "pingpong")).lower()
        default_pause = data.get("default_pause_sec", [2.0, 5.0])
        if not (isinstance(default_pause, Sequence) and len(default_pause) == 2):
            default_pause = [2.0, 5.0]
        pmin, pmax = (float(default_pause[0]), float(default_pause[1]))
        pmin, pmax = (min(pmin, pmax), max(pmin, pmax))

        obs_list = data.get("obstacles", [])
        if not isinstance(obs_list, list) or not obs_list:
            self.get_logger().warn("scenario_file has no obstacles list -> fallback to agri_suite")
            self._scenario = "agri_suite"
            return self._build_agri_suite()

        prefix = self.get_parameter("obstacle_name_prefix").get_parameter_value().string_value
        built: List[ObstacleAgent] = []

        for i, item in enumerate(obs_list):
            if not isinstance(item, dict):
                continue
            name = str(item.get("name", f"{prefix}_yaml_{i:02d}"))
            kind = str(item.get("kind", "line")).lower()
            behavior = str(item.get("behavior", kind)).lower()

            size_xyz = self._default_size_xyz
            if "size_profile" in item:
                prof = str(item["size_profile"]).lower()
                size_xyz = SIZE_PROFILES.get(prof, self._default_size_xyz)
            if "size_xyz" in item and isinstance(item["size_xyz"], Sequence) and len(item["size_xyz"]) == 3:
                size_xyz = (float(item["size_xyz"][0]), float(item["size_xyz"][1]), float(item["size_xyz"][2]))

            speed = float(item.get("speed_mps", default_speed))
            loop = str(item.get("loop", default_loop)).lower()

            pause_start = item.get("pause_start_sec", [pmin, pmax])
            pause_end = item.get("pause_end_sec", [pmin, pmax])
            if not (isinstance(pause_start, Sequence) and len(pause_start) == 2):
                pause_start = [pmin, pmax]
            if not (isinstance(pause_end, Sequence) and len(pause_end) == 2):
                pause_end = [pmin, pmax]
            ps0, ps1 = (float(pause_start[0]), float(pause_start[1]))
            pe0, pe1 = (float(pause_end[0]), float(pause_end[1]))
            ps0, ps1 = (min(ps0, ps1), max(ps0, ps1))
            pe0, pe1 = (min(pe0, pe1), max(pe0, pe1))

            start = (0.0, 0.0)
            end = (0.0, 0.0)

            # Allow either explicit start/end OR anchor + direction type
            if "start" in item and "end" in item:
                s = item["start"]
                e = item["end"]
                if isinstance(s, Sequence) and len(s) == 2 and isinstance(e, Sequence) and len(e) == 2:
                    start = (float(s[0]), float(s[1]))
                    end = (float(e[0]), float(e[1]))
            else:
                # anchor specification
                anchor = item.get("anchor", {})
                ax = ay = None
                tangent = None

                if isinstance(anchor, dict):
                    if "segment" in anchor:
                        seg = int(anchor.get("segment", 0))
                        frac = float(anchor.get("frac", 0.5))
                        assert self._plan is not None
                        (ax, ay), tangent = _pick_anchor_and_tangent(self._plan, seg, frac)
                    elif "x" in anchor and "y" in anchor:
                        ax = float(anchor["x"])
                        ay = float(anchor["y"])
                        yaw = float(anchor.get("yaw", 0.0))
                        tangent = _yaw_to_unit(yaw)

                if ax is None or ay is None or tangent is None:
                    # fallback center
                    xmin, ymin, xmax, ymax = self._bbox
                    ax, ay = (0.5 * (xmin + xmax), 0.5 * (ymin + ymax))
                    tangent = (1.0, 0.0)

                if behavior in ("crossing", "cross", "transient"):
                    d = (-tangent[1], tangent[0])
                    ep = _line_bbox_endpoints((ax, ay), d, self._bbox, edge_margin=edge_margin)
                    if ep is None:
                        continue
                    start, end = ep
                elif behavior in ("parallel", "co_moving", "comoving"):
                    ep = _line_bbox_endpoints((ax, ay), tangent, self._bbox, edge_margin=edge_margin)
                    if ep is None:
                        continue
                    start, end = ep
                elif behavior in ("blocking", "block", "persistent"):
                    d = (-tangent[1], tangent[0])
                    ep = _line_bbox_endpoints((ax, ay), d, self._bbox, edge_margin=edge_margin)
                    if ep is None:
                        continue
                    s_edge, e_edge = ep
                    start = random.choice([s_edge, e_edge])
                    end = (ax, ay)
                    # default blocking loop
                    if loop not in ("pingpong", "stay", "restart"):
                        loop = "stay"
                else:
                    # generic line through anchor along tangent
                    ep = _line_bbox_endpoints((ax, ay), tangent, self._bbox, edge_margin=edge_margin)
                    if ep is None:
                        continue
                    start, end = ep

            # wiggle/noise overrides
            axis = (0.0, 0.0)
            if "wiggle_axis" in item and isinstance(item["wiggle_axis"], Sequence) and len(item["wiggle_axis"]) == 2:
                axis = _unit(float(item["wiggle_axis"][0]), float(item["wiggle_axis"][1]))
            wiggle_amp = float(item.get("wiggle_amp_m", 0.0))
            wiggle_freq = float(item.get("wiggle_freq_hz", 0.0))
            gauss = float(item.get("noise_std_m", float(self.get_parameter("gaussian_noise_std_m").value)))
            noise = NoiseModel(wiggle_amp=wiggle_amp, wiggle_freq=wiggle_freq, wiggle_phase=random.uniform(-math.pi, math.pi), gauss_std=gauss)

            built.append(
                ObstacleAgent(
                    name=name,
                    kind=kind,
                    behavior=behavior,
                    size_xyz=size_xyz,
                    speed=speed,
                    z=self._spawn_z,
                    start=start,
                    end=end,
                    loop=loop,
                    pause_start_min=ps0,
                    pause_start_max=ps1,
                    pause_end_min=pe0,
                    pause_end_max=pe1,
                    step_m=float(item.get("step_m", 4.0)),
                    bbox=self._bbox,
                    wiggle_axis=axis,
                    noise=noise,
                )
            )

        if not built:
            self.get_logger().warn("No valid obstacles parsed from scenario_file -> fallback to agri_suite")
            self._scenario = "agri_suite"
            return self._build_agri_suite()

        return built

    # ---------------------------- spawn / set_state ----------------------------

    def _initial_spawn_blocking(self) -> None:
        # wait a bit for Gazebo services
        wait_s = float(self.get_parameter("startup_wait_sec").value)
        t0 = _now()
        while _now() - t0 < wait_s:
            if self._first_available_client(self._spawn_clients) is not None:
                break
            time.sleep(0.2)

        # Optional cleanup before spawning (best effort)
        if bool(self.get_parameter("cleanup_before_spawn").value) and self._delete_clients:
            av = self._first_available_client(self._delete_clients)
            if av is not None:
                _, del_cli = av
                for o in self._obstacles:
                    try:
                        req = DeleteEntity.Request()  # type: ignore
                        req.name = o.name
                        fut = del_cli.call_async(req)
                        rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
                    except Exception:
                        pass

        # Try spawn all once (blocking)
        self._spawn_all_blocking()

    def _spawn_all_blocking(self) -> None:
        av = self._first_available_client(self._spawn_clients)
        if av is None:
            self.get_logger().warn("SpawnEntity service not available yet; will retry asynchronously.")
            return
        _, spawn_cli = av

        for o in self._obstacles:
            if o.spawned:
                continue
            ok = self._spawn_one_blocking(spawn_cli, o)
            o.spawned = ok

    def _spawn_one_blocking(self, spawn_cli, obs: ObstacleAgent) -> bool:
        sx, sy, sz = obs.size_xyz
        sdf = _make_box_sdf(obs.name, sx, sy, sz, mass=self._mass, kinematic=self._kinematic, rgba=None)

        req = SpawnEntity.Request()
        req.name = obs.name
        req.xml = sdf
        req.robot_namespace = ""
        req.reference_frame = "world"
        spawn_xy = obs.parked if self._safe_spawn_enable else obs.start
        req.initial_pose = Pose(
            position=Point(x=float(spawn_xy[0]), y=float(spawn_xy[1]), z=float(obs.z)),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        try:
            fut = spawn_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            if not fut.done():
                self.get_logger().warn(f"Spawn timeout for {obs.name}")
                return False
            res = fut.result()
            if getattr(res, "success", False):
                if self._safe_spawn_enable:
                    obs.active = False
                    obs.activate_not_before = _now() + self._safe_spawn_arm_delay_sec
                else:
                    obs.active = True
                self.get_logger().info(f"Spawned obstacle: {obs.name}")
                return True
            msg = str(getattr(res, "status_message", ""))
            if "already" in msg.lower() and "exist" in msg.lower():
                self.get_logger().info(f"Obstacle already exists, reuse: {obs.name}")
                return True
            self.get_logger().warn(f"Spawn failed for {obs.name}: {msg}")
            return False
        except Exception as e:
            self.get_logger().warn(f"Spawn exception for {obs.name}: {e}")
            return False

    def _tick_spawn_async(self, now: float) -> None:
        # all spawned?
        if all(o.spawned for o in self._obstacles):
            return

        # throttle
        retry_period = float(self.get_parameter("spawn_retry_period_sec").value)
        if now - self._last_spawn_attempt_time < retry_period:
            return

        av = self._first_available_client(self._spawn_clients)
        if av is None:
            if not self._warned_spawn:
                self._warned_spawn = True
                self.get_logger().warn("SpawnEntity service not available (will keep retrying).")
            return
        _, spawn_cli = av

        # pending request?
        if self._pending_spawn_future is not None:
            fut = self._pending_spawn_future
            timeout = float(self.get_parameter("spawn_call_timeout_sec").value)
            if (not fut.done()) and (now - self._pending_spawn_sent_time > timeout):
                self.get_logger().warn(f"Spawn request timeout (> {timeout:.1f}s), retrying: {self._pending_spawn_name}")
                self._pending_spawn_future = None
                self._pending_spawn_name = None
                return

            if fut.done():
                name = self._pending_spawn_name
                self._pending_spawn_future = None
                self._pending_spawn_name = None
                try:
                    res = fut.result()
                    success = bool(getattr(res, "success", False))
                    msg = str(getattr(res, "status_message", ""))
                except Exception as e:
                    success = False
                    msg = str(e)

                if name is not None:
                    for o in self._obstacles:
                        if o.name == name:
                            if success or ("already" in msg.lower() and "exist" in msg.lower()):
                                o.spawned = True
                                if self._safe_spawn_enable:
                                    o.active = False
                                    o.activate_not_before = _now() + self._safe_spawn_arm_delay_sec
                                else:
                                    o.active = True
                                self.get_logger().info(f"Spawn OK (async): {name}")
                            else:
                                self.get_logger().warn(f"Spawn failed (async) {name}: {msg}")
                            break
            return

        # send next spawn
        target = next((o for o in self._obstacles if not o.spawned), None)
        if target is None:
            return

        sx, sy, sz = target.size_xyz
        sdf = _make_box_sdf(target.name, sx, sy, sz, mass=self._mass, kinematic=self._kinematic, rgba=None)

        req = SpawnEntity.Request()
        req.name = target.name
        req.xml = sdf
        req.robot_namespace = ""
        req.reference_frame = "world"
        spawn_xy = target.parked if self._safe_spawn_enable else target.start
        req.initial_pose = Pose(
            position=Point(x=float(spawn_xy[0]), y=float(spawn_xy[1]), z=float(target.z)),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        self._pending_spawn_name = target.name
        self._pending_spawn_future = spawn_cli.call_async(req)
        self._pending_spawn_sent_time = now
        self._last_spawn_attempt_time = now

    def _set_state_async(self, name: str, x: float, y: float, z: float) -> None:
        # Drop finished futures
        fut = self._pending_set_futures.get(name)
        if fut is not None and getattr(fut, "done", lambda: False)():
            self._pending_set_futures.pop(name, None)

        # avoid backlog
        if name in self._pending_set_futures:
            return

        av = self._first_available_client(self._set_state_clients)
        if av is None:
            if not self._warned_set:
                self._warned_set = True
                self.get_logger().warn("SetEntityState service not available.")
            return
        _, set_cli = av

        st = EntityState()
        st.name = name
        st.reference_frame = "world"
        st.pose.position.x = float(x)
        st.pose.position.y = float(y)
        st.pose.position.z = float(z)
        st.pose.orientation.w = 1.0

        req = SetEntityState.Request()
        req.state = st
        self._pending_set_futures[name] = set_cli.call_async(req)

    # ------------------------------ motion update ------------------------------

    def _apply_uncertainty(self, obs: ObstacleAgent, pos: Tuple[float, float]) -> Tuple[float, float]:
        x, y = pos
        n = obs.noise

        # wiggle (sinusoidal)
        if n.wiggle_amp > 1e-6 and n.wiggle_freq > 1e-6:
            s = n.wiggle_amp * math.sin(2.0 * math.pi * n.wiggle_freq * obs.t_total + n.wiggle_phase)
            x += obs.wiggle_axis[0] * s
            y += obs.wiggle_axis[1] * s

        # gaussian noise (small)
        if n.gauss_std > 1e-9:
            x += random.gauss(0.0, n.gauss_std)
            y += random.gauss(0.0, n.gauss_std)

        return (x, y)

    def _update_line(self, obs: ObstacleAgent, now: float, dt: float) -> Tuple[float, float]:
        obs.t_total += dt

        # pause handling
        if not obs.moving:
            if now >= obs.next_start_time:
                obs.moving = True
                # restart: when resuming at end, teleport back to start
                if obs.loop == "restart" and obs.t >= 1.0:
                    obs.t = 0.0
            else:
                pos = (_lerp(obs.start[0], obs.end[0], obs.t), _lerp(obs.start[1], obs.end[1], obs.t))
                return self._apply_uncertainty(obs, pos)

        dx = obs.end[0] - obs.start[0]
        dy = obs.end[1] - obs.start[1]
        length = max(1e-6, _hypot(dx, dy))

        if obs.speed <= 1e-6 or length <= 1e-6:
            pos = obs.start
            return self._apply_uncertainty(obs, pos)

        step = (obs.speed * dt) / length

        if obs.loop == "pingpong":
            obs.t += obs.direction * step
            if obs.t >= 1.0:
                obs.t = 1.0
                obs.direction = -1
                obs.moving = False
                obs.next_start_time = now + random.uniform(obs.pause_end_min, obs.pause_end_max)
            elif obs.t <= 0.0:
                obs.t = 0.0
                obs.direction = 1
                obs.moving = False
                obs.next_start_time = now + random.uniform(obs.pause_start_min, obs.pause_start_max)
        elif obs.loop == "stay":
            obs.t += step
            if obs.t >= 1.0:
                obs.t = 1.0
                obs.moving = False
                obs.next_start_time = float("inf")
        else:  # restart
            obs.t += step
            if obs.t >= 1.0:
                obs.t = 1.0
                obs.moving = False
                obs.next_start_time = now + random.uniform(obs.pause_end_min, obs.pause_end_max)

        pos = (_lerp(obs.start[0], obs.end[0], obs.t), _lerp(obs.start[1], obs.end[1], obs.t))
        return self._apply_uncertainty(obs, pos)

    def _choose_random_target_near(self, cur: Tuple[float, float], step: float) -> Tuple[float, float]:
        assert self._plan is not None

        pts = self._plan.points
        if not pts:
            xmin, ymin, xmax, ymax = self._bbox
            return (random.uniform(xmin, xmax), random.uniform(ymin, ymax))

        for _ in range(40):
            cand = random.choice(pts)
            if _hypot(cand[0] - cur[0], cand[1] - cur[1]) <= step:
                return cand
        return random.choice(pts)

    def _update_random_walk(self, obs: ObstacleAgent, now: float, dt: float) -> Tuple[float, float]:
        obs.t_total += dt

        # init target
        if obs.start == obs.end:
            obs.end = self._choose_random_target_near(obs.start, obs.step_m)
            obs.t = 0.0

        # pause handling
        if not obs.moving:
            if now >= obs.next_start_time:
                obs.moving = True
            else:
                pos = (_lerp(obs.start[0], obs.end[0], obs.t), _lerp(obs.start[1], obs.end[1], obs.t))
                return self._apply_uncertainty(obs, pos)

        dx = obs.end[0] - obs.start[0]
        dy = obs.end[1] - obs.start[1]
        length = max(1e-6, _hypot(dx, dy))

        if obs.speed <= 1e-6:
            return self._apply_uncertainty(obs, obs.start)

        obs.t += (obs.speed * dt) / length

        if obs.t >= 1.0:
            obs.t = 1.0
            cur = obs.end
            obs.moving = False
            obs.next_start_time = now + random.uniform(obs.pause_end_min, obs.pause_end_max)
            obs.start = cur
            obs.end = self._choose_random_target_near(cur, obs.step_m)
            obs.t = 0.0

        pos = (_lerp(obs.start[0], obs.end[0], obs.t), _lerp(obs.start[1], obs.end[1], obs.t))
        return self._apply_uncertainty(obs, pos)

    def _update_circle(self, obs: ObstacleAgent, now: float, dt: float) -> Tuple[float, float]:
        obs.t_total += dt
        obs.theta += obs.omega * dt
        pos = (obs.center[0] + obs.radius * math.cos(obs.theta), obs.center[1] + obs.radius * math.sin(obs.theta))
        return self._apply_uncertainty(obs, pos)

    # ------------------------------- main timer -------------------------------

    def _on_timer(self) -> None:
        now = _now()
        dt = max(1e-3, now - self._last_update)
        self._last_update = now

        # 1) ensure everything is spawned
        self._tick_spawn_async(now)

        # 2) update motion + set state
        for obs in self._obstacles:
            if not obs.spawned:
                if now >= obs.warn_next_time:
                    obs.warn_next_time = now + 5.0
                    self.get_logger().info(f"Waiting for spawn: {obs.name}")
                continue

            if self._safe_spawn_enable and (not obs.active):
                self._try_activate_obstacle(obs, now)
                continue

            if obs.kind == "line":
                x, y = self._update_line(obs, now, dt)
            elif obs.kind == "random_walk":
                x, y = self._update_random_walk(obs, now, dt)
            elif obs.kind == "circle":
                x, y = self._update_circle(obs, now, dt)
            else:  # static
                x, y = obs.start

            self._set_state_async(obs.name, x, y, obs.z)


def main() -> None:
    rclpy.init()
    node = DynamicObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
