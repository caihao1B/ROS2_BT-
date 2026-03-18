#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import signal
import time
from array import array
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from std_msgs.msg import String
import tf2_ros

try:
    from nav2_msgs.msg import Costmap  # type: ignore
except Exception:  # pragma: no cover
    Costmap = None  # type: ignore

try:
    from coverage_msgs.msg import StrategyDecisionStamped
except Exception:  # pragma: no cover
    StrategyDecisionStamped = None  # type: ignore

try:  # optional
    import psutil  # type: ignore
except Exception:  # pragma: no cover
    psutil = None  # type: ignore


def now_iso() -> str:
    return dt.datetime.utcnow().replace(tzinfo=dt.timezone.utc).isoformat()


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def point_to_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    c1 = vx * wx + vy * wy
    if c1 <= 0.0:
        return math.hypot(px - ax, py - ay)
    c2 = vx * vx + vy * vy
    if c2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    if c1 >= c2:
        return math.hypot(px - bx, py - by)
    t = c1 / c2
    qx = ax + t * vx
    qy = ay + t * vy
    return math.hypot(px - qx, py - qy)


def polyline_length(points: Sequence[Tuple[float, float]]) -> float:
    out = 0.0
    for (ax, ay), (bx, by) in zip(points[:-1], points[1:]):
        out += math.hypot(bx - ax, by - ay)
    return out


def percentile(values: Sequence[float], q: float) -> Optional[float]:
    if not values:
        return None
    xs = sorted(float(v) for v in values)
    if len(xs) == 1:
        return xs[0]
    q = min(100.0, max(0.0, float(q)))
    pos = (len(xs) - 1) * (q / 100.0)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return xs[lo]
    frac = pos - lo
    return xs[lo] * (1.0 - frac) + xs[hi] * frac


def summarize_numeric(values: Sequence[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {'count': 0, 'mean': None, 'median': None, 'p90': None, 'p95': None, 'max': None}
    vals = [float(v) for v in values]
    return {
        'count': len(vals),
        'mean': sum(vals) / len(vals),
        'median': percentile(vals, 50.0),
        'p90': percentile(vals, 90.0),
        'p95': percentile(vals, 95.0),
        'max': max(vals),
    }


def load_plan_metrics(plan_path: str) -> Dict[str, Any]:
    p = Path(plan_path)
    if not plan_path or not p.is_file():
        return {
            'field_meta': {},
            'nominal_plan_length_m': None,
            'segments': [],
            'turn_points': [],
            'segment_points': [],
        }
    try:
        data = json.loads(p.read_text(encoding='utf-8'))
    except Exception:
        return {
            'field_meta': {},
            'nominal_plan_length_m': None,
            'segments': [],
            'turn_points': [],
            'segment_points': [],
        }
    seg_lines: List[Tuple[float, float, float, float]] = []
    turn_points: List[Tuple[float, float]] = []
    segment_points: List[List[Tuple[float, float]]] = []
    nominal_len = 0.0
    for seg in data.get('segments', []):
        pts: List[Tuple[float, float]] = []
        for pose in seg.get('poses', []):
            try:
                pts.append((float(pose.get('x')), float(pose.get('y'))))
            except Exception:
                continue
        if not pts:
            continue
        segment_points.append(pts)
        nominal_len += polyline_length(pts)
        for a, b in zip(pts[:-1], pts[1:]):
            seg_lines.append((a[0], a[1], b[0], b[1]))
        kind = str(seg.get('kind', '')).strip().lower()
        if kind == 'turn':
            turn_points.extend(pts)
    if not turn_points:
        # fallback: use joints where heading changes sharply
        all_pts = [p for seg in segment_points for p in seg]
        for i in range(1, len(all_pts) - 1):
            a = all_pts[i - 1]
            b = all_pts[i]
            c = all_pts[i + 1]
            va = (b[0] - a[0], b[1] - a[1])
            vb = (c[0] - b[0], c[1] - b[1])
            na = math.hypot(*va)
            nb = math.hypot(*vb)
            if na < 1e-6 or nb < 1e-6:
                continue
            ca = (va[0] / na, va[1] / na)
            cb = (vb[0] / nb, vb[1] / nb)
            dot = max(-1.0, min(1.0, ca[0] * cb[0] + ca[1] * cb[1]))
            ang = math.degrees(math.acos(dot))
            if ang >= 20.0:
                turn_points.append(b)
    return {
        'field_meta': data.get('field_meta', {}) or {},
        'nominal_plan_length_m': nominal_len,
        'segments': seg_lines,
        'turn_points': turn_points,
        'segment_points': segment_points,
    }


class GracefulStop:
    stop = False

    @classmethod
    def handler(cls, *_args):
        cls.stop = True


class E1RuntimeLogger(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('e1_runtime_logger')
        self.args = args
        self.output_dir = Path(args.output_dir).resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.summary_path = self.output_dir / 'summary.json'
        self.timeseries_path = self.output_dir / 'timeseries.csv'
        self.events_path = self.output_dir / 'events.jsonl'
        self.decisions_path = self.output_dir / 'decisions.jsonl'
        self.odom_path = self.output_dir / 'odom_samples.csv'
        self.run_meta_path = self.output_dir / 'run_meta.json'
        self.path_path = self.output_dir / 'current_path.jsonl'
        self.resources_path = self.output_dir / 'resources.csv'

        self._events_fp = self.events_path.open('w', encoding='utf-8')
        self._decisions_fp = self.decisions_path.open('w', encoding='utf-8')
        self._paths_fp = self.path_path.open('w', encoding='utf-8')
        self._odom_fp = self.odom_path.open('w', newline='', encoding='utf-8')
        self._odom_writer = csv.writer(self._odom_fp)
        self._odom_writer.writerow(['ros_time_sec', 'wall_iso', 'x', 'y', 'yaw', 'path_length_m'])

        self._ts_fp = self.timeseries_path.open('w', newline='', encoding='utf-8')
        self._ts_writer = csv.writer(self._ts_fp)
        self._ts_writer.writerow([
            'ros_time_sec', 'wall_iso', 'x', 'y', 'yaw',
            'covered_free', 'overlapped_free', 'total_free', 'coverage_ratio', 'overlap_ratio',
            'swept_area_m2', 'overlap_area_m2', 'path_length_m', 'tracking_error_m',
            'global_cost', 'local_cost', 'latest_decision_mode', 'event_count'
        ])

        self._res_fp = self.resources_path.open('w', newline='', encoding='utf-8')
        self._res_writer = csv.writer(self._res_fp)
        self._res_writer.writerow([
            'wall_iso', 'ros_time_sec', 'system_cpu_percent', 'system_mem_percent',
            'gazebo_cpu_percent', 'gazebo_mem_mb',
            'nav2_cpu_percent', 'nav2_mem_mb',
            'mission_cpu_percent', 'mission_mem_mb'
        ])

        # map / coverage state
        self._map: Optional[OccupancyGrid] = None
        self._width = 0
        self._height = 0
        self._res = 0.0
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._free_mask: Optional[bytearray] = None
        self._cover_counts: Optional[array] = None
        self._total_free = 0
        self._covered_free = 0
        self._overlapped_free = 0

        # runtime state
        self._event_count = 0
        self._event_counts: Dict[str, int] = {}
        self._decision_counts: Dict[str, int] = {}
        self._latest_decision_mode = 'NONE'
        self._latest_decision: Dict[str, Any] = {}
        self._start_ros_time: Optional[float] = None
        self._end_ros_time: Optional[float] = None
        self._first_wall_time: Optional[str] = None
        self._end_reason: str = ''
        self._success: bool = False
        self._done: bool = False
        self._aborted: bool = False
        self._mission_complete_event: str = ''
        self._last_xy: Optional[Tuple[float, float]] = None
        self._last_yaw: float = 0.0
        self._path_length_m: float = 0.0
        self._latest_path_info: Dict[str, Any] = {}
        self._latest_odom: Optional[Odometry] = None
        self._wall_started_iso = now_iso()

        # plan-derived metrics
        plan_metrics = load_plan_metrics(args.plan_json)
        self._field_meta: Dict[str, Any] = dict(plan_metrics['field_meta'])
        self._nominal_plan_length_m: Optional[float] = plan_metrics['nominal_plan_length_m']
        self._plan_segments: List[Tuple[float, float, float, float]] = list(plan_metrics['segments'])
        self._turn_points: List[Tuple[float, float]] = list(plan_metrics['turn_points'])
        self._tracking_errors: List[float] = []
        self._turn_tracking_errors: List[float] = []

        # costmap state
        self._global_costmap = None
        self._local_costmap = None
        self._global_cost_values: List[float] = []
        self._local_cost_values: List[float] = []
        self._global_high_cost_samples = 0
        self._local_high_cost_samples = 0
        self._sample_count = 0

        # resources
        self._resource_monitor_enabled = bool(args.enable_resource_monitor and psutil is not None)
        self._last_resource_sample_wall = 0.0
        self._resource_stats: Dict[str, List[float]] = {
            'system_cpu_percent': [],
            'system_mem_percent': [],
            'gazebo_cpu_percent': [],
            'gazebo_mem_mb': [],
            'nav2_cpu_percent': [],
            'nav2_mem_mb': [],
            'mission_cpu_percent': [],
            'mission_mem_mb': [],
        }
        if self._resource_monitor_enabled:
            try:
                psutil.cpu_percent(interval=None)
                for proc in psutil.process_iter(['name']):
                    try:
                        proc.cpu_percent(interval=None)
                    except Exception:
                        pass
            except Exception:
                self._resource_monitor_enabled = False

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._write_meta()

        qos_map = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, args.map_topic, self._map_cb, qos_map)
        self.create_subscription(String, args.events_topic, self._event_cb, 50)
        self.create_subscription(String, args.recovery_topic, self._event_cb, 50)
        self.create_subscription(NavPath, args.current_path_topic, self._path_cb, 10)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 50)
        if StrategyDecisionStamped is not None:
            self.create_subscription(StrategyDecisionStamped, args.decision_topic, self._decision_cb, 20)
        else:
            self.get_logger().warn('coverage_msgs unavailable: decision subscription disabled')
        if Costmap is not None:
            self.create_subscription(Costmap, args.global_costmap_topic, self._global_costmap_cb, 10)
            self.create_subscription(Costmap, args.local_costmap_topic, self._local_costmap_cb, 10)
        else:
            self.get_logger().warn('nav2_msgs unavailable: costmap subscriptions disabled')

        self._timer = self.create_timer(max(0.05, float(args.sample_period)), self._sample_once)

    # ----------------------------
    # setup / teardown
    # ----------------------------
    def _write_meta(self) -> None:
        payload = {
            'created_at': now_iso(),
            'method': self.args.method,
            'map_id': self.args.map_id,
            'planner': self.args.planner,
            'scenario_tag': self.args.scenario_tag,
            'notes': self.args.notes,
            'plan_json': self.args.plan_json,
            'map_yaml': self.args.map_yaml,
            'world_file': self.args.world_file,
            'tool_radius_m': self.args.tool_radius,
            'sample_period_sec': self.args.sample_period,
            'events_topic': self.args.events_topic,
            'recovery_topic': self.args.recovery_topic,
            'decision_topic': self.args.decision_topic,
            'odom_topic': self.args.odom_topic,
            'global_costmap_topic': self.args.global_costmap_topic,
            'local_costmap_topic': self.args.local_costmap_topic,
            'base_frame': self.args.base_frame,
            'turn_zone_radius_m': self.args.turn_zone_radius,
            'resource_monitor_enabled': self._resource_monitor_enabled,
        }
        with self.run_meta_path.open('w', encoding='utf-8') as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

    def close_files(self) -> None:
        for fp in [self._events_fp, self._decisions_fp, self._paths_fp, self._odom_fp, self._ts_fp, self._res_fp]:
            try:
                fp.flush()
                fp.close()
            except Exception:
                pass

    def destroy_node(self):
        try:
            self.write_summary(final=True)
        except Exception:
            pass
        self.close_files()
        super().destroy_node()

    # ----------------------------
    # map / costmap handling
    # ----------------------------
    def _map_cb(self, msg: OccupancyGrid) -> None:
        if self._map is not None:
            return
        self._map = msg
        self._width = int(msg.info.width)
        self._height = int(msg.info.height)
        self._res = float(msg.info.resolution)
        self._origin_x = float(msg.info.origin.position.x)
        self._origin_y = float(msg.info.origin.position.y)
        n = self._width * self._height
        free = bytearray(n)
        total_free = 0
        for i, v in enumerate(msg.data):
            if v == 0:
                free[i] = 1
                total_free += 1
        self._free_mask = free
        self._cover_counts = array('H', [0] * n)
        self._total_free = total_free
        self.get_logger().info(f'e1 logger latched /map: {self._width}x{self._height}, free={total_free}')

    def _global_costmap_cb(self, msg) -> None:
        self._global_costmap = msg

    def _local_costmap_cb(self, msg) -> None:
        self._local_costmap = msg

    def _xy_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        mx = int((x - self._origin_x) / self._res)
        my = int((y - self._origin_y) / self._res)
        if mx < 0 or my < 0 or mx >= self._width or my >= self._height:
            return None
        return (mx, my)

    def _costmap_meta(self, costmap_msg):
        if costmap_msg is None:
            return None
        meta = getattr(costmap_msg, 'metadata', None)
        if meta is None:
            meta = getattr(costmap_msg, 'meta_data', None)
        return meta

    def _costmap_cost_at(self, costmap_msg, x: float, y: float) -> Optional[int]:
        meta = self._costmap_meta(costmap_msg)
        if meta is None:
            return None
        try:
            res = float(getattr(meta, 'resolution'))
            origin = getattr(meta, 'origin')
            ox = float(origin.position.x)
            oy = float(origin.position.y)
            size_x = int(getattr(meta, 'size_x', getattr(meta, 'width')))
            size_y = int(getattr(meta, 'size_y', getattr(meta, 'height')))
            mx = int((x - ox) / res)
            my = int((y - oy) / res)
            if mx < 0 or my < 0 or mx >= size_x or my >= size_y:
                return None
            data = getattr(costmap_msg, 'data')
            return int(data[my * size_x + mx])
        except Exception:
            return None

    # ----------------------------
    # topic callbacks
    # ----------------------------
    def _event_cb(self, msg: String) -> None:
        raw = msg.data
        try:
            ev = json.loads(raw)
        except Exception:
            ev = {'type': 'raw_string', 'raw': raw}
        ev_type = str(ev.get('type', 'unknown'))
        self._event_count += 1
        self._event_counts[ev_type] = self._event_counts.get(ev_type, 0) + 1
        line = {'wall_iso': now_iso(), 'event': ev}
        self._events_fp.write(json.dumps(line, ensure_ascii=False) + '\n')
        self._events_fp.flush()

        ros_t = self._current_ros_time_sec()
        if ros_t is not None and self._start_ros_time is None:
            self._start_ros_time = ros_t
            self._first_wall_time = line['wall_iso']

        if ev_type in {'executor_done', 'mission_complete'}:
            self._success = True
            self._done = True
            self._mission_complete_event = ev_type
            self._end_reason = ev_type
            self._end_ros_time = ros_t if ros_t is not None else self._end_ros_time
        elif ev_type in {'executor_aborted'}:
            self._aborted = True
            self._done = True
            self._end_reason = ev_type
            self._end_ros_time = ros_t if ros_t is not None else self._end_ros_time

    def _decision_cb(self, msg) -> None:
        try:
            d = msg.decision
            rec = {
                'wall_iso': now_iso(),
                'ros_time_sec': self._current_ros_time_sec(),
                'mode': str(d.mode),
                'stop_now': bool(d.stop_now),
                'slowdown_ratio': float(d.slowdown_ratio),
                't_clear': float(d.t_clear),
                't_block': float(d.t_block),
                'affected_segment_id': str(d.affected_segment_id),
                'recommended_cluster_id': str(d.recommended_cluster_id),
            }
            self._latest_decision_mode = rec['mode']
            self._latest_decision = rec
            self._decision_counts[rec['mode']] = self._decision_counts.get(rec['mode'], 0) + 1
            self._decisions_fp.write(json.dumps(rec, ensure_ascii=False) + '\n')
            self._decisions_fp.flush()
        except Exception as e:
            self.get_logger().warn(f'decision parse failed: {e}')

    def _path_cb(self, msg: NavPath) -> None:
        try:
            length = 0.0
            poses = msg.poses
            for a, b in zip(poses[:-1], poses[1:]):
                ax = float(a.pose.position.x)
                ay = float(a.pose.position.y)
                bx = float(b.pose.position.x)
                by = float(b.pose.position.y)
                length += math.hypot(bx - ax, by - ay)
            rec = {
                'wall_iso': now_iso(),
                'ros_time_sec': self._current_ros_time_sec(),
                'frame_id': msg.header.frame_id,
                'poses': len(msg.poses),
                'length_m': length,
            }
            self._latest_path_info = rec
            self._paths_fp.write(json.dumps(rec, ensure_ascii=False) + '\n')
            self._paths_fp.flush()
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg

    # ----------------------------
    # sampling / metrics
    # ----------------------------
    def _current_ros_time_sec(self) -> Optional[float]:
        try:
            nsec = self.get_clock().now().nanoseconds
            if nsec <= 0:
                return None
            return float(nsec) * 1.0e-9
        except Exception:
            return None

    def _lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform('map', self.args.base_frame, Time())
            x = float(tf.transform.translation.x)
            y = float(tf.transform.translation.y)
            q = tf.transform.rotation
            yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
            return (x, y, yaw)
        except Exception:
            return None

    def _mark_coverage(self, x: float, y: float) -> None:
        if self._map is None or self._free_mask is None or self._cover_counts is None:
            return
        cell = self._xy_to_cell(x, y)
        if cell is None:
            return
        radius_m = float(self.args.tool_radius)
        r_cells = max(1, int(radius_m / self._res))
        cx, cy = cell
        for dy in range(-r_cells, r_cells + 1):
            yy = cy + dy
            if yy < 0 or yy >= self._height:
                continue
            for dx in range(-r_cells, r_cells + 1):
                xx = cx + dx
                if xx < 0 or xx >= self._width:
                    continue
                if (dx * dx + dy * dy) * (self._res * self._res) > (radius_m * radius_m):
                    continue
                idx = yy * self._width + xx
                if self._free_mask[idx] != 1:
                    continue
                prev = self._cover_counts[idx]
                self._cover_counts[idx] = prev + 1
                if prev == 0:
                    self._covered_free += 1
                elif prev == 1:
                    self._overlapped_free += 1

    def _tracking_error(self, x: float, y: float) -> Optional[float]:
        if not self._plan_segments:
            return None
        best = min(point_to_segment_distance(x, y, ax, ay, bx, by) for ax, ay, bx, by in self._plan_segments)
        return float(best)

    def _in_turn_zone(self, x: float, y: float) -> bool:
        if not self._turn_points:
            return False
        rr = float(self.args.turn_zone_radius) ** 2
        for tx, ty in self._turn_points:
            if (x - tx) ** 2 + (y - ty) ** 2 <= rr:
                return True
        return False

    def _sample_resources(self, ros_t: Optional[float]) -> None:
        if not self._resource_monitor_enabled:
            return
        now_wall = time.monotonic()
        if (now_wall - self._last_resource_sample_wall) < max(0.2, float(self.args.resource_period)):
            return
        self._last_resource_sample_wall = now_wall
        try:
            system_cpu = float(psutil.cpu_percent(interval=None))
            system_mem = float(psutil.virtual_memory().percent)
        except Exception:
            return

        groups = {
            'gazebo': ['gzserver', 'gzclient', 'gazebo'],
            'nav2': ['nav2', 'map_server', 'lifecycle_manager', 'controller_server', 'planner_server', 'bt_navigator', 'component_container'],
            'mission': ['coverage_supervisor', 'strategy_selector', 'cv_predictor', 'scan_tracker', 'dynamic_obstacles', 'coverage_executor', 'coverage_executor_nobt_rejoin', 'coverage_executor_guarded', 'd1_advisor', 'd2_advisor', 'region_state'],
        }
        stats = {k: {'cpu': 0.0, 'mem': 0.0} for k in groups}
        for proc in psutil.process_iter(['name', 'cmdline', 'memory_info']):
            try:
                name = proc.info.get('name') or ''
                cmd = ' '.join(proc.info.get('cmdline') or [])
                hay = f'{name} {cmd}'.lower()
                cpu = float(proc.cpu_percent(interval=None))
                mem_mb = float(proc.info.get('memory_info').rss) / (1024.0 * 1024.0)
            except Exception:
                continue
            for g, keys in groups.items():
                if any(k.lower() in hay for k in keys):
                    stats[g]['cpu'] += cpu
                    stats[g]['mem'] += mem_mb
                    break

        row = [
            now_iso(),
            f'{ros_t:.6f}' if ros_t is not None else '',
            f'{system_cpu:.3f}', f'{system_mem:.3f}',
            f"{stats['gazebo']['cpu']:.3f}", f"{stats['gazebo']['mem']:.3f}",
            f"{stats['nav2']['cpu']:.3f}", f"{stats['nav2']['mem']:.3f}",
            f"{stats['mission']['cpu']:.3f}", f"{stats['mission']['mem']:.3f}",
        ]
        self._res_writer.writerow(row)
        self._res_fp.flush()

        self._resource_stats['system_cpu_percent'].append(system_cpu)
        self._resource_stats['system_mem_percent'].append(system_mem)
        self._resource_stats['gazebo_cpu_percent'].append(stats['gazebo']['cpu'])
        self._resource_stats['gazebo_mem_mb'].append(stats['gazebo']['mem'])
        self._resource_stats['nav2_cpu_percent'].append(stats['nav2']['cpu'])
        self._resource_stats['nav2_mem_mb'].append(stats['nav2']['mem'])
        self._resource_stats['mission_cpu_percent'].append(stats['mission']['cpu'])
        self._resource_stats['mission_mem_mb'].append(stats['mission']['mem'])

    def _sample_once(self) -> None:
        pose = self._lookup_robot_pose()
        if pose is None:
            return
        x, y, yaw = pose
        ros_t = self._current_ros_time_sec()
        wall = now_iso()

        if self._start_ros_time is None and ros_t is not None:
            self._start_ros_time = ros_t
            self._first_wall_time = wall

        self._mark_coverage(x, y)
        self._sample_count += 1

        if self._last_xy is not None:
            self._path_length_m += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        self._last_yaw = yaw

        tracking_error = self._tracking_error(x, y)
        if tracking_error is not None:
            self._tracking_errors.append(tracking_error)
            if self._in_turn_zone(x, y):
                self._turn_tracking_errors.append(tracking_error)

        g_cost = self._costmap_cost_at(self._global_costmap, x, y)
        l_cost = self._costmap_cost_at(self._local_costmap, x, y)
        if g_cost is not None:
            self._global_cost_values.append(float(g_cost))
            if g_cost >= self.args.high_cost_threshold:
                self._global_high_cost_samples += 1
        if l_cost is not None:
            self._local_cost_values.append(float(l_cost))
            if l_cost >= self.args.high_cost_threshold:
                self._local_high_cost_samples += 1

        if ros_t is not None:
            self._odom_writer.writerow([f'{ros_t:.6f}', wall, f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}', f'{self._path_length_m:.6f}'])
            self._odom_fp.flush()

        cov_ratio = (self._covered_free / self._total_free) if self._total_free > 0 else 0.0
        ov_ratio = (self._overlapped_free / self._covered_free) if self._covered_free > 0 else 0.0
        cell_area = (self._res * self._res) if self._res > 0.0 else 0.0
        swept_area = self._covered_free * cell_area
        overlap_area = self._overlapped_free * cell_area
        self._ts_writer.writerow([
            f'{ros_t:.6f}' if ros_t is not None else '',
            wall,
            f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}',
            self._covered_free, self._overlapped_free, self._total_free,
            f'{cov_ratio:.6f}', f'{ov_ratio:.6f}',
            f'{swept_area:.6f}', f'{overlap_area:.6f}',
            f'{self._path_length_m:.6f}',
            f'{tracking_error:.6f}' if tracking_error is not None else '',
            g_cost if g_cost is not None else '',
            l_cost if l_cost is not None else '',
            self._latest_decision_mode,
            self._event_count,
        ])
        self._ts_fp.flush()

        self._sample_resources(ros_t)

    # ----------------------------
    # outputs
    # ----------------------------
    def write_summary(self, final: bool = False) -> None:
        cov_ratio = (self._covered_free / self._total_free) if self._total_free > 0 else 0.0
        ov_ratio = (self._overlapped_free / self._covered_free) if self._covered_free > 0 else 0.0
        end_ros = self._end_ros_time if self._end_ros_time is not None else self._current_ros_time_sec()
        start_ros = self._start_ros_time
        duration = None
        if start_ros is not None and end_ros is not None:
            duration = max(0.0, float(end_ros - start_ros))
        cell_area = (self._res * self._res) if self._res > 0.0 else 0.0
        swept_area = self._covered_free * cell_area
        overlap_area = self._overlapped_free * cell_area
        total_free_area = self._total_free * cell_area
        target_area = None
        for k in ('coverage_area_m2', 'driveable_area_m2', 'field_area_m2'):
            if k in self._field_meta:
                try:
                    target_area = float(self._field_meta[k])
                    break
                except Exception:
                    pass
        resource_summary = {k: summarize_numeric(v) for k, v in self._resource_stats.items() if v}
        payload = {
            'written_at': now_iso(),
            'final': bool(final),
            'method': self.args.method,
            'map_id': self.args.map_id,
            'planner': self.args.planner,
            'scenario_tag': self.args.scenario_tag,
            'notes': self.args.notes,
            'plan_json': self.args.plan_json,
            'map_yaml': self.args.map_yaml,
            'world_file': self.args.world_file,
            'start_ros_time_sec': start_ros,
            'end_ros_time_sec': end_ros,
            'duration_sec': duration,
            'success': bool(self._success),
            'aborted': bool(self._aborted),
            'end_reason': self._end_reason,
            'mission_complete_event': self._mission_complete_event,
            'path_length_m': float(self._path_length_m),
            'nominal_plan_length_m': self._nominal_plan_length_m,
            'path_vs_nominal_ratio': (float(self._path_length_m) / float(self._nominal_plan_length_m)) if self._nominal_plan_length_m not in (None, 0, 0.0) else None,
            'coverage': {
                'covered_free': int(self._covered_free),
                'overlapped_free': int(self._overlapped_free),
                'total_free': int(self._total_free),
                'coverage_ratio': cov_ratio,
                'overlap_ratio': ov_ratio,
                'covered_area_m2': swept_area,
                'overlap_area_m2': overlap_area,
                'total_free_area_m2': total_free_area,
                'uncovered_free_area_m2': max(0.0, total_free_area - swept_area),
                'target_area_m2': target_area,
                'swept_vs_target_ratio': (swept_area / target_area) if target_area not in (None, 0, 0.0) else None,
            },
            'tracking_error': summarize_numeric(self._tracking_errors),
            'turn_tracking_error': summarize_numeric(self._turn_tracking_errors),
            'global_cost': {
                **summarize_numeric(self._global_cost_values),
                'high_cost_threshold': int(self.args.high_cost_threshold),
                'high_cost_ratio': (self._global_high_cost_samples / len(self._global_cost_values)) if self._global_cost_values else None,
            },
            'local_cost': {
                **summarize_numeric(self._local_cost_values),
                'high_cost_threshold': int(self.args.high_cost_threshold),
                'high_cost_ratio': (self._local_high_cost_samples / len(self._local_cost_values)) if self._local_cost_values else None,
            },
            'event_count': int(self._event_count),
            'event_counts': self._event_counts,
            'decision_counts': self._decision_counts,
            'latest_decision': self._latest_decision,
            'latest_path_info': self._latest_path_info,
            'wall_started_iso': self._wall_started_iso,
            'first_active_wall_iso': self._first_wall_time,
            'sample_count': int(self._sample_count),
            'field_meta': self._field_meta,
            'resource_monitor': {
                'enabled': bool(self._resource_monitor_enabled),
                'stats': resource_summary,
            },
        }
        with self.summary_path.open('w', encoding='utf-8') as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='Runtime logger for three-terminal experiments')
    p.add_argument('--output-dir', required=True)
    p.add_argument('--method', default='bt_full')
    p.add_argument('--map-id', default='')
    p.add_argument('--planner', default='')
    p.add_argument('--scenario-tag', default='nominal')
    p.add_argument('--notes', default='')
    p.add_argument('--plan-json', default='')
    p.add_argument('--map-yaml', default='')
    p.add_argument('--world-file', default='')
    p.add_argument('--tool-radius', type=float, default=0.20)
    p.add_argument('--sample-period', type=float, default=0.10)
    p.add_argument('--resource-period', type=float, default=1.0)
    p.add_argument('--base-frame', default='base_link')
    p.add_argument('--map-topic', default='/map')
    p.add_argument('--events-topic', default='/coverage/events')
    p.add_argument('--recovery-topic', default='/coverage/recovery_event')
    p.add_argument('--decision-topic', default='/coverage/strategy_decision')
    p.add_argument('--current-path-topic', default='/coverage/current_path')
    p.add_argument('--odom-topic', default='/odom')
    p.add_argument('--global-costmap-topic', default='/global_costmap/costmap_raw')
    p.add_argument('--local-costmap-topic', default='/local_costmap/costmap_raw')
    p.add_argument('--high-cost-threshold', type=int, default=200)
    p.add_argument('--turn-zone-radius', type=float, default=0.8)
    p.add_argument('--enable-resource-monitor', action='store_true')
    p.add_argument('--auto-exit-on-complete', action='store_true')
    return p.parse_args()


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, GracefulStop.handler)
    signal.signal(signal.SIGTERM, GracefulStop.handler)

    rclpy.init(args=None)
    node = E1RuntimeLogger(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not GracefulStop.stop:
            executor.spin_once(timeout_sec=0.1)
            if node._done and args.auto_exit_on_complete:
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.write_summary(final=True)
        except Exception:
            pass
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
