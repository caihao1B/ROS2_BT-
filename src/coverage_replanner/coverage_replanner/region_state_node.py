import json
import math
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener  # type: ignore
try:
    from tf2_ros import TransformException  # type: ignore
except Exception:  # pragma: no cover
    TransformException = Exception  # type: ignore

from coverage_msgs.msg import StrategyDecisionStamped, ObstacleTrackArray
from coverage_replanner.common import GridSource, astar_cost, bbox_from_points, load_plan_clusters, occupancy_stats_in_bbox

try:
    from nav2_msgs.msg import Costmap  # type: ignore
except Exception:  # pragma: no cover
    Costmap = None  # type: ignore


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _tf2d_apply(tx: float, ty: float, yaw: float, x: float, y: float) -> Tuple[float, float]:
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return tx + cy * x - sy * y, ty + sy * x + cy * y


def _stamp_to_sec(stamp) -> float:
    try:
        return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)
    except Exception:
        return 0.0


def _path_overlap_len(path: Path, ox: float, oy: float, radius: float) -> float:
    if path is None or len(path.poses) < 2 or radius <= 0.0:
        return 0.0
    total = 0.0
    for a, b in zip(path.poses[:-1], path.poses[1:]):
        ax = float(a.pose.position.x); ay = float(a.pose.position.y)
        bx = float(b.pose.position.x); by = float(b.pose.position.y)
        seg = math.hypot(bx - ax, by - ay)
        if seg <= 1.0e-6:
            continue
        # inline point-to-segment distance to keep this module self-contained
        vx = bx - ax; vy = by - ay
        wx = ox - ax; wy = oy - ay
        c1 = vx * wx + vy * wy
        if c1 <= 0.0:
            d = math.hypot(ox - ax, oy - ay)
        else:
            c2 = vx * vx + vy * vy
            if c2 <= 1.0e-9:
                d = math.hypot(ox - ax, oy - ay)
            else:
                t = max(0.0, min(1.0, c1 / c2))
                px = ax + t * vx
                py = ay + t * vy
                d = math.hypot(ox - px, oy - py)
        if d <= radius:
            total += seg
    return total


class RegionStateNode(Node):
    def __init__(self):
        super().__init__('coverage_region_state')

        self.declare_parameter('plan_path', '')
        self.declare_parameter('decision_topic', '/coverage/strategy_decision')
        self.declare_parameter('current_path_topic', '/coverage/current_path')
        self.declare_parameter('tracks_topic', '/coverage/obstacles/tracks')
        self.declare_parameter('recovery_event_topic', '/coverage/recovery_event')
        self.declare_parameter('summary_topic', '/coverage/region_state/summary')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap_raw')
        self.declare_parameter('path_frame', 'map')
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('cluster_bbox_pad_m', 0.60)
        self.declare_parameter('track_timeout_sec', 1.5)
        self.declare_parameter('current_path_stale_sec', 3.0)
        self.declare_parameter('persistent_speed_thresh', 0.08)
        self.declare_parameter('persistent_hold_sec', 1.2)
        self.declare_parameter('persistent_path_overlap_inflate_m', 0.18)
        self.declare_parameter('persistent_path_overlap_min_m', 0.75)
        self.declare_parameter('occ_thresh', 50)
        self.declare_parameter('d1_score_threshold', 4.5)
        self.declare_parameter('d2_score_threshold', 4.5)

        plan_path = str(self.get_parameter('plan_path').value)
        if not plan_path:
            raise RuntimeError('region_state plan_path empty')
        self._clusters, self._cluster_bboxes, self._cluster_counts = load_plan_clusters(plan_path)
        pad = float(self.get_parameter('cluster_bbox_pad_m').value)
        for cid, bbox in list(self._cluster_bboxes.items()):
            self._cluster_bboxes[cid] = (bbox[0] - pad, bbox[1] - pad, bbox[2] + pad, bbox[3] + pad)

        self._state: Dict[int, Dict[str, Any]] = {}
        now = time.monotonic()
        for cid, count in self._cluster_counts.items():
            self._state[cid] = {
                'cluster_id': int(cid),
                'remaining_segments': int(count),
                'cleanup_backlog': 0,
                'n_abort': 0,
                'n_c_fail': 0,
                'n_c_refresh': 0,
                'n_d1_fail': 0,
                'n_d_fail': 0,
                'n_d_no_candidates': 0,
                'n_d_attempts_exceeded': 0,
                'n_d2_fail': 0,
                'n_cluster_jump': 0,
                'n_path_rejected': 0,
                'blocked_time_sec': 0.0,
                'last_update_mono': float(now),
                'recent_d_requests': [],
                'all_entries_no_route': False,
                'last_event_seg': None,
            }

        self._grid = GridSource()
        self._tracks: Dict[str, Dict[str, Any]] = {}
        self._current_path: Optional[Path] = None
        self._current_cluster_id: Optional[int] = None
        self._last_decision_mode: str = 'NONE'
        self._last_decision_t_mono: float = now

        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)

        qos_map = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, str(self.get_parameter('map_topic').value), self._cb_map, qos_map)
        if Costmap is not None:
            try:
                self.create_subscription(Costmap, str(self.get_parameter('costmap_topic').value), self._cb_costmap, 10)
            except Exception:
                self.get_logger().warn('[region_state] failed to subscribe costmap topic, fallback to /map only')

        self.create_subscription(StrategyDecisionStamped, str(self.get_parameter('decision_topic').value), self._cb_decision, 10)
        self.create_subscription(Path, str(self.get_parameter('current_path_topic').value), self._cb_path, 10)
        self.create_subscription(ObstacleTrackArray, str(self.get_parameter('tracks_topic').value), self._cb_tracks, 10)
        self.create_subscription(String, str(self.get_parameter('recovery_event_topic').value), self._cb_event, 50)
        self._pub = self.create_publisher(String, str(self.get_parameter('summary_topic').value), 10)
        self._timer = self.create_timer(float(self.get_parameter('publish_period_sec').value), self._publish_summary)

        self.get_logger().info(f'[region_state] plan={plan_path} clusters={sorted(self._state.keys())}')

    def _lookup_tf2d(self, target_frame: str, source_frame: str, stamp) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf.lookup_transform(target_frame, source_frame, rclpy.time.Time.from_msg(stamp))
        except Exception:
            try:
                tf = self._tf.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            except TransformException:
                return None
            except Exception:
                return None
        try:
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            q = tf.transform.rotation
            yaw = _yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
            return tx, ty, yaw
        except Exception:
            return None

    def _cb_map(self, msg: OccupancyGrid):
        # keep /map only until a fresher costmap arrives; costmap callback will overwrite it.
        if self._grid.grid is None:
            self._grid.update_from_occgrid(msg)
            self.get_logger().info('[region_state] latched /map received')

    def _cb_costmap(self, msg):
        self._grid.update_from_costmap(msg)

    def _cb_path(self, msg: Path):
        self._current_path = msg
        if len(msg.poses) < 1:
            return
        x = float(msg.poses[0].pose.position.x)
        y = float(msg.poses[0].pose.position.y)
        best_cid: Optional[int] = None
        best_d = 1e18
        for cid, segs in self._clusters.items():
            for seg in segs:
                sx, sy = seg['start']
                d = math.hypot(float(sx) - x, float(sy) - y)
                if d < best_d:
                    best_d = d
                    best_cid = int(cid)
        if best_cid is not None:
            self._current_cluster_id = int(best_cid)

    def _cb_decision(self, msg: StrategyDecisionStamped):
        now = time.monotonic()
        if self._current_cluster_id is not None and self._last_decision_mode in ('C_DEFER', 'D_REPLAN'):
            dt = max(0.0, now - float(self._last_decision_t_mono))
            self._state[int(self._current_cluster_id)]['blocked_time_sec'] += float(dt)
        self._last_decision_mode = str(msg.decision.mode)
        self._last_decision_t_mono = now

    def _cb_tracks(self, msg: ObstacleTrackArray):
        now = time.monotonic()
        target_frame = str(self.get_parameter('path_frame').value)
        tf = self._lookup_tf2d(target_frame, msg.header.frame_id, msg.header.stamp)
        for tr in msg.tracks:
            tid = str(tr.id)
            x = float(tr.pose.position.x)
            y = float(tr.pose.position.y)
            if tf is not None:
                x, y = _tf2d_apply(tf[0], tf[1], tf[2], x, y)
            vx = float(tr.velocity.linear.x)
            vy = float(tr.velocity.linear.y)
            speed = math.hypot(vx, vy)
            rec = self._tracks.get(tid, {})
            static_since = rec.get('static_since_mono')
            if speed <= float(self.get_parameter('persistent_speed_thresh').value):
                if static_since is None:
                    static_since = now
            else:
                static_since = None
            cid = self._cluster_for_point(x, y)
            self._tracks[tid] = {
                'x': float(x),
                'y': float(y),
                'radius': float(tr.radius),
                'speed': float(speed),
                'last_seen_mono': float(now),
                'static_since_mono': static_since,
                'cluster_id': cid,
            }
        self._cleanup_tracks(now)

    def _cleanup_tracks(self, now_mono: float):
        ttl = max(0.3, float(self.get_parameter('track_timeout_sec').value))
        stale = [tid for tid, rec in self._tracks.items() if (now_mono - float(rec.get('last_seen_mono', -1e9))) > ttl]
        for tid in stale:
            self._tracks.pop(tid, None)

    def _cluster_for_point(self, x: float, y: float) -> Optional[int]:
        for cid, bbox in self._cluster_bboxes.items():
            if float(bbox[0]) <= x <= float(bbox[2]) and float(bbox[1]) <= y <= float(bbox[3]):
                return int(cid)
        return None

    def _cb_event(self, msg: String):
        try:
            rec = json.loads(msg.data)
        except Exception:
            return
        cid = rec.get('cluster_id', self._current_cluster_id)
        if cid is None:
            return
        cid = int(cid)
        st = self._state.setdefault(cid, {
            'cluster_id': cid,
            'remaining_segments': 0,
            'cleanup_backlog': 0,
            'n_abort': 0,
            'n_c_fail': 0,
            'n_c_refresh': 0,
            'n_d1_fail': 0,
            'n_d_fail': 0,
            'n_d_no_candidates': 0,
            'n_d_attempts_exceeded': 0,
            'n_d2_fail': 0,
            'n_cluster_jump': 0,
            'n_path_rejected': 0,
            'blocked_time_sec': 0.0,
            'last_update_mono': time.monotonic(),
            'recent_d_requests': [],
            'all_entries_no_route': False,
            'last_event_seg': None,
        })
        typ = str(rec.get('type', ''))
        st['last_update_mono'] = time.monotonic()
        if 'seg_id' in rec:
            st['last_event_seg'] = rec.get('seg_id')
        if typ == 'segment_done':
            st['remaining_segments'] = max(0, int(st['remaining_segments']) - 1)
        elif typ in ('skip_segment', 'tiny_residual_drop', 'tiny_residual_resume_drop'):
            st['remaining_segments'] = max(0, int(st['remaining_segments']) - 1)
        elif typ == 'd2_replace':
            st['remaining_segments'] = int(rec.get('new_segment_count', st['remaining_segments']))
        elif typ == 'defer_segment':
            st['cleanup_backlog'] = int(st['cleanup_backlog']) + 1
        elif typ == 'skipped_slice':
            st['cleanup_backlog'] = int(st['cleanup_backlog']) + 1
        elif typ == 'follow_aborted':
            st['n_abort'] = int(st['n_abort']) + 1
        elif typ in ('c_fail', 'c_no_candidates', 'c2_no_candidates'):
            st['n_c_fail'] = int(st['n_c_fail']) + 1
        elif typ == 'c_refresh':
            st['n_c_refresh'] = int(st['n_c_refresh']) + 1
        elif typ == 'path_rejected':
            st['n_path_rejected'] = int(st['n_path_rejected']) + 1
        elif typ in ('d_request', 'd1_request', 'd2_request'):
            lst = list(st.get('recent_d_requests', []))
            lst.append(float(time.monotonic()))
            st['recent_d_requests'] = lst[-8:]
        elif typ == 'd1_fail':
            st['n_d1_fail'] = int(st['n_d1_fail']) + 1
        elif typ == 'd2_fail':
            st['n_d2_fail'] = int(st.get('n_d2_fail', 0)) + 1
        elif typ == 'd_fail':
            st['n_d_fail'] = int(st.get('n_d_fail', 0)) + 1
            reason = str(rec.get('reason', ''))
            if reason == 'D_no_candidates':
                st['n_d_no_candidates'] = int(st.get('n_d_no_candidates', 0)) + 1
            elif reason == 'D_attempts_exceeded':
                st['n_d_attempts_exceeded'] = int(st.get('n_d_attempts_exceeded', 0)) + 1
        elif typ in ('cluster_jump', 'd1_applied'):
            st['n_cluster_jump'] = int(st['n_cluster_jump']) + 1

    def _persistent_count(self, cid: int) -> int:
        now = time.monotonic()
        hold = float(self.get_parameter('persistent_hold_sec').value)
        cnt = 0
        for rec in self._tracks.values():
            if rec.get('cluster_id') != cid:
                continue
            ss = rec.get('static_since_mono')
            if ss is not None and (now - float(ss)) >= hold:
                cnt += 1
        return cnt

    def _current_path_is_fresh(self) -> bool:
        if self._current_path is None:
            return False
        stale = float(self.get_parameter('current_path_stale_sec').value)
        if stale <= 0.0:
            return True
        age = _stamp_to_sec(self.get_clock().now().to_msg()) - _stamp_to_sec(self._current_path.header.stamp)
        return age <= stale

    def _persistent_path_metrics(self, cid: int) -> Tuple[int, float, float]:
        path = self._current_path if (self._current_cluster_id == cid and self._current_path_is_fresh()) else None
        if path is None or len(path.poses) < 2:
            return 0, 0.0, 0.0
        now = time.monotonic()
        hold = float(self.get_parameter('persistent_hold_sec').value)
        inflate = float(self.get_parameter('persistent_path_overlap_inflate_m').value)
        min_ol = float(self.get_parameter('persistent_path_overlap_min_m').value)
        cnt = 0
        total_ol = 0.0
        max_ol = 0.0
        for rec in self._tracks.values():
            if rec.get('cluster_id') != cid:
                continue
            ss = rec.get('static_since_mono')
            if ss is None or (now - float(ss)) < hold:
                continue
            radius = max(0.10, float(rec.get('radius', 0.0))) + inflate
            ol = _path_overlap_len(path, float(rec.get('x', 0.0)), float(rec.get('y', 0.0)), radius)
            if ol >= min_ol:
                cnt += 1
                total_ol += float(ol)
                max_ol = max(max_ol, float(ol))
        return cnt, total_ol, max_ol

    def _robot_pose_map(self) -> Optional[Tuple[float, float, float]]:
        target_frame = str(self.get_parameter('path_frame').value)
        return self._lookup_tf2d(target_frame, 'base_link', self.get_clock().now().to_msg())

    def _compute_all_entries_no_route(self, cid: int) -> bool:
        grid = self._grid.grid
        robot = self._robot_pose_map()
        if grid is None or robot is None:
            return False
        occ_thresh = int(self.get_parameter('occ_thresh').value)
        any_candidate = False
        for seg in self._clusters.get(cid, []):
            for px, py in (seg['start'], seg['end']):
                if not grid.point_is_free(float(px), float(py), occ_thresh=occ_thresh, allow_unknown=False):
                    continue
                any_candidate = True
                cost, reachable = astar_cost(grid, (float(robot[0]), float(robot[1])), (float(px), float(py)), occ_thresh=occ_thresh)
                eu = math.hypot(float(px) - float(robot[0]), float(py) - float(robot[1]))
                if reachable or cost <= max(1.5, 3.0 * eu):
                    return False
        return True if any_candidate else True

    def _publish_summary(self):
        now = time.monotonic()
        self._cleanup_tracks(now)
        occ_thresh = int(self.get_parameter('occ_thresh').value)
        d1_thr = float(self.get_parameter('d1_score_threshold').value)
        d2_thr = float(self.get_parameter('d2_score_threshold').value)
        clusters_out: Dict[str, Any] = {}
        for cid, st in self._state.items():
            bbox = self._cluster_bboxes.get(cid)
            stats = {'occupied_ratio': 0.0, 'unknown_ratio': 0.0, 'free_ratio': 0.0, 'fragment_count': 0}
            if bbox is not None and self._grid.grid is not None:
                stats = occupancy_stats_in_bbox(self._grid.grid, bbox, occ_thresh=occ_thresh, free_unknown=False)
            n_persist = self._persistent_count(cid)
            n_path_persist, path_block_overlap_m, max_path_block_overlap_m = self._persistent_path_metrics(cid)
            recent_d = [t for t in st.get('recent_d_requests', []) if (now - float(t)) <= 30.0]
            st['recent_d_requests'] = recent_d
            all_entries_blocked = self._compute_all_entries_no_route(cid)
            st['all_entries_no_route'] = bool(all_entries_blocked)
            backlog = float(st.get('cleanup_backlog', 0))
            score_d1 = (
                2.0 * float(st.get('n_abort', 0))
                + 1.5 * float(st.get('n_c_fail', 0))
                + 1.0 * float(st.get('n_c_refresh', 0))
                + 2.0 * (1.0 if int(st.get('n_path_rejected', 0)) > 0 else 0.0)
                + 0.1 * float(st.get('blocked_time_sec', 0.0))
                + 1.2 * float(n_path_persist)
                + 0.8 * min(3.0, float(path_block_overlap_m))
                + 0.3 * max(0.0, float(n_persist - n_path_persist))
                + 1.0 * backlog
            )
            score_d2 = (
                2.0 * float(st.get('n_d1_fail', 0))
                + 1.2 * float(st.get('n_d_fail', 0))
                + 2.0 * float(st.get('n_d_no_candidates', 0))
                + 2.5 * float(st.get('n_d_attempts_exceeded', 0))
                + 2.5 * float(st.get('n_d2_fail', 0))
                + 1.5 * float(st.get('n_cluster_jump', 0))
                + 2.0 * (1.0 if all_entries_blocked else 0.0)
                + 1.5 * float(stats.get('unknown_ratio', 0.0))
                + 1.0 * float(stats.get('fragment_count', 0))
                + 0.8 * min(3.0, float(path_block_overlap_m))
                + 1.0 * backlog
            )
            hard_d1 = bool(
                len(recent_d) >= 2
                or int(st.get('n_c_refresh', 0)) >= 2
                or int(st.get('n_c_fail', 0)) >= 2
                or int(st.get('n_path_rejected', 0)) >= 1
                or int(st.get('n_abort', 0)) >= 2
                or (int(n_path_persist) >= 1 and float(path_block_overlap_m) >= 0.80)
            )
            hard_d2 = bool(
                int(st.get('n_d1_fail', 0)) >= 2
                or int(st.get('n_d_no_candidates', 0)) >= 2
                or int(st.get('n_d_attempts_exceeded', 0)) >= 1
                or int(st.get('n_d2_fail', 0)) >= 1
                or all_entries_blocked
                or (int(n_path_persist) >= 2 and float(path_block_overlap_m) >= 1.80)
                or backlog > max(1.0, float(st.get('remaining_segments', 0)))
            )
            clusters_out[str(cid)] = {
                'cluster_id': int(cid),
                'bbox': list(bbox) if bbox is not None else None,
                'remaining_segments': int(st.get('remaining_segments', 0)),
                'cleanup_backlog': int(st.get('cleanup_backlog', 0)),
                'blocked_time_sec': float(st.get('blocked_time_sec', 0.0)),
                'n_abort': int(st.get('n_abort', 0)),
                'n_c_fail': int(st.get('n_c_fail', 0)),
                'n_c_refresh': int(st.get('n_c_refresh', 0)),
                'n_d1_fail': int(st.get('n_d1_fail', 0)),
                'n_d_fail': int(st.get('n_d_fail', 0)),
                'n_d_no_candidates': int(st.get('n_d_no_candidates', 0)),
                'n_d_attempts_exceeded': int(st.get('n_d_attempts_exceeded', 0)),
                'n_d2_fail': int(st.get('n_d2_fail', 0)),
                'n_cluster_jump': int(st.get('n_cluster_jump', 0)),
                'n_path_rejected': int(st.get('n_path_rejected', 0)),
                'n_persistent': int(n_persist),
                'n_path_persistent': int(n_path_persist),
                'path_block_overlap_m': float(path_block_overlap_m),
                'max_path_block_overlap_m': float(max_path_block_overlap_m),
                'occupied_ratio': float(stats.get('occupied_ratio', 0.0)),
                'unknown_ratio': float(stats.get('unknown_ratio', 0.0)),
                'fragment_count': int(stats.get('fragment_count', 0)),
                'all_entries_no_route': bool(all_entries_blocked),
                'hard_d1': bool(hard_d1),
                'hard_d2': bool(hard_d2),
                'score_d1': float(score_d1),
                'score_d2': float(score_d2),
                'allow_d_replan': bool(hard_d1 or hard_d2 or score_d1 >= d1_thr or score_d2 >= d2_thr),
                'last_event_seg': st.get('last_event_seg'),
            }
        out = String()
        out.data = json.dumps({'stamp_mono': now, 'clusters': clusters_out}, ensure_ascii=False)
        self._pub.publish(out)


def main():
    rclpy.init()
    node = RegionStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
