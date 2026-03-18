import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener  # type: ignore
try:
    from tf2_ros import TransformException  # type: ignore
except Exception:  # pragma: no cover
    TransformException = Exception  # type: ignore

from coverage_msgs.msg import ObstaclePredictionArray, ObstacleTrackArray, StrategyDecisionStamped


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _tf2d_apply(tx: float, ty: float, yaw: float, x: float, y: float) -> Tuple[float, float]:
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    xo = tx + cy * x - sy * y
    yo = ty + sy * x + cy * y
    return xo, yo


def _dist_point_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    c1 = vx * wx + vy * wy
    if c1 <= 0.0:
        return math.hypot(px - ax, py - ay)
    c2 = vx * vx + vy * vy
    if c2 <= 1e-9:
        return math.hypot(px - ax, py - ay)
    t = c1 / c2
    if t >= 1.0:
        return math.hypot(px - bx, py - by)
    projx = ax + t * vx
    projy = ay + t * vy
    return math.hypot(px - projx, py - projy)


def _dist_point_to_polyline(px: float, py: float, pts: List[Tuple[float, float]]) -> float:
    if len(pts) < 2:
        return float("inf")
    best = float("inf")
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        d = _dist_point_to_segment(px, py, ax, ay, bx, by)
        if d < best:
            best = d
    return best


def _stamp_to_sec(stamp) -> float:
    try:
        return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)
    except Exception:
        return 0.0


def _polyline_length(pts: List[Tuple[float, float]]) -> float:
    if len(pts) < 2:
        return 0.0
    s = 0.0
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        s += math.hypot(bx - ax, by - ay)
    return s


def _polyline_overlap_span(pts: List[Tuple[float, float]], ox: float, oy: float, radius: float) -> float:
    if len(pts) < 2 or radius <= 0.0:
        return 0.0
    covered = 0.0
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        seg_len = math.hypot(bx - ax, by - ay)
        if seg_len <= 1.0e-6:
            continue
        if _dist_point_to_segment(ox, oy, ax, ay, bx, by) <= radius:
            covered += seg_len
    return covered


class StrategySelector(Node):
    """
    Inputs:
      - /coverage/obstacles/predictions  (predictions in frame=odom by default)
      - /coverage/current_path           (the path currently being executed)
    Output:
      - /coverage/strategy_decision
    """

    def __init__(self):
        super().__init__('strategy_selector')

        self.declare_parameter('pred_topic', '/coverage/obstacles/predictions')
        self.declare_parameter('decision_topic', '/coverage/strategy_decision')
        self.declare_parameter('coverage_path_topic', '/coverage/current_path')

        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('path_frame', 'map')
        self.declare_parameter('use_latest_tf', True)

        # Optional: filter out predictions that overlap with the *static map*.
        # Rationale: scan-based tracking can cluster static walls into "obstacles".
        # When that happens the selector may trigger A/B/C/D/E even with *no* dynamic
        # obstacles spawned. Nav2 already handles static map obstacles via costmaps,
        # so the decision layer should usually only react to *non-map* obstacles.
        self.declare_parameter('use_map_filter', False)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_occ_thresh', 50)
        self.declare_parameter('map_pad_m', 0.15)
        self.declare_parameter('map_filter_min_overlap', 0.6)
        # Optional: only consider the first N predicted points when computing
        # overlap with the static map. This is often more robust than using the
        # full horizon (long-horizon drift can otherwise dominate the overlap).
        # Set <=0 to use the whole horizon.
        self.declare_parameter('map_filter_first_n_points', 0)

        # geometry thresholds
        self.declare_parameter('corridor_radius', 0.60)      # m (tube around path)
        self.declare_parameter('danger_radius', 0.55)        # m (hard stop)
        self.declare_parameter('lookahead_dist', 2.0)        # m (only consider forward portion)

        # timing thresholds
        self.declare_parameter('t_clear_max', 4.0)           # seconds -> Mode A
        self.declare_parameter('t_block_defer', 12.0)        # seconds -> Mode C

        # yield heuristics
        self.declare_parameter('enable_yield', True)
        self.declare_parameter('yield_angle_deg', 45.0)
        self.declare_parameter('yield_slowdown_ratio', 0.45)
        self.declare_parameter('velocity_estimation_points', 4)
        self.declare_parameter('yield_min_parallel_speed', 0.18)
        self.declare_parameter('yield_max_t_block', 2.5)
        self.declare_parameter('static_obstacle_speed_thresh', 0.12)
        self.declare_parameter('static_conflicts_prefer_c', True)
        self.declare_parameter('static_emergency_radius', 0.08)

        # Track-aware motion classification / persistence
        self.declare_parameter('tracks_topic', '/coverage/obstacles/tracks')
        self.declare_parameter('track_info_timeout_sec', 0.70)
        self.declare_parameter('moving_speed_thresh', 0.18)
        self.declare_parameter('moving_recent_sec', 1.00)
        self.declare_parameter('quasi_static_speed_thresh', 0.06)
        self.declare_parameter('quasi_static_hold_sec', 1.00)
        self.declare_parameter('c_conflict_persist_sec', 0.80)
        self.declare_parameter('c_confidence_min', 0.35)
        self.declare_parameter('dynamic_wait_t_clear_max', 2.50)
        self.declare_parameter('dynamic_conflicts_prefer_wait', True)
        self.declare_parameter('static_large_radius_thresh', 0.30)
        self.declare_parameter('semantic_speed_ema_alpha', 0.35)
        self.declare_parameter('current_path_stale_sec', 3.0)
        self.declare_parameter('semantic_overlap_inflate_m', 0.18)
        self.declare_parameter('persistent_blocker_speed_thresh', 0.05)
        self.declare_parameter('persistent_blocker_hold_sec', 1.8)
        self.declare_parameter('persistent_blocker_overlap_m', 0.80)
        self.declare_parameter('persistent_blocker_force_d_overlap_m', 1.80)
        self.declare_parameter('persistent_blocker_min_confidence', 0.30)

        # replan/skip
        self.declare_parameter('enable_replan', True)

        # Whether the selector is allowed to emit E_SKIP.
        # E_SKIP is a last-resort action that is better handled at the executive level
        # (after repeated failures / cleanup budgeting). Keeping it disabled by default
        # prevents single-frame perception glitches (often near corners/walls) from
        # skipping critical segments and causing cascaded FollowPath ABORTs.
        self.declare_parameter('enable_skip', False)

        pred_topic = self.get_parameter('pred_topic').value
        decision_topic = self.get_parameter('decision_topic').value
        path_topic = self.get_parameter('coverage_path_topic').value

        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)

        self._path: Optional[Path] = None
        self.create_subscription(Path, path_topic, self._cb_path, 10)
        self.create_subscription(ObstaclePredictionArray, pred_topic, self._cb_pred, 10)
        tracks_topic = str(self.get_parameter('tracks_topic').value)
        self.create_subscription(ObstacleTrackArray, tracks_topic, self._cb_tracks, 10)
        self._pub = self.create_publisher(StrategyDecisionStamped, decision_topic, 10)

        # Recent track / motion state keyed by prediction/track id.
        self._tracks: Dict[str, Dict[str, float]] = {}
        self._last_moving_mono: Dict[str, float] = {}
        self._conflict_since_mono: Dict[str, float] = {}

        # Map (used only when use_map_filter==True)
        self._map: Optional[OccupancyGrid] = None
        self._map_received_once: bool = False
        self._warned_no_map: bool = False
        map_topic = str(self.get_parameter('map_topic').value)
        # IMPORTANT: /map from map_server is usually TRANSIENT_LOCAL (latched).
        # If we subscribe with VOLATILE durability (default), we may miss the map and the filter will never work.
        qos_map = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._cb_map, qos_map)

        self.get_logger().info(f"[selector] pred={pred_topic} path={path_topic} tracks={tracks_topic} -> decision={decision_topic}")

    def _cb_path(self, msg: Path):
        self._path = msg

    def _cb_tracks(self, msg: ObstacleTrackArray):
        now = time.monotonic()
        alpha = max(0.05, min(1.0, float(self.get_parameter('semantic_speed_ema_alpha').value)))
        pb_speed = float(self.get_parameter('persistent_blocker_speed_thresh').value)
        qs_speed = float(self.get_parameter('quasi_static_speed_thresh').value)
        static_thresh = min(pb_speed, qs_speed)
        for tr in msg.tracks:
            try:
                tid = str(tr.id)
                vx = float(tr.velocity.linear.x)
                vy = float(tr.velocity.linear.y)
                speed = math.hypot(vx, vy)
                prev = self._tracks.get(tid, {})
                prev_ema = float(prev.get('speed_ema', speed))
                speed_ema = float(alpha) * speed + (1.0 - float(alpha)) * prev_ema
                static_since = prev.get('static_since_mono')
                if speed_ema <= static_thresh:
                    if static_since is None:
                        static_since = float(now)
                else:
                    static_since = None
                self._tracks[tid] = {
                    'vx': vx,
                    'vy': vy,
                    'speed': speed,
                    'speed_ema': speed_ema,
                    'static_since_mono': static_since,
                    'radius': float(tr.radius),
                    'confidence': float(tr.confidence),
                    'seen_mono': float(now),
                }
            except Exception:
                continue
        self._cleanup_history(now)

    def _cleanup_history(self, now_mono: float):
        ttl = max(0.1, float(self.get_parameter('track_info_timeout_sec').value))
        stale = [k for k, v in self._tracks.items() if (now_mono - float(v.get('seen_mono', -1e9))) > ttl]
        for k in stale:
            self._tracks.pop(k, None)
        mem = max(1.0, float(self.get_parameter('moving_recent_sec').value)
                  + float(self.get_parameter('quasi_static_hold_sec').value) + 1.0)
        for store in (self._last_moving_mono, self._conflict_since_mono):
            stale2 = [k for k, t in store.items() if (now_mono - float(t)) > mem]
            for k in stale2:
                store.pop(k, None)

    def _track_info(self, pred_id: str, now_mono: float) -> Optional[Dict[str, float]]:
        info = self._tracks.get(str(pred_id))
        if info is None:
            return None
        if (now_mono - float(info.get('seen_mono', -1e9))) > float(self.get_parameter('track_info_timeout_sec').value):
            return None
        return info

    def _conflict_age(self, pred_id: str, in_conflict: bool, now_mono: float) -> float:
        key = str(pred_id)
        if not in_conflict:
            self._conflict_since_mono.pop(key, None)
            return 0.0
        if key not in self._conflict_since_mono:
            self._conflict_since_mono[key] = float(now_mono)
            return 0.0
        return max(0.0, float(now_mono) - float(self._conflict_since_mono[key]))

    def _cb_map(self, msg: OccupancyGrid):
        self._map = msg
        if not self._map_received_once:
            self._map_received_once = True
            info = msg.info
            self.get_logger().info(
                f"[selector] map received: w={info.width} h={info.height} res={info.resolution:.3f} "
                f"origin=({info.origin.position.x:.2f},{info.origin.position.y:.2f}) frame={msg.header.frame_id}"
            )

    def _map_is_occupied(self, x: float, y: float, occ_thresh: int, pad_m: float) -> bool:
        """Return True if (x,y) in map frame lies on/near an occupied cell."""
        if self._map is None:
            return False
        info = self._map.info
        res = float(info.resolution)
        if res <= 0.0:
            return False
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        w = int(info.width)
        h = int(info.height)

        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if mx < 0 or my < 0 or mx >= w or my >= h:
            return False

        pad_cells = int(math.ceil(pad_m / res)) if pad_m > 0.0 else 0
        x0 = max(0, mx - pad_cells)
        x1 = min(w - 1, mx + pad_cells)
        y0 = max(0, my - pad_cells)
        y1 = min(h - 1, my + pad_cells)
        data = self._map.data

        for yy in range(y0, y1 + 1):
            base = yy * w
            for xx in range(x0, x1 + 1):
                v = int(data[base + xx])
                if v >= occ_thresh:
                    return True
        return False

    def _pred_overlaps_static_map(
        self,
        pts_world: List[Tuple[float, float, float]],
        occ_thresh: int,
        pad_m: float,
        min_overlap: float,
    ) -> bool:
        """Heuristic: if most predicted points fall on occupied map (with padding), treat as static map."""
        if self._map is None or not pts_world:
            return False
        n_first = int(self.get_parameter('map_filter_first_n_points').value)
        if n_first > 0:
            pts_world = pts_world[:n_first]
        # Sample at most ~20 points for speed.
        step = max(1, len(pts_world) // 20)
        tot = 0
        occ = 0
        for xw, yw, _tw in pts_world[::step]:
            tot += 1
            if self._map_is_occupied(float(xw), float(yw), occ_thresh, pad_m):
                occ += 1
        if tot <= 0:
            return False
        return (float(occ) / float(tot)) >= float(min_overlap)

    def _lookup_tf2d(self, target_frame: str, source_frame: str, stamp) -> Optional[Tuple[float, float, float]]:
        use_latest = bool(self.get_parameter('use_latest_tf').value)
        try:
            if use_latest:
                tf = self._tf.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            else:
                tf = self._tf.lookup_transform(target_frame, source_frame, rclpy.time.Time.from_msg(stamp))
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            q = tf.transform.rotation
            yaw = _yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
            return tx, ty, yaw
        except TransformException:
            return None
        except Exception:
            return None

    def _get_robot_pose_2d(self, frame_id: str, stamp) -> Optional[Tuple[float, float, float]]:
        base = self.get_parameter('robot_base_frame').value
        tf2d = self._lookup_tf2d(frame_id, base, stamp)
        if tf2d is None:
            return None
        tx, ty, yaw = tf2d
        return tx, ty, yaw

    def _forward_polyline(self, path: Path, rx: float, ry: float) -> List[Tuple[float, float]]:
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        if not pts:
            return []

        # nearest index
        best_i = 0
        best_d = float("inf")
        for i, (x, y) in enumerate(pts):
            d = math.hypot(x - rx, y - ry)
            if d < best_d:
                best_d = d
                best_i = i

        # forward subset by arc-length
        L = float(self.get_parameter('lookahead_dist').value)
        out = [pts[best_i]]
        acc = 0.0
        for (x0, y0), (x1, y1) in zip(pts[best_i:], pts[best_i+1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            acc += seg
            out.append((x1, y1))
            if acc >= L:
                break
        return out

    def _path_tangent(self, poly: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        if len(poly) < 2:
            return None
        x0, y0 = poly[0]
        x1, y1 = poly[1]
        vx = x1 - x0
        vy = y1 - y0
        n = math.hypot(vx, vy)
        if n < 1e-6:
            return None
        return vx / n, vy / n

    def _cb_pred(self, msg: ObstaclePredictionArray):
        dec = StrategyDecisionStamped()
        dec.header = msg.header
        dec.decision.mode = "NONE"
        dec.decision.stop_now = False
        dec.decision.slowdown_ratio = 1.0
        dec.decision.t_clear = 0.0
        dec.decision.t_block = 0.0

        if self._path is None or len(self._path.poses) < 2:
            self._pub.publish(dec)
            return

        path_frame = self.get_parameter('path_frame').value
        stale_sec = float(self.get_parameter('current_path_stale_sec').value)
        if stale_sec > 0.0:
            try:
                age = _stamp_to_sec(msg.header.stamp) - _stamp_to_sec(self._path.header.stamp)
                if age > stale_sec:
                    self._pub.publish(dec)
                    return
            except Exception:
                pass

        # robot pose in path_frame
        rpose = self._get_robot_pose_2d(path_frame, msg.header.stamp)
        if rpose is None:
            self._pub.publish(dec)
            return
        rx, ry, ryaw = rpose

        forward = self._forward_polyline(self._path, rx, ry)
        if len(forward) < 2:
            self._pub.publish(dec)
            return

        tan = self._path_tangent(forward)
        if tan is None:
            self._pub.publish(dec)
            return
        tx, ty = tan
        forward_len = max(1.0e-6, _polyline_length(forward))

        # TF from prediction frame -> path_frame
        src_frame = msg.header.frame_id
        tf2d = self._lookup_tf2d(path_frame, src_frame, msg.header.stamp)
        if tf2d is None:
            # cannot compare frames
            self._pub.publish(dec)
            return
        tfx, tfy, tfyaw = tf2d

        now_mono = time.monotonic()
        self._cleanup_history(now_mono)

        corridor_r = float(self.get_parameter('corridor_radius').value)
        danger_r = float(self.get_parameter('danger_radius').value)
        t_clear_max = float(self.get_parameter('t_clear_max').value)
        t_block_defer = float(self.get_parameter('t_block_defer').value)
        enable_yield = bool(self.get_parameter('enable_yield').value)
        yield_cos = math.cos(math.radians(float(self.get_parameter('yield_angle_deg').value)))
        yield_ratio = float(self.get_parameter('yield_slowdown_ratio').value)
        enable_replan = bool(self.get_parameter('enable_replan').value)
        enable_skip = bool(self.get_parameter('enable_skip').value)

        use_map_filter = bool(self.get_parameter('use_map_filter').value)
        map_occ_thresh = int(self.get_parameter('map_occ_thresh').value)
        map_pad_m = float(self.get_parameter('map_pad_m').value)
        map_min_overlap = float(self.get_parameter('map_filter_min_overlap').value)

        if use_map_filter and (self._map is None) and (not self._warned_no_map):
            self._warned_no_map = True
            self.get_logger().warn(
                "[selector] use_map_filter enabled but no /map received yet. "
                "Map filter will be inactive until a map arrives. "
                "Check /map QoS (should be TRANSIENT_LOCAL) and that map_server is running."
            )

        # Evaluate all predictions, choose the "worst" one
        worst_mode = "NONE"
        worst_stop = False
        worst_t_clear = 0.0
        worst_t_block = 0.0
        worst_slow = 1.0
        best_rejoin: Optional[PoseStamped] = None

        for pred in msg.predictions:
            if pred.dt <= 1e-6:
                continue

            # transform predicted points into path_frame
            pts_world: List[Tuple[float, float, float]] = []  # x,y,t
            for k, pt in enumerate(pred.points, start=1):
                xw, yw = _tf2d_apply(tfx, tfy, tfyaw, float(pt.x), float(pt.y))
                tw = float(k) * float(pred.dt)
                pts_world.append((xw, yw, tw))

            if not pts_world:
                continue

            # Optional: ignore predictions that are likely static map structure.
            if use_map_filter and self._pred_overlaps_static_map(pts_world, map_occ_thresh, map_pad_m, map_min_overlap):
                continue

            # estimate velocity direction from the first few points (more robust than using only 2 points)
            vx, vy = 0.0, 0.0
            vel_pts = max(2, int(self.get_parameter('velocity_estimation_points').value))
            n_vel = min(len(pts_world), vel_pts)
            if n_vel >= 2:
                (x1, y1, t1), (x2, y2, t2) = pts_world[0], pts_world[n_vel - 1]
                dt = max(1e-3, t2 - t1)
                vx = (x2 - x1) / dt
                vy = (y2 - y1) / dt
            pred_speed = math.hypot(vx, vy)

            first_conflict_t: Optional[float] = None
            last_conflict_t: Optional[float] = None
            stop_now = False
            min_robot_dist = float('inf')

            for xw, yw, tw in pts_world:
                d_robot = math.hypot(xw - rx, yw - ry)
                if d_robot < min_robot_dist:
                    min_robot_dist = d_robot
                # danger check
                if d_robot <= danger_r:
                    stop_now = True
                    first_conflict_t = 0.0 if first_conflict_t is None else min(first_conflict_t, tw)
                    last_conflict_t = tw if last_conflict_t is None else max(last_conflict_t, tw)
                    continue

                d_path = _dist_point_to_polyline(xw, yw, forward)
                if d_path <= corridor_r:
                    first_conflict_t = tw if first_conflict_t is None else min(first_conflict_t, tw)
                    last_conflict_t = tw if last_conflict_t is None else max(last_conflict_t, tw)

            conflict_age = self._conflict_age(str(pred.id), first_conflict_t is not None, now_mono)
            if first_conflict_t is None:
                continue

            horizon = float(len(pred.points)) * float(pred.dt)
            t_clear = (last_conflict_t + float(pred.dt)) if last_conflict_t is not None else 0.0
            t_block = horizon if (last_conflict_t is not None and abs(last_conflict_t - horizon) < 1e-6) else max(0.0, float(last_conflict_t or 0.0))

            # Track-aware motion classification helps separate moving blockers (prefer wait/yield)
            # from static/manual obstacles (prefer C after a short persistence hold).
            track = self._track_info(str(pred.id), now_mono)
            track_speed = 0.0
            track_speed_ema = 0.0
            track_radius = 0.0
            track_conf = float(pred.confidence)
            static_dwell = 0.0
            if track is not None:
                try:
                    track_speed = float(track.get('speed', math.hypot(float(track.get('vx', 0.0)), float(track.get('vy', 0.0)))))
                    track_speed_ema = float(track.get('speed_ema', track_speed))
                    track_radius = float(track.get('radius', 0.0))
                    track_conf = max(track_conf, float(track.get('confidence', 0.0)))
                    ss = track.get('static_since_mono')
                    if ss is not None:
                        static_dwell = max(0.0, now_mono - float(ss))
                except Exception:
                    pass

            motion_speed = max(pred_speed, track_speed, track_speed_ema)
            moving_speed_thresh = float(self.get_parameter('moving_speed_thresh').value)
            moving_recent_sec = float(self.get_parameter('moving_recent_sec').value)
            quasi_static_speed_thresh = float(self.get_parameter('quasi_static_speed_thresh').value)
            quasi_static_hold_sec = float(self.get_parameter('quasi_static_hold_sec').value)
            c_persist_sec = float(self.get_parameter('c_conflict_persist_sec').value)
            c_conf_min = float(self.get_parameter('c_confidence_min').value)
            dynamic_wait_t_clear_max = float(self.get_parameter('dynamic_wait_t_clear_max').value)
            static_large_radius_thresh = float(self.get_parameter('static_large_radius_thresh').value)
            semantic_overlap_inflate = float(self.get_parameter('semantic_overlap_inflate_m').value)
            persistent_blocker_speed_thresh = float(self.get_parameter('persistent_blocker_speed_thresh').value)
            persistent_blocker_hold_sec = float(self.get_parameter('persistent_blocker_hold_sec').value)
            persistent_blocker_overlap_m = float(self.get_parameter('persistent_blocker_overlap_m').value)
            persistent_blocker_force_d_overlap_m = float(self.get_parameter('persistent_blocker_force_d_overlap_m').value)
            persistent_blocker_min_conf = float(self.get_parameter('persistent_blocker_min_confidence').value)

            # update 'recently moving' memory
            if motion_speed >= moving_speed_thresh:
                self._last_moving_mono[str(pred.id)] = float(now_mono)
            last_move = float(self._last_moving_mono.get(str(pred.id), -1e9))
            recently_moving = (now_mono - last_move) <= moving_recent_sec

            # alignment with path tangent (used for yield)
            v_parallel = (vx * tx + vy * ty)
            ahead = ((pts_world[0][0] - rx) * tx + (pts_world[0][1] - ry) * ty) > 0.0
            cosang = (vx * tx + vy * ty) / max(1e-6, pred_speed) if pred_speed > 1e-6 else -1.0

            mode = "NONE"
            slow = 1.0
            prefer_static_c = bool(self.get_parameter('static_conflicts_prefer_c').value)
            static_emergency_r = float(self.get_parameter('static_emergency_radius').value)
            yield_min_parallel = float(self.get_parameter('yield_min_parallel_speed').value)
            yield_max_t_block = float(self.get_parameter('yield_max_t_block').value)
            dynamic_conflicts_prefer_wait = bool(self.get_parameter('dynamic_conflicts_prefer_wait').value)

            dynamic_like = recently_moving or motion_speed >= moving_speed_thresh
            static_like = (
                ((motion_speed <= quasi_static_speed_thresh) and (conflict_age >= c_persist_sec)
                 and ((now_mono - last_move) >= quasi_static_hold_sec))
                or ((track_radius >= static_large_radius_thresh) and (motion_speed <= moving_speed_thresh))
            )
            blocker_radius = max(track_radius, corridor_r * 0.35) + semantic_overlap_inflate
            ox, oy = float(pts_world[0][0]), float(pts_world[0][1])

            # Ignore quasi-static tracks whose centroid already lies on/inside known static map structure.
            # This suppresses ghost tracks from corners / walls without affecting runtime blockers that appear
            # in free space.
            if use_map_filter and (self._map is not None):
                static_track_pad = max(map_pad_m, blocker_radius)
                if self._map_is_occupied(ox, oy, map_occ_thresh, static_track_pad) and motion_speed <= quasi_static_speed_thresh:
                    continue

            overlap_len = _polyline_overlap_span(forward, ox, oy, blocker_radius)
            overlap_ratio = overlap_len / max(1.0e-6, forward_len)
            persistent_blocker = (
                (not dynamic_like)
                and (min(track_speed_ema if track_speed_ema > 0.0 else motion_speed, motion_speed) <= persistent_blocker_speed_thresh)
                and (static_dwell >= persistent_blocker_hold_sec or conflict_age >= max(c_persist_sec, 0.8 * persistent_blocker_hold_sec))
                and (overlap_len >= persistent_blocker_overlap_m or overlap_ratio >= 0.18)
                and (track_conf >= persistent_blocker_min_conf)
            )
            structural_blocker = bool(
                persistent_blocker
                and (overlap_len >= persistent_blocker_force_d_overlap_m or overlap_ratio >= 0.32)
            )

            if dynamic_like:
                # Do not route around moving trajectories with C. Prefer wait/yield.
                if stop_now:
                    mode = "A_WAIT"
                elif enable_yield and pred_speed > 1e-6 and v_parallel > yield_min_parallel and cosang >= yield_cos and ahead and t_block <= yield_max_t_block:
                    mode = "B_YIELD"
                    slow = yield_ratio
                elif dynamic_conflicts_prefer_wait or t_clear <= dynamic_wait_t_clear_max:
                    mode = "A_WAIT"
                else:
                    mode = "NONE"
            elif persistent_blocker:
                if stop_now and min_robot_dist <= static_emergency_r:
                    mode = "A_WAIT"
                elif structural_blocker and enable_replan:
                    mode = "D_REPLAN"
                else:
                    mode = "C_DEFER"
            elif prefer_static_c and static_like:
                if stop_now and min_robot_dist <= static_emergency_r:
                    mode = "A_WAIT"
                elif track_conf >= c_conf_min:
                    mode = "C_DEFER"
                elif t_clear <= t_clear_max:
                    mode = "A_WAIT"
                else:
                    mode = "NONE"
            else:
                if t_clear <= t_clear_max:
                    mode = "A_WAIT"
                elif (t_block >= t_block_defer) and (conflict_age >= c_persist_sec) and (track_conf >= c_conf_min):
                    mode = "C_DEFER"
                elif enable_replan:
                    mode = "D_REPLAN"
                else:
                    mode = "E_SKIP" if enable_skip else "NONE"

            # choose a rejoin pose (for D) = farthest point in forward polyline
            if mode == "D_REPLAN":
                rp = PoseStamped()
                rp.header = self._path.header
                rp.pose = self._path.poses[-1].pose  # fallback
                # choose last point in forward polyline as target
                rxj, ryj = forward[-1]
                rp.pose.position.x = float(rxj)
                rp.pose.position.y = float(ryj)
                rp.pose.position.z = 0.0
                rp.pose.orientation = self._path.poses[-1].pose.orientation
                best_rejoin = rp

            # Priority: hard wait > yield > local defer > replan > skip > none.
            # B outranks C so a moving obstacle can interrupt a static-C suggestion.
            prio = {"NONE": 0, "E_SKIP": 1, "D_REPLAN": 2, "C_DEFER": 3, "B_YIELD": 4, "A_WAIT": 5}
            cand_key = ((1 if stop_now else 0), prio.get(mode, 0))
            worst_key = ((1 if worst_stop else 0), prio.get(worst_mode, 0))
            if cand_key > worst_key:
                worst_mode = mode
                worst_stop = stop_now
                worst_t_clear = float(t_clear)
                worst_t_block = float(t_block)
                worst_slow = float(slow)

        dec.decision.mode = worst_mode
        dec.decision.stop_now = bool(worst_stop)
        dec.decision.slowdown_ratio = float(worst_slow)
        dec.decision.t_clear = float(worst_t_clear)
        dec.decision.t_block = float(worst_t_block)

        # safe_stop_pose: robot current pose
        dec.decision.safe_stop_pose.header.frame_id = path_frame
        dec.decision.safe_stop_pose.header.stamp = msg.header.stamp
        dec.decision.safe_stop_pose.pose.position.x = float(rx)
        dec.decision.safe_stop_pose.pose.position.y = float(ry)
        dec.decision.safe_stop_pose.pose.position.z = 0.0
        dec.decision.safe_stop_pose.pose.orientation.w = 1.0

        if best_rejoin is not None:
            dec.decision.rejoin_pose = best_rejoin

        self._pub.publish(dec)


def main():
    rclpy.init()
    n = StrategySelector()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
