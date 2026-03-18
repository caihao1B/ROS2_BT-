"""coverage_prediction.cv_predictor_node

Upgraded predictor (keeps the original topic/message interface).

Inputs:
  - /coverage/obstacles/tracks   (coverage_msgs/ObstacleTrackArray)

Outputs:
  - /coverage/obstacles/predictions   (coverage_msgs/ObstaclePredictionArray)
  - /coverage/obstacles/pred_markers  (visualization_msgs/MarkerArray)

Baseline behavior in the thesis is a deterministic constant-velocity (CV) predictor
(Eq. 5.9). This file upgrades it with an *optional* lightweight Kalman filter per
obstacle ID and then propagates a linear Gaussian CV model (Eqs. 5.10-5.12) to
visualize uncertainty growth using confidence ellipses.

Important engineering constraint for your project: we **do not change** the
ObstaclePrediction message. The decision/BT pipeline keeps consuming mean
trajectory points exactly like before. Covariance is only visualized via RViz
markers. This keeps the A–E mode selection + BT validation workflow stable.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from coverage_msgs.msg import ObstaclePrediction, ObstaclePredictionArray, ObstacleTrackArray


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _A_cv(dt: float) -> np.ndarray:
    """Constant-velocity transition matrix for [px, py, vx, vy]."""
    return np.array(
        [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _Q_white_noise_accel(dt: float, sigma_a: float) -> np.ndarray:
    """Discrete process noise for CV model with white acceleration noise."""
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt2 * dt2
    q = sigma_a * sigma_a
    return q * np.array(
        [
            [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
            [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
            [dt3 / 2.0, 0.0, dt2, 0.0],
            [0.0, dt3 / 2.0, 0.0, dt2],
        ],
        dtype=float,
    )


@dataclass
class _KF:
    x: np.ndarray  # (4,)
    P: np.ndarray  # (4,4)
    last_t: float
    radius: float = 0.20
    confidence: float = 0.3


class CVPredictor(Node):
    def __init__(self):
        super().__init__("cv_predictor")

        # ---------------- I/O ----------------
        self.declare_parameter("tracks_topic", "/coverage/obstacles/tracks")
        self.declare_parameter("pred_topic", "/coverage/obstacles/predictions")
        self.declare_parameter("pred_markers_topic", "/coverage/obstacles/pred_markers")

        # ---------------- prediction discretization ----------------
        self.declare_parameter("dt", 0.2)
        self.declare_parameter("horizon", 3.0)

        # ---------------- visualization ----------------
        self.declare_parameter("marker_lifetime", 0.6)
        self.declare_parameter("publish_ellipses", True)
        self.declare_parameter("ellipse_step", 6)          # draw every N prediction steps
        self.declare_parameter("ellipse_points", 24)       # polyline resolution
        self.declare_parameter("ellipse_chi2", 5.991)      # chi^2(2 dof) ~ 95%
        self.declare_parameter("ellipse_inflate_radius", True)
        self.declare_parameter("ellipse_min_eig", 1e-4)

        # ---------------- optional KF ----------------
        self.declare_parameter("enable_kf", True)
        self.declare_parameter("kf_min_dt", 0.05)
        self.declare_parameter("kf_max_age", 1.2)
        self.declare_parameter("kf_sigma_meas_p", 0.12)
        self.declare_parameter("kf_sigma_meas_v", 0.60)
        self.declare_parameter("kf_sigma_a", 0.80)
        self.declare_parameter("kf_init_sigma_p", 0.40)
        self.declare_parameter("kf_init_sigma_v", 0.80)
        self.declare_parameter("kf_meas_use_velocity", True)

        # Low-confidence velocity damping (robustness; see thesis discussion)
        self.declare_parameter("kf_vel_damping_conf0", 0.25)
        self.declare_parameter("kf_vel_damping_power", 1.0)

        # --- Optional static-map filter (prediction-level) ---
        # If enabled, the predictor will *NOT* publish predictions for tracks that lie on occupied map cells.
        # This prevents walls / static obstacles from being treated as "dynamic" by downstream BT logic.
        self.declare_parameter("use_map_filter", False)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_occ_thresh", 50)          # OccupancyGrid value (0..100)
        self.declare_parameter("map_pad_m", 0.15)             # inflation when checking occupancy
        self.declare_parameter("map_outside_is_occupied", True)
        self.declare_parameter("map_unknown_is_occupied", True)
        self.declare_parameter("map_filter_fail_open", True)  # if no map/TF, keep publishing preds
        self.declare_parameter("map_keep_if_speed_gt", 0.25)  # m/s: keep even if map-occupied (safety)
        self.declare_parameter("map_use_track_radius", True)  # include track.radius in occupied check
        self.declare_parameter("map_track_radius_min_m", 0.55)  # only use radius if >= this
        self.declare_parameter("map_track_radius_cap_m", 2.0)   # cap added radius to limit compute
        self.declare_parameter("track_timeout_s", 1.0)        # prune KF states if no update for this long

        # Map-filter diagnostics (optional)
        self.declare_parameter("map_filter_debug", False)
        self.declare_parameter("map_filter_debug_period", 2.0)

        self._kf: Dict[str, _KF] = {}

        tracks_topic = self.get_parameter("tracks_topic").value
        pred_topic = self.get_parameter("pred_topic").value
        pred_markers_topic = self.get_parameter("pred_markers_topic").value

        self._pub_pred = self.create_publisher(ObstaclePredictionArray, pred_topic, 10)
        self._pub_mark = self.create_publisher(MarkerArray, pred_markers_topic, 10)

        # Map filter config (optional)
        self._use_map_filter = bool(self.get_parameter("use_map_filter").value)
        self._map_topic = str(self.get_parameter("map_topic").value)
        self._map_occ_thresh = int(self.get_parameter("map_occ_thresh").value)
        self._map_pad_m = float(self.get_parameter("map_pad_m").value)
        self._map_outside_is_occupied = bool(self.get_parameter("map_outside_is_occupied").value)
        self._map_unknown_is_occupied = bool(self.get_parameter("map_unknown_is_occupied").value)
        self._map_filter_fail_open = bool(self.get_parameter("map_filter_fail_open").value)
        self._map_keep_if_speed_gt = float(self.get_parameter("map_keep_if_speed_gt").value)
        self._map_use_track_radius = bool(self.get_parameter("map_use_track_radius").value)
        self._map_track_radius_min_m = float(self.get_parameter("map_track_radius_min_m").value)
        self._map_track_radius_cap_m = float(self.get_parameter("map_track_radius_cap_m").value)
        self._track_timeout_s = float(self.get_parameter("track_timeout_s").value)
        self._map_filter_debug = bool(self.get_parameter("map_filter_debug").value)
        self._map_filter_debug_period = float(self.get_parameter("map_filter_debug_period").value)
        self._map_filter_last_log_t = -1.0e9

        # Map storage (OccupancyGrid)
        self._map = None
        self._map_w = 0
        self._map_h = 0
        self._map_res = 0.0
        self._map_ox = 0.0
        self._map_oy = 0.0
        self._map_frame = "map"
        self._map_data = None

        # TF buffer for map<->odom transforms (used by map filter)
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # Subscribe map with TRANSIENT_LOCAL (latched) QoS so late subscribers still receive it
        qos_map = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        if self._use_map_filter:
            self._sub_map = self.create_subscription(OccupancyGrid, self._map_topic, self._cb_map, qos_map)
        else:
            self._sub_map = None

        self._sub = self.create_subscription(ObstacleTrackArray, tracks_topic, self._cb_tracks, 10)

        self.get_logger().info(
            f"{tracks_topic} -> {pred_topic} (CV predictor + optional KF) + markers {pred_markers_topic}"
        )
        if self._use_map_filter:
            self.get_logger().info(
                f"[cv_predictor] map filter ON: topic={self._map_topic} occ_thresh={self._map_occ_thresh} "
                f"pad_m={self._map_pad_m:.3f} keep_if_speed_gt={self._map_keep_if_speed_gt:.2f} "
                f"use_track_radius={self._map_use_track_radius} radius_min={self._map_track_radius_min_m:.2f} "
                f"radius_cap={self._map_track_radius_cap_m:.2f} "
                f"fail_open={self._map_filter_fail_open} outside_occ={self._map_outside_is_occupied} "
                f"unknown_occ={self._map_unknown_is_occupied}"
            )
        else:
            self.get_logger().info("[cv_predictor] map filter OFF")

    # ---------------- KF core ----------------
    def _vel_damping(self, conf: float) -> float:
        c0 = float(self.get_parameter("kf_vel_damping_conf0").value)
        p = float(self.get_parameter("kf_vel_damping_power").value)
        if conf <= c0:
            return 0.0
        if conf >= 1.0:
            return 1.0
        s = (conf - c0) / max(1e-6, 1.0 - c0)
        s = max(0.0, min(1.0, s))
        return float(s ** max(1e-3, p))

    def _kf_init(self, t: float, px: float, py: float, vx: float, vy: float, radius: float, conf: float) -> _KF:
        sp = float(self.get_parameter("kf_init_sigma_p").value)
        sv = float(self.get_parameter("kf_init_sigma_v").value)
        x = np.array([px, py, vx, vy], dtype=float)
        P = np.diag([sp * sp, sp * sp, sv * sv, sv * sv]).astype(float)
        return _KF(x=x, P=P, last_t=t, radius=radius, confidence=conf)

    def _kf_predict(self, st: _KF, dt: float) -> None:
        A = _A_cv(dt)
        Q = _Q_white_noise_accel(dt, float(self.get_parameter("kf_sigma_a").value))
        st.x = A @ st.x
        st.P = A @ st.P @ A.T + Q

    def _kf_update_pos(self, st: _KF, px: float, py: float, sigma_p: float) -> None:
        z = np.array([px, py], dtype=float)
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=float)
        R = np.diag([sigma_p * sigma_p, sigma_p * sigma_p]).astype(float)

        y = z - (H @ st.x)
        S = H @ st.P @ H.T + R
        try:
            K = st.P @ H.T @ np.linalg.inv(S)
        except Exception:
            return
        st.x = st.x + K @ y
        st.P = (np.eye(4, dtype=float) - K @ H) @ st.P

    def _kf_update_full(
        self, st: _KF, px: float, py: float, vx: float, vy: float, sigma_p: float, sigma_v: float
    ) -> None:
        z = np.array([px, py, vx, vy], dtype=float)
        H = np.eye(4, dtype=float)
        R = np.diag([sigma_p * sigma_p, sigma_p * sigma_p, sigma_v * sigma_v, sigma_v * sigma_v]).astype(float)
        y = z - (H @ st.x)
        S = H @ st.P @ H.T + R
        try:
            K = st.P @ H.T @ np.linalg.inv(S)
        except Exception:
            return
        st.x = st.x + K @ y
        st.P = (np.eye(4, dtype=float) - K @ H) @ st.P

    # ---------------- ellipse helper ----------------
    def _ellipse_polyline(
        self, cx: float, cy: float, cov2: np.ndarray, chi2: float, n: int, inflate_r: float
    ) -> Tuple[Point, ...]:
        C = 0.5 * (cov2 + cov2.T)  # symmetrize
        w, V = np.linalg.eigh(C)   # eigenvalues ascending
        w = np.maximum(w, float(self.get_parameter("ellipse_min_eig").value))
        a = math.sqrt(max(0.0, chi2 * float(w[1]))) + inflate_r
        b = math.sqrt(max(0.0, chi2 * float(w[0]))) + inflate_r
        R = V

        pts = []
        for i in range(n + 1):
            th = 2.0 * math.pi * float(i) / float(n)
            local = np.array([a * math.cos(th), b * math.sin(th)], dtype=float)
            world = R @ local
            p = Point()
            p.x = float(cx + world[0])
            p.y = float(cy + world[1])
            p.z = 0.06
            pts.append(p)
        return tuple(pts)

    # ---------------- main callback ----------------

    def _cb_map(self, msg: OccupancyGrid):
        # Store map and metadata for fast queries
        self._map = msg
        self._map_w = int(msg.info.width)
        self._map_h = int(msg.info.height)
        self._map_res = float(msg.info.resolution)
        self._map_ox = float(msg.info.origin.position.x)
        self._map_oy = float(msg.info.origin.position.y)
        self._map_frame = str(msg.header.frame_id) if msg.header.frame_id else "map"
        self._map_data = msg.data
        self.get_logger().info(
            f"[cv_predictor] map received: w={self._map_w} h={self._map_h} "
            f"res={self._map_res:.3f} origin=({self._map_ox:.2f},{self._map_oy:.2f}) frame={self._map_frame}"
        )

    def _lookup_tf2d(self, to_frame: str, from_frame: str):
        if to_frame == from_frame:
            return 0.0, 0.0, 0.0
        tf = self._tf_buf.lookup_transform(to_frame, from_frame, rclpy.time.Time())
        q = tf.transform.rotation
        # yaw from quaternion
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        return tx, ty, yaw

    @staticmethod
    def _tf2d_apply(tx: float, ty: float, yaw: float, x: float, y: float):
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        return tx + cy * x - sy * y, ty + sy * x + cy * y

    def _world_to_map_rc(self, x: float, y: float):
        # map coords -> (row, col) in OccupancyGrid
        if self._map_res <= 0.0:
            return -1, -1
        col = int(math.floor((x - self._map_ox) / self._map_res))
        row = int(math.floor((y - self._map_oy) / self._map_res))
        if col < 0 or col >= self._map_w or row < 0 or row >= self._map_h:
            return -1, -1
        return row, col

    def _map_is_occupied_map_frame(self, x: float, y: float, occ_thresh: int, pad_m: float):
        # Conservative: treat outside / unknown as occupied if configured
        row, col = self._world_to_map_rc(x, y)
        if row < 0:
            return bool(self._map_outside_is_occupied)

        pad_px = int(math.ceil(max(0.0, pad_m) / max(1e-6, self._map_res)))
        # Quick single-cell check
        if pad_px <= 0:
            v = int(self._map_data[row * self._map_w + col])
            if v < 0:
                return bool(self._map_unknown_is_occupied)
            return v >= occ_thresh

        r0 = max(0, row - pad_px)
        r1 = min(self._map_h - 1, row + pad_px)
        c0 = max(0, col - pad_px)
        c1 = min(self._map_w - 1, col + pad_px)
        rr = pad_px * pad_px
        for r in range(r0, r1 + 1):
            dr = r - row
            for c in range(c0, c1 + 1):
                dc = c - col
                if dr * dr + dc * dc > rr:
                    continue
                v = int(self._map_data[r * self._map_w + c])
                if v < 0:
                    if self._map_unknown_is_occupied:
                        return True
                    continue
                if v >= occ_thresh:
                    return True
        return False

    def _cb_tracks(self, msg: ObstacleTrackArray):
        # refresh params dynamically
        dt_pred = float(self.get_parameter("dt").value)
        horizon = float(self.get_parameter("horizon").value)
        steps = max(1, int(horizon / max(1e-3, dt_pred)))
        life = float(self.get_parameter("marker_lifetime").value)
        lifetime = Duration(seconds=life).to_msg()

        enable_kf = bool(self.get_parameter("enable_kf").value)
        use_vel_meas = bool(self.get_parameter("kf_meas_use_velocity").value)
        kf_min_dt = float(self.get_parameter("kf_min_dt").value)
        kf_max_age = float(self.get_parameter("kf_max_age").value)
        sigma_p = float(self.get_parameter("kf_sigma_meas_p").value)
        sigma_v = float(self.get_parameter("kf_sigma_meas_v").value)

        publish_ell = bool(self.get_parameter("publish_ellipses").value)
        ell_step = max(1, int(self.get_parameter("ellipse_step").value))
        ell_n = max(12, int(self.get_parameter("ellipse_points").value))
        chi2 = float(self.get_parameter("ellipse_chi2").value)
        infl_r = bool(self.get_parameter("ellipse_inflate_radius").value)

        t_now = _stamp_to_sec(msg.header.stamp)

        # prune stale tracks
        for tid in list(self._kf.keys()):
            if (t_now - self._kf[tid].last_t) > kf_max_age:
                self._kf.pop(tid, None)

        # update / init KF states
        for tr in msg.tracks:
            tid = str(tr.id)
            px = float(tr.pose.position.x)
            py = float(tr.pose.position.y)
            vx_m = float(tr.velocity.linear.x)
            vy_m = float(tr.velocity.linear.y)
            radius = float(getattr(tr, "radius", 0.20))
            conf = float(getattr(tr, "confidence", 0.3))

            if tid not in self._kf:
                damp = self._vel_damping(conf)
                self._kf[tid] = self._kf_init(t_now, px, py, damp * vx_m, damp * vy_m, radius, conf)
                continue

            st = self._kf[tid]
            dt_upd = max(kf_min_dt, t_now - st.last_t)
            if enable_kf:
                self._kf_predict(st, dt_upd)
                if use_vel_meas:
                    self._kf_update_full(st, px, py, vx_m, vy_m, sigma_p, sigma_v)
                else:
                    self._kf_update_pos(st, px, py, sigma_p)
            else:
                # no filter: just mirror measurement
                st.x = np.array([px, py, vx_m, vy_m], dtype=float)
                st.P = np.eye(4, dtype=float)

            st.last_t = t_now
            st.radius = radius
            st.confidence = conf

        # build predictions and markers
        out = ObstaclePredictionArray()
        out.header = msg.header

        marr = MarkerArray()
        mid = 0
        preds = []

        A_d = _A_cv(dt_pred)
        Q_d = _Q_white_noise_accel(dt_pred, float(self.get_parameter("kf_sigma_a").value))

        drop_occ = 0
        drop_no_map = 0
        drop_tf = 0

        for tid, st in self._kf.items():
            # use filtered mean; optionally damp velocity for low confidence
            x0 = st.x.copy()
            damp = self._vel_damping(st.confidence) if enable_kf else 1.0
            x0[2] *= damp
            x0[3] *= damp

            # Optional static-map filter (prediction-level):
            # skip publishing predictions for tracks that sit on occupied map cells (walls / static obstacles).
            #
            # Design goal: In "no dynamic obstacles" runs, /tracks may still contain wall clusters (OK),
            # but /predictions should be empty or near-empty so the BT does NOT mis-trigger.
            if self._use_map_filter:
                # "Keep if moving" safeguard uses *undamped* velocity (avoid dropping true movers due to low-confidence damping).
                sp = float(math.hypot(float(st.x[2]), float(st.x[3])))
                if sp < self._map_keep_if_speed_gt:
                    # If map is not available yet, follow fail-open/fail-close.
                    if (self._map is None) or (self._map_data is None):
                        if not self._map_filter_fail_open:
                            drop_no_map += 1
                            continue
                    else:
                        try:
                            src_frame = msg.header.frame_id if msg.header.frame_id else "odom"
                            map_frame = self._map_frame
                            xw = float(x0[0])
                            yw = float(x0[1])
                            if src_frame != map_frame:
                                tx, ty, yaw = self._lookup_tf2d(map_frame, src_frame)
                                xm, ym = self._tf2d_apply(tx, ty, yaw, xw, yw)
                            else:
                                xm, ym = xw, yw
                            pad_eff = float(self._map_pad_m)
                            if self._map_use_track_radius and (float(st.radius) >= self._map_track_radius_min_m):
                                r_add = float(st.radius)
                                if self._map_track_radius_cap_m > 0.0:
                                    r_add = min(r_add, float(self._map_track_radius_cap_m))
                                pad_eff += r_add
                            if self._map_is_occupied_map_frame(xm, ym, self._map_occ_thresh, pad_eff):
                                drop_occ += 1
                                continue
                        except Exception:
                            if not self._map_filter_fail_open:
                                drop_tf += 1
                                continue

            pred = ObstaclePrediction()
            pred.id = tid
            pred.dt = float(dt_pred)
            pred.confidence = float(st.confidence)

            pts = []
            xk = x0.copy()
            Pk = st.P.copy()

            for k in range(1, steps + 1):
                xk = A_d @ xk
                if enable_kf:
                    Pk = A_d @ Pk @ A_d.T + Q_d

                pt = Point()
                pt.x = float(xk[0])
                pt.y = float(xk[1])
                pt.z = 0.0
                pts.append(pt)

                if enable_kf and publish_ell and (k % ell_step == 0 or k == 1 or k == steps):
                    cov2 = Pk[0:2, 0:2]
                    r_infl = float(st.radius) if infl_r else 0.0
                    em = Marker()
                    em.header = msg.header
                    em.ns = "pred_ellipse"
                    em.id = mid
                    mid += 1
                    em.type = Marker.LINE_STRIP
                    em.action = Marker.ADD
                    em.pose.orientation.w = 1.0
                    em.scale.x = 0.02
                    em.color.r, em.color.g, em.color.b, em.color.a = 1.0, 1.0, 0.2, 0.55
                    em.lifetime = lifetime
                    em.points = list(self._ellipse_polyline(float(xk[0]), float(xk[1]), cov2, chi2, ell_n, r_infl))
                    marr.markers.append(em)

            pred.points = pts
            preds.append(pred)

            # mean polyline marker
            m = Marker()
            m.header = msg.header
            m.ns = "pred"
            m.id = mid
            mid += 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.03
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.6, 1.0, 0.85
            m.lifetime = lifetime
            m.points = []

            a = Point()
            a.x = float(st.x[0])
            a.y = float(st.x[1])
            a.z = 0.05
            m.points.append(a)
            for q in pts:
                qq = Point()
                qq.x = q.x
                qq.y = q.y
                qq.z = 0.05
                m.points.append(qq)
            marr.markers.append(m)

        out.predictions = preds

        if self._use_map_filter and self._map_filter_debug:
            if (t_now - self._map_filter_last_log_t) >= max(0.1, self._map_filter_debug_period):
                self._map_filter_last_log_t = t_now
                map_ready = (self._map is not None) and (self._map_data is not None)
                self.get_logger().info(
                    f"[cv_predictor] map_filter: tracks={len(self._kf)} preds={len(preds)} "
                    f"drop_occ={drop_occ} drop_no_map={drop_no_map} drop_tf={drop_tf} "
                    f"map_ready={map_ready}"
                )

        self._pub_pred.publish(out)
        self._pub_mark.publish(marr)


def main() -> None:
    rclpy.init()
    node = CVPredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
