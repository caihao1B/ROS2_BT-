import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener  # type: ignore
try:
    from tf2_ros import TransformException  # type: ignore
except Exception:  # pragma: no cover
    TransformException = Exception  # type: ignore

from coverage_msgs.msg import ObstacleTrack, ObstacleTrackArray


@dataclass
class _Track:
    tid: int
    sid: str
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    radius: float = 0.20
    confidence: float = 0.3
    cls: int = 0
    last_t: float = 0.0
    age: int = 0
    misses: int = 0


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


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


class ScanTracker(Node):
    """
    TB3 LaserScan -> clustering -> simple nearest-neighbor tracking -> velocity estimate
    Publishes:
      - /coverage/obstacles/tracks  (ObstacleTrackArray)
      - /coverage/obstacles/markers (MarkerArray)
      - /coverage/obstacles/cluster_markers (MarkerArray)
    """

    def __init__(self):
        super().__init__("scan_tracker")

        # topics / frames
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_frame", "odom")  # track positions in this frame
        self.declare_parameter("tracks_topic", "/coverage/obstacles/tracks")
        self.declare_parameter("markers_topic", "/coverage/obstacles/markers")
        self.declare_parameter("cluster_markers_topic", "/coverage/obstacles/cluster_markers")

        # scan filtering
        self.declare_parameter("min_range", 0.05)
        self.declare_parameter("max_range", 6.0)

        # clustering (LaserScan is ordered by angle, so we cluster consecutive points)
        self.declare_parameter("cluster_dist_base", 0.10)   # meters
        self.declare_parameter("cluster_dist_scale", 0.03)  # meters per meter range (threshold = base + scale*r)
        self.declare_parameter("min_cluster_size", 3)
        self.declare_parameter("max_cluster_size", 60)

        # tracking
        self.declare_parameter("match_dist", 0.60)          # gating distance (m)
        self.declare_parameter("track_timeout", 1.0)        # seconds since last seen
        self.declare_parameter("vel_alpha", 0.70)           # EMA smoothing (higher = smoother)
        self.declare_parameter("pos_alpha", 0.20)           # EMA on position (0..1)
        self.declare_parameter("min_dt", 0.05)              # avoid crazy velocity from tiny dt

        # classification by estimated radius
        self.declare_parameter("cls_small_r", 0.25)
        self.declare_parameter("cls_medium_r", 0.55)

        # marker config
        self.declare_parameter("marker_lifetime", 0.4)
        self.declare_parameter("publish_clusters", True)

        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)

        scan_topic = self.get_parameter("scan_topic").value
        self._output_frame = self.get_parameter("output_frame").value
        tracks_topic = self.get_parameter("tracks_topic").value
        markers_topic = self.get_parameter("markers_topic").value
        cluster_markers_topic = self.get_parameter("cluster_markers_topic").value

        self._pub_tracks = self.create_publisher(ObstacleTrackArray, tracks_topic, 10)
        self._pub_markers = self.create_publisher(MarkerArray, markers_topic, 10)
        self._pub_cluster_markers = self.create_publisher(MarkerArray, cluster_markers_topic, 10)

        self._sub = self.create_subscription(LaserScan, scan_topic, self._cb_scan, qos_profile_sensor_data)

        self._tracks: List[_Track] = []
        self._next_id = 1
        self._last_scan_t: Optional[float] = None

        self.get_logger().info(
            f"[scan_tracker] scan={scan_topic} output_frame={self._output_frame} "
            f"tracks={tracks_topic} markers={markers_topic}"
        )

    # ---------------- scan -> points ----------------
    def _scan_to_points(self, msg: LaserScan) -> List[Tuple[float, float, float]]:
        """Return list of (x,y,r) in scan frame."""
        rmin = float(self.get_parameter("min_range").value)
        rmax = float(self.get_parameter("max_range").value)

        pts = []
        a = float(msg.angle_min)
        da = float(msg.angle_increment)

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                a += da
                continue
            rr = float(r)
            if rr < rmin or rr > rmax:
                a += da
                continue
            x = rr * math.cos(a)
            y = rr * math.sin(a)
            pts.append((x, y, rr))
            a += da
        return pts

    # ---------------- clustering ----------------
    def _cluster_points(self, pts: List[Tuple[float, float, float]]) -> List[List[Tuple[float, float, float]]]:
        base = float(self.get_parameter("cluster_dist_base").value)
        scale = float(self.get_parameter("cluster_dist_scale").value)
        min_n = int(self.get_parameter("min_cluster_size").value)
        max_n = int(self.get_parameter("max_cluster_size").value)

        clusters: List[List[Tuple[float, float, float]]] = []
        if not pts:
            return clusters

        cur: List[Tuple[float, float, float]] = [pts[0]]
        for p in pts[1:]:
            x0, y0, r0 = cur[-1]
            x1, y1, r1 = p
            d = math.hypot(x1 - x0, y1 - y0)
            thr = base + scale * min(r0, r1)
            if d <= thr and len(cur) < max_n:
                cur.append(p)
            else:
                if len(cur) >= min_n:
                    clusters.append(cur)
                cur = [p]
        if len(cur) >= min_n:
            clusters.append(cur)
        return clusters

    def _cluster_stats(self, cl: List[Tuple[float, float, float]]) -> Tuple[float, float, float, int]:
        xs = [p[0] for p in cl]
        ys = [p[1] for p in cl]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        r = max(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys))
        return cx, cy, max(0.10, r), len(cl)

    # ---------------- TF helper ----------------
    def _lookup_tf2d(self, target_frame: str, source_frame: str, use_latest: bool, stamp) -> Optional[Tuple[float, float, float]]:
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

    # ---------------- tracking ----------------
    def _assign(self, dets: List[Tuple[float, float, float, int]]) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """
        dets: list of (x,y,radius,cluster_size) in output_frame
        returns: matches (track_idx, det_idx), unmatched_tracks, unmatched_dets
        """
        gate = float(self.get_parameter("match_dist").value)
        matches: List[Tuple[int, int]] = []
        used_t = set()
        used_d = set()

        # greedy by distance
        cand = []
        for ti, tr in enumerate(self._tracks):
            for di, (x, y, _, _) in enumerate(dets):
                d = math.hypot(x - tr.x, y - tr.y)
                if d <= gate:
                    cand.append((d, ti, di))
        cand.sort(key=lambda x: x[0])

        for _, ti, di in cand:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            matches.append((ti, di))

        unmatched_t = [i for i in range(len(self._tracks)) if i not in used_t]
        unmatched_d = [i for i in range(len(dets)) if i not in used_d]
        return matches, unmatched_t, unmatched_d

    def _classify(self, radius: float) -> int:
        rs = float(self.get_parameter("cls_small_r").value)
        rm = float(self.get_parameter("cls_medium_r").value)
        if radius <= rs:
            return 1
        if radius <= rm:
            return 2
        return 3

    # ---------------- markers ----------------
    def _make_markers(self, stamp, frame_id: str) -> MarkerArray:
        life = float(self.get_parameter("marker_lifetime").value)
        lifetime = Duration(seconds=life).to_msg()

        arr = MarkerArray()
        m_id = 0

        for tr in self._tracks:
            # sphere for obstacle body
            body = Marker()
            body.header.stamp = stamp
            body.header.frame_id = frame_id
            body.ns = "tracks_body"
            body.id = m_id; m_id += 1
            body.type = Marker.SPHERE
            body.action = Marker.ADD
            body.pose.position.x = float(tr.x)
            body.pose.position.y = float(tr.y)
            body.pose.position.z = 0.10
            body.pose.orientation.w = 1.0
            s = max(0.10, 2.0 * float(tr.radius))
            body.scale.x = s
            body.scale.y = s
            body.scale.z = 0.10
            # color by class (simple)
            if tr.cls == 1:
                body.color.r, body.color.g, body.color.b, body.color.a = 0.2, 0.9, 0.2, 0.9
            elif tr.cls == 2:
                body.color.r, body.color.g, body.color.b, body.color.a = 0.95, 0.8, 0.2, 0.9
            else:
                body.color.r, body.color.g, body.color.b, body.color.a = 0.95, 0.2, 0.2, 0.9
            body.lifetime = lifetime
            arr.markers.append(body)

            # arrow for velocity
            vel = Marker()
            vel.header.stamp = stamp
            vel.header.frame_id = frame_id
            vel.ns = "tracks_vel"
            vel.id = m_id; m_id += 1
            vel.type = Marker.ARROW
            vel.action = Marker.ADD
            vel.pose.orientation.w = 1.0
            vel.scale.x = 0.05
            vel.scale.y = 0.10
            vel.scale.z = 0.10
            vel.color.r, vel.color.g, vel.color.b, vel.color.a = 0.2, 0.4, 1.0, 0.9
            vel.lifetime = lifetime

            p0x, p0y = tr.x, tr.y
            vnorm = math.hypot(tr.vx, tr.vy)
            scale = 0.8
            p1x = p0x + scale * tr.vx
            p1y = p0y + scale * tr.vy
            vel.points = []
            from geometry_msgs.msg import Point
            a = Point(); a.x = float(p0x); a.y = float(p0y); a.z = 0.15
            b = Point(); b.x = float(p1x); b.y = float(p1y); b.z = 0.15
            vel.points.append(a); vel.points.append(b)
            arr.markers.append(vel)

            # text
            txt = Marker()
            txt.header.stamp = stamp
            txt.header.frame_id = frame_id
            txt.ns = "tracks_text"
            txt.id = m_id; m_id += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(tr.x)
            txt.pose.position.y = float(tr.y)
            txt.pose.position.z = 0.35
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.18
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 1.0, 1.0, 0.95
            spd = math.hypot(tr.vx, tr.vy)
            txt.text = f"{tr.sid} v={spd:.2f} conf={tr.confidence:.2f}"
            txt.lifetime = lifetime
            arr.markers.append(txt)

        return arr

    def _make_cluster_markers(self, stamp, frame_id: str, clusters_out: List[Tuple[float, float]]) -> MarkerArray:
        life = float(self.get_parameter("marker_lifetime").value)
        lifetime = Duration(seconds=life).to_msg()

        arr = MarkerArray()
        pts = Marker()
        pts.header.stamp = stamp
        pts.header.frame_id = frame_id
        pts.ns = "clusters"
        pts.id = 0
        pts.type = Marker.POINTS
        pts.action = Marker.ADD
        pts.pose.orientation.w = 1.0
        pts.scale.x = 0.06
        pts.scale.y = 0.06
        pts.color.r, pts.color.g, pts.color.b, pts.color.a = 0.9, 0.9, 0.9, 0.9
        pts.lifetime = lifetime

        from geometry_msgs.msg import Point
        pts.points = []
        for x, y in clusters_out:
            p = Point()
            p.x = float(x); p.y = float(y); p.z = 0.10
            pts.points.append(p)

        arr.markers.append(pts)
        return arr

    # ---------------- callback ----------------
    def _cb_scan(self, msg: LaserScan):
        t = _stamp_to_sec(msg.header.stamp)
        dt_msg = 0.1 if self._last_scan_t is None else max(float(self.get_parameter("min_dt").value), t - self._last_scan_t)
        self._last_scan_t = t

        pts = self._scan_to_points(msg)
        clusters = self._cluster_points(pts)

        # transform clusters centroid to output_frame
        use_latest_tf = True  # default for stability with sim time
        tf2d = self._lookup_tf2d(self._output_frame, msg.header.frame_id, use_latest_tf, msg.header.stamp)

        dets_out: List[Tuple[float, float, float, int]] = []
        cluster_centroids_out: List[Tuple[float, float]] = []

        for cl in clusters:
            cx, cy, radius, n = self._cluster_stats(cl)
            if tf2d is None:
                # publish in scan frame if TF missing (but keep output_frame in header)
                xo, yo = cx, cy
            else:
                tx, ty, yaw = tf2d
                xo, yo = _tf2d_apply(tx, ty, yaw, cx, cy)

            dets_out.append((xo, yo, radius, n))
            cluster_centroids_out.append((xo, yo))

        # assign
        matches, unmatched_t, unmatched_d = self._assign(dets_out)

        # update matched
        vel_alpha = float(self.get_parameter("vel_alpha").value)
        pos_alpha = float(self.get_parameter("pos_alpha").value)

        for ti, di in matches:
            tr = self._tracks[ti]
            x, y, radius, n = dets_out[di]

            # measured velocity
            vx_m = (x - tr.x) / dt_msg
            vy_m = (y - tr.y) / dt_msg

            tr.x = (1.0 - pos_alpha) * x + pos_alpha * tr.x
            tr.y = (1.0 - pos_alpha) * y + pos_alpha * tr.y
            tr.vx = vel_alpha * tr.vx + (1.0 - vel_alpha) * vx_m
            tr.vy = vel_alpha * tr.vy + (1.0 - vel_alpha) * vy_m
            tr.radius = radius
            tr.cls = self._classify(radius)
            tr.last_t = t
            tr.age += 1
            tr.misses = 0

            # confidence from cluster size + age
            conf_sz = min(1.0, float(n) / 10.0)
            conf_age = min(1.0, float(tr.age) / 5.0)
            tr.confidence = 0.2 + 0.8 * (0.5 * conf_sz + 0.5 * conf_age)

        # unmatched tracks -> timeout
        timeout = float(self.get_parameter("track_timeout").value)
        keep: List[_Track] = []
        for i, tr in enumerate(self._tracks):
            if i in unmatched_t:
                tr.misses += 1
            # remove if too old
            if (t - tr.last_t) <= timeout:
                keep.append(tr)
        self._tracks = keep

        # new tracks
        for di in unmatched_d:
            x, y, radius, n = dets_out[di]
            tid = self._next_id
            self._next_id += 1
            sid = f"trk_{tid:04d}"
            tr = _Track(tid=tid, sid=sid, x=x, y=y, radius=radius, cls=self._classify(radius), last_t=t, age=1)
            conf_sz = min(1.0, float(n) / 10.0)
            tr.confidence = 0.2 + 0.8 * conf_sz
            self._tracks.append(tr)

        # publish tracks
        out = ObstacleTrackArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._output_frame

        tracks_msg: List[ObstacleTrack] = []
        for tr in self._tracks:
            m = ObstacleTrack()
            m.id = tr.sid

            p = Pose()
            p.position.x = float(tr.x)
            p.position.y = float(tr.y)
            p.position.z = 0.0
            p.orientation.w = 1.0
            m.pose = p

            tw = Twist()
            tw.linear.x = float(tr.vx)
            tw.linear.y = float(tr.vy)
            tw.angular.z = 0.0
            m.velocity = tw

            m.radius = float(tr.radius)
            m.confidence = float(tr.confidence)
            m.cls = int(tr.cls)

            tracks_msg.append(m)

        out.tracks = tracks_msg
        self._pub_tracks.publish(out)

        # markers
        self._pub_markers.publish(self._make_markers(msg.header.stamp, self._output_frame))

        if bool(self.get_parameter("publish_clusters").value):
            self._pub_cluster_markers.publish(self._make_cluster_markers(msg.header.stamp, self._output_frame, cluster_centroids_out))


def main():
    rclpy.init()
    n = ScanTracker()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
