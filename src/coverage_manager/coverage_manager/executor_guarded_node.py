import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time as RclTime

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener  # type: ignore
try:
    from tf2_ros import TransformException  # type: ignore
except Exception:  # pragma: no cover
    TransformException = Exception  # type: ignore

from .executor_node import CoverageExecutor
from .event_utils import make_event


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Return yaw (Z) from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GuardedCoverageExecutor(CoverageExecutor):
    """CoverageExecutor with segment boundary guards.

    Adds:
      - Start gate: if robot is too far from segment start, align with NavigateToPose.
      - End gate: if FollowPath reports success but robot is far from segment end, align to end.
      - Stop-and-settle before/after actions.
    """

    def __init__(self) -> None:
        super().__init__()

        # Guard parameters
        self.declare_parameter("guard_enable_start_gate", True)
        self.declare_parameter("guard_start_gate_m", 1.0)

        self.declare_parameter("guard_enable_end_gate", True)
        self.declare_parameter("guard_end_gate_m", 0.5)

        self.declare_parameter("guard_use_nav_to_pose", True)
        self.declare_parameter("guard_nav_action", "/navigate_to_pose")
        self.declare_parameter("guard_nav_timeout_sec", 120.0)

        self.declare_parameter("guard_wait_stop_before_follow", True)
        self.declare_parameter("guard_wait_stop_after_follow", True)
        self.declare_parameter("guard_stop_timeout_sec", 10.0)
        self.declare_parameter("guard_stop_hold_sec", 0.5)
        self.declare_parameter("guard_stop_lin_eps", 0.03)
        self.declare_parameter("guard_stop_ang_eps", 0.20)

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("guard_tf_timeout_sec", 1.0)

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Odom for stop detection
        self._last_twist = None
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        # NavigateToPose client (lazy)
        self._nav_client: Optional[ActionClient] = None

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_twist = msg.twist.twist

    def _ensure_nav_client(self) -> ActionClient:
        if self._nav_client is None:
            name = self.get_parameter("guard_nav_action").get_parameter_value().string_value
            self._nav_client = ActionClient(self, NavigateToPose, name)
        return self._nav_client

    def _wait_until_stopped(self) -> bool:
        timeout = float(self.get_parameter("guard_stop_timeout_sec").value)
        hold = float(self.get_parameter("guard_stop_hold_sec").value)
        v_eps = float(self.get_parameter("guard_stop_lin_eps").value)
        w_eps = float(self.get_parameter("guard_stop_ang_eps").value)

        t0 = time.monotonic()
        ok_since: Optional[float] = None

        while rclpy.ok() and (time.monotonic() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self._last_twist is None:
                continue

            v = abs(float(self._last_twist.linear.x))
            w = abs(float(self._last_twist.angular.z))

            if v < v_eps and w < w_eps:
                if ok_since is None:
                    ok_since = time.monotonic()
                elif (time.monotonic() - ok_since) >= hold:
                    return True
            else:
                ok_since = None

        self.publish_event(make_event("guard_wait_stop_timeout", timeout=timeout))
        return False

    def _lookup_robot_pose(self, target_frame: str) -> Optional[Tuple[float, float, float]]:
        base_frame = self.get_parameter("robot_base_frame").get_parameter_value().string_value
        tf_timeout = float(self.get_parameter("guard_tf_timeout_sec").value)

        t0 = time.monotonic()
        last_err: Optional[str] = None
        while rclpy.ok() and (time.monotonic() - t0) < tf_timeout:
            try:
                tf = self._tf_buffer.lookup_transform(target_frame, base_frame, RclTime())
                x = float(tf.transform.translation.x)
                y = float(tf.transform.translation.y)
                q = tf.transform.rotation
                yaw = _yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
                return (x, y, yaw)
            except TransformException as e:
                last_err = str(e)
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception as e:
                last_err = str(e)
                rclpy.spin_once(self, timeout_sec=0.05)

        if last_err:
            self.publish_event(make_event(
                "guard_tf_failed", target_frame=target_frame, base_frame=base_frame, err=last_err
            ))
        return None

    def _distance_to_pose(self, frame_id: str, pose: PoseStamped) -> Optional[float]:
        rp = self._lookup_robot_pose(frame_id)
        if rp is None:
            return None
        rx, ry, _ = rp
        px = float(pose.pose.position.x)
        py = float(pose.pose.position.y)
        return math.hypot(px - rx, py - ry)

    def _nav_to_pose(self, pose: PoseStamped, reason: str) -> Tuple[bool, str]:
        client = self._ensure_nav_client()

        if not client.wait_for_server(timeout_sec=2.0):
            return (False, "NavigateToPose server not available")

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            return (False, "NavigateToPose goal rejected")

        timeout = float(self.get_parameter("guard_nav_timeout_sec").value)
        result_future = gh.get_result_async()

        t0 = time.monotonic()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (time.monotonic() - t0) > timeout:
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                return (False, f"NavigateToPose timeout after {timeout:.1f}s ({reason})")

        res = result_future.result()
        if res is None:
            return (False, "NavigateToPose result missing")

        if int(res.status) == GoalStatus.STATUS_SUCCEEDED:
            return (True, "")
        return (False, f"NavigateToPose failed status={int(res.status)}")

    def follow_path(self, path: Path) -> Tuple[bool, int, str]:
        frame = path.header.frame_id or "map"
        if len(path.poses) == 0:
            return (False, -900, "Guard: input path is empty")

        start_gate = bool(self.get_parameter("guard_enable_start_gate").value)
        use_nav = bool(self.get_parameter("guard_use_nav_to_pose").value)
        start_gate_m = float(self.get_parameter("guard_start_gate_m").value)

        start_pose = path.poses[0]
        end_pose = path.poses[-1]

        # settle before FollowPath
        if bool(self.get_parameter("guard_wait_stop_before_follow").value):
            self._wait_until_stopped()

        # ---- START GATE ----
        d_start = self._distance_to_pose(frame, start_pose)
        if start_gate and d_start is not None and d_start > start_gate_m:
            self.publish_event(make_event(
                "guard_start_gate_triggered",
                frame=frame,
                dist_start=float(d_start),
                gate_m=float(start_gate_m),
            ))

            if use_nav:
                # stamp=0 => latest TF
                start_pose.header.stamp.sec = 0
                start_pose.header.stamp.nanosec = 0
                ok_nav, nav_msg = self._nav_to_pose(start_pose, reason="align_to_segment_start")
                if not ok_nav:
                    self.publish_event(make_event("guard_align_start_failed", msg=str(nav_msg)))
                    return (False, -910, f"Guard start align failed: {nav_msg}")
                self.publish_event(make_event("guard_align_start_succeeded"))
                self._wait_until_stopped()
            else:
                return (False, -911, f"Guard start gate violated (d_start={d_start:.2f}m > {start_gate_m:.2f}m)")

        # ---- ORIGINAL FOLLOW PATH ----
        ok, err_code, err_msg = super().follow_path(path)

        # settle after FollowPath
        if bool(self.get_parameter("guard_wait_stop_after_follow").value):
            self._wait_until_stopped()

        if not ok:
            return (ok, err_code, err_msg)

        # ---- END GATE ----
        end_gate = bool(self.get_parameter("guard_enable_end_gate").value)
        end_gate_m = float(self.get_parameter("guard_end_gate_m").value)

        d_end = self._distance_to_pose(frame, end_pose)
        if end_gate and d_end is not None and d_end > end_gate_m:
            self.publish_event(make_event(
                "guard_end_gate_triggered",
                frame=frame,
                dist_end=float(d_end),
                gate_m=float(end_gate_m),
            ))

            if use_nav:
                end_pose.header.stamp.sec = 0
                end_pose.header.stamp.nanosec = 0
                ok_nav, nav_msg = self._nav_to_pose(end_pose, reason="align_to_segment_end")
                if not ok_nav:
                    self.publish_event(make_event("guard_align_end_failed", msg=str(nav_msg)))
                    return (False, -920, f"Guard end align failed: {nav_msg}")
                self.publish_event(make_event("guard_align_end_succeeded"))
                self._wait_until_stopped()
                return (True, 0, "")
            else:
                return (False, -921, f"Guard end gate violated (d_end={d_end:.2f}m > {end_gate_m:.2f}m)")

        return (True, 0, "")


def main() -> None:
    rclpy.init()
    node = GuardedCoverageExecutor()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
