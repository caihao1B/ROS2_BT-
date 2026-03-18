import math
import os
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from nav2_msgs.action import FollowPath

from tf2_ros import Buffer, TransformListener  # type: ignore

try:
    from tf2_ros import TransformException  # type: ignore
except Exception:  # pragma: no cover
    TransformException = Exception  # type: ignore

from .event_utils import make_event, safe_int
from .executor_node import CoverageExecutor
from .plan_io import load_plan, merge_short_segments, segment_to_path, sha256_file
from .state_store import SegmentState, StateStore, default_state_path


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return quaternion (x,y,z,w) from planar yaw."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class NoBTRejoinCoverageExecutor(CoverageExecutor):
    """Non-BT baseline executor: MPPI + FollowPath + simple rejoin.

    - 不使用 BehaviorTree / coverage_bt_supervisor。
    - 只依赖 Nav2 controller_server 的 FollowPath（MPPI 负责局部避障）。
    - 由于路径被分成 segment：
        1) 每次发送 FollowPath 前，把 segment 路径裁剪到机器人最近点（trim），避免避障后回头找 segment 起点。
        2) 可选：如果机器人明显远离当前 segment，但非常接近后续 segment，则“跳段”到后续 segment 的最近点。
        3) 可选：stuck_watchdog 触发时主动 cancel FollowPath，强制进入重试。
    """

    def __init__(self) -> None:
        super().__init__()

        # TF parameters for robot pose in plan frame
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("tf_timeout_sec", 1.0)

        # Rejoin / trim (baseline logic)
        self.declare_parameter("enable_trim_to_robot", True)
        self.declare_parameter("enable_jump_to_future_segment", True)
        self.declare_parameter("jump_search_ahead_segments", 1)  # consider i..i+N
        self.declare_parameter("jump_current_far_m", 1.0)
        self.declare_parameter("jump_candidate_close_m", 0.7)
        self.declare_parameter("jump_hysteresis_m", 0.25)
        self.declare_parameter("min_remaining_poses_after_trim", 3)

        # When rejoining a path, don't rejoin *exactly* at the nearest pose.
        # A small lookahead reduces oscillations and avoids large heading changes.
        # (0.0 disables)
        self.declare_parameter("rejoin_lookahead_m", 0.50)

        # Optional: prepend the robot pose as the first pose of the path we send to FollowPath.
        # This can make controllers happier when the first path pose is not very close.
        self.declare_parameter("prepend_robot_pose", True)
        self.declare_parameter("prepend_if_dist_gt_m", 0.25)

        # Optional: cancel FollowPath goal when stuck watchdog fires (instead of only logging).
        self.declare_parameter("cancel_on_stuck", True)
        self.declare_parameter("stuck_cancel_wait_sec", 1.0)

        # --- Progress watchdog robustness ---
        # NOTE:
        # Some Nav2 FollowPath feedback versions do NOT expose `current_waypoint`.
        # The base class watchdog updates the timer on *any* feedback in that case,
        # which makes the watchdog useless when the robot is standing still.
        #
        # For the NoBT baseline we therefore detect progress from:
        #   1) distance_remaining decreasing (if available), else
        #   2) current_pose motion (if available), else
        #   3) fall back to the base behavior.
        self.declare_parameter("progress_min_trans_m", 0.05)
        self.declare_parameter("progress_min_dist_remaining_drop_m", 0.05)

        # Internal progress tracking
        self._last_fb_xy: Optional[Tuple[float, float]] = None
        self._last_fb_dist_remaining: Optional[float] = None
        self._last_fb_progress_time: float = time.monotonic()

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    # ----------------------------
    # Feedback / progress tracking
    # ----------------------------
    def _follow_feedback_cb(self, fb_msg) -> None:
        """Override to make stuck watchdog meaningful for FollowPath.

        The base CoverageExecutor uses `current_waypoint` when available, otherwise it
        considers any feedback as "progress".

        In Nav2 Humble, FollowPath feedback often contains `distance_remaining` and
        `current_pose`, but NOT `current_waypoint`. If we treat every feedback as progress,
        the watchdog never fires and the robot can stay stopped forever.
        """
        now = time.monotonic()
        try:
            fb = fb_msg.feedback
        except Exception:
            return

        # 1) Use waypoint index if present (works for some controller variants)
        try:
            if hasattr(fb, "current_waypoint"):
                idx = int(getattr(fb, "current_waypoint"))
                if self._last_waypoint is None or idx != self._last_waypoint:
                    self._last_waypoint = idx
                    self._last_progress_time = now
                    self._last_fb_progress_time = now
                return
        except Exception:
            pass

        # 2) Prefer distance_remaining decreasing (if present)
        try:
            if hasattr(fb, "distance_remaining"):
                d = float(getattr(fb, "distance_remaining"))
                if self._last_fb_dist_remaining is None:
                    self._last_fb_dist_remaining = d
                else:
                    drop = float(self._last_fb_dist_remaining - d)
                    min_drop = float(self.get_parameter("progress_min_dist_remaining_drop_m").value)
                    if drop >= min_drop:
                        self._last_fb_dist_remaining = d
                        self._last_progress_time = now
                        self._last_fb_progress_time = now
                        return
        except Exception:
            pass

        # 3) Otherwise use current_pose motion (if present)
        try:
            if hasattr(fb, "current_pose"):
                cp = getattr(fb, "current_pose")
                x = float(cp.pose.position.x)
                y = float(cp.pose.position.y)
                if self._last_fb_xy is None:
                    self._last_fb_xy = (x, y)
                else:
                    dx = x - float(self._last_fb_xy[0])
                    dy = y - float(self._last_fb_xy[1])
                    dist = math.hypot(dx, dy)
                    min_trans = float(self.get_parameter("progress_min_trans_m").value)
                    if dist >= min_trans:
                        self._last_fb_xy = (x, y)
                        self._last_progress_time = now
                        self._last_fb_progress_time = now
                        return
        except Exception:
            pass

        # 4) Fallback: DO NOT refresh progress timer here.
        # If we cannot detect progress signals, we would rather allow watchdog to fire
        # (and recover via cancel+retry) than hang forever.
        return

    # ----------------------------
    # TF helpers
    # ----------------------------
    def _lookup_robot_pose_2d(self, target_frame: str) -> Optional[Tuple[float, float, float]]:
        base = self.get_parameter("robot_base_frame").get_parameter_value().string_value
        timeout = float(self.get_parameter("tf_timeout_sec").value)
        use_latest = bool(self.get_parameter("use_latest_tf").value)

        t0 = time.monotonic()
        last_err: Optional[str] = None
        while rclpy.ok() and (time.monotonic() - t0) < timeout:
            try:
                if use_latest:
                    tf = self._tf_buffer.lookup_transform(target_frame, base, RclTime())
                else:
                    # For this baseline we do not track stamps; fallback to latest.
                    tf = self._tf_buffer.lookup_transform(target_frame, base, RclTime())
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
            self.publish_event(make_event("nobt_tf_failed", target_frame=target_frame, base_frame=base, err=last_err))
        return None

    # ----------------------------
    # Path trimming / rejoin
    # ----------------------------
    @staticmethod
    def _nearest_pose_index(path: Path, rx: float, ry: float) -> Tuple[int, float]:
        best_i = 0
        best_d = float("inf")
        for i, ps in enumerate(path.poses):
            dx = float(ps.pose.position.x) - rx
            dy = float(ps.pose.position.y) - ry
            d = math.hypot(dx, dy)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i, best_d

    @staticmethod
    def _trim_path(path: Path, start_i: int) -> Path:
        out = Path()
        out.header = path.header
        out.poses = list(path.poses[start_i:])
        return out

    @staticmethod
    def _index_ahead_by_distance(path: Path, start_i: int, lookahead_m: float) -> int:
        """Return an index >= start_i that is ~lookahead_m ahead along the path."""
        if lookahead_m <= 1e-6:
            return int(start_i)
        if start_i >= len(path.poses) - 1:
            return max(0, len(path.poses) - 1)

        acc = 0.0
        i = int(start_i)
        while i < len(path.poses) - 1:
            x1 = float(path.poses[i].pose.position.x)
            y1 = float(path.poses[i].pose.position.y)
            x2 = float(path.poses[i + 1].pose.position.x)
            y2 = float(path.poses[i + 1].pose.position.y)
            acc += math.hypot(x2 - x1, y2 - y1)
            if acc >= lookahead_m:
                return int(i + 1)
            i += 1
        return max(0, len(path.poses) - 1)

    def _maybe_prepend_robot_pose(self, path: Path, rx: float, ry: float, ryaw: float, dist_to_first: float) -> Path:
        if not bool(self.get_parameter("prepend_robot_pose").value):
            return path
        thresh = float(self.get_parameter("prepend_if_dist_gt_m").value)
        if dist_to_first <= thresh:
            return path

        ps = PoseStamped()
        ps.header.frame_id = path.header.frame_id
        ps.header.stamp.sec = 0
        ps.header.stamp.nanosec = 0
        ps.pose.position.x = float(rx)
        ps.pose.position.y = float(ry)
        qx, qy, qz, qw = _quat_from_yaw(float(ryaw))
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        out = Path()
        out.header = path.header
        out.poses = [ps] + list(path.poses)
        return out

    def _select_segment_and_path(
        self,
        segments: list,
        cur_i: int,
        plan_frame: str,
    ) -> Tuple[int, Path, Dict[str, Any]]:
        """Return (selected_segment_index, path_to_follow, debug_info)."""
        dbg: Dict[str, Any] = {"cur_i": int(cur_i)}

        # Default: use the entire segment path.
        if not bool(self.get_parameter("enable_trim_to_robot").value):
            path = segment_to_path(segments[cur_i], self.get_clock().now().to_msg(), default_frame=plan_frame)
            path = self.normalize_path_headers(path, plan_frame)
            return cur_i, path, dbg

        rpose = self._lookup_robot_pose_2d(plan_frame)
        if rpose is None:
            path = segment_to_path(segments[cur_i], self.get_clock().now().to_msg(), default_frame=plan_frame)
            path = self.normalize_path_headers(path, plan_frame)
            dbg["tf_ok"] = False
            return cur_i, path, dbg

        rx, ry, ryaw = rpose
        dbg.update({"tf_ok": True, "rx": float(rx), "ry": float(ry)})

        ahead = int(self.get_parameter("jump_search_ahead_segments").value)
        ahead = max(0, ahead)
        max_i = min(len(segments) - 1, cur_i + ahead)

        best = None  # (dist, seg_i, pose_i, full_path)
        cur_best = None

        for si in range(cur_i, max_i + 1):
            p = segment_to_path(segments[si], self.get_clock().now().to_msg(), default_frame=plan_frame)
            p = self.normalize_path_headers(p, plan_frame)
            if len(p.poses) < 2:
                continue
            pi, dist = self._nearest_pose_index(p, rx, ry)
            item = (float(dist), int(si), int(pi), p)
            if si == cur_i:
                cur_best = item
            if best is None or dist < best[0]:
                best = item

        if best is None or cur_best is None:
            # fallback: raw segment
            p = segment_to_path(segments[cur_i], self.get_clock().now().to_msg(), default_frame=plan_frame)
            p = self.normalize_path_headers(p, plan_frame)
            dbg["fallback"] = True
            return cur_i, p, dbg

        d0, _, p0_i, p0 = cur_best
        d1, s1_i, p1_i, p1 = best
        dbg.update(
            {
                "d_current": float(d0),
                "nearest_i_current": int(p0_i),
                "best_seg": int(s1_i),
                "d_best": float(d1),
                "nearest_i_best": int(p1_i),
            }
        )

        # Choose whether to jump to a future segment
        selected_seg_i = cur_i
        selected_pose_i = p0_i
        selected_path_full = p0

        do_jump = False
        if bool(self.get_parameter("enable_jump_to_future_segment").value) and s1_i > cur_i:
            cur_far = float(self.get_parameter("jump_current_far_m").value)
            cand_close = float(self.get_parameter("jump_candidate_close_m").value)
            hyst = float(self.get_parameter("jump_hysteresis_m").value)
            if (d0 > cur_far) and (d1 < cand_close) and ((d0 - d1) > hyst):
                do_jump = True
                selected_seg_i = s1_i
                selected_pose_i = p1_i
                selected_path_full = p1

        dbg["do_jump"] = bool(do_jump)

        # Optional lookahead: rejoin slightly ahead of the nearest pose.
        lookahead_m = float(self.get_parameter("rejoin_lookahead_m").value)
        selected_pose_i = self._index_ahead_by_distance(selected_path_full, selected_pose_i, lookahead_m)
        dbg["rejoin_lookahead_m"] = float(lookahead_m)
        dbg["rejoin_pose_i_after_lookahead"] = int(selected_pose_i)

        trimmed = self._trim_path(selected_path_full, selected_pose_i)

        # If trimming makes it too short, clamp to last N poses so FollowPath still has enough points.
        min_left = int(self.get_parameter("min_remaining_poses_after_trim").value)
        min_left = max(2, min_left)
        if len(trimmed.poses) < min_left:
            trimmed = Path()
            trimmed.header = selected_path_full.header
            trimmed.poses = list(selected_path_full.poses[-min_left:])
            dbg["trim_clamped"] = True
        else:
            dbg["trim_clamped"] = False

        # Prepend robot pose if requested
        if len(trimmed.poses) >= 1:
            fx = float(trimmed.poses[0].pose.position.x)
            fy = float(trimmed.poses[0].pose.position.y)
            d_first = math.hypot(fx - rx, fy - ry)
            trimmed = self._maybe_prepend_robot_pose(trimmed, rx, ry, ryaw, d_first)
            dbg["d_first"] = float(d_first)

        return selected_seg_i, trimmed, dbg

    # ----------------------------
    # FollowPath with optional stuck cancel
    # ----------------------------
    def follow_path(self, path: Path) -> Tuple[bool, int, str]:
        client = self._ensure_follow_client()
        if not client.wait_for_server(timeout_sec=5.0):
            return (False, -1, "FollowPath action server not available")

        goal = FollowPath.Goal()
        if hasattr(goal, "path"):
            goal.path = path

        controller_id = self.get_parameter("controller_id").get_parameter_value().string_value
        goal_checker_id = self.get_parameter("goal_checker_id").get_parameter_value().string_value
        progress_checker_id = self.get_parameter("progress_checker_id").get_parameter_value().string_value

        if hasattr(goal, "controller_id"):
            goal.controller_id = controller_id
        if hasattr(goal, "goal_checker_id"):
            goal.goal_checker_id = goal_checker_id
        if hasattr(goal, "progress_checker_id"):
            goal.progress_checker_id = progress_checker_id

        # Reset watchdog state for this attempt
        self._last_progress_time = time.monotonic()
        self._last_waypoint = None
        self._last_stuck_event_time = 0.0
        self._last_fb_xy = None
        self._last_fb_dist_remaining = None
        self._last_fb_progress_time = self._last_progress_time

        send_future = client.send_goal_async(goal, feedback_callback=self._follow_feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return (False, -2, "FollowPath goal rejected")

        result_future = goal_handle.get_result_async()

        enable_watchdog = bool(self.get_parameter("enable_stuck_watchdog").value)
        cancel_on_stuck = bool(self.get_parameter("cancel_on_stuck").value)
        stuck_timeout = float(self.get_parameter("stuck_timeout_sec").value)
        stuck_cooldown = float(self.get_parameter("stuck_cooldown_sec").value)
        cancel_wait = float(self.get_parameter("stuck_cancel_wait_sec").value)

        # Cancel at most once per attempt
        cancel_sent = False

        while not result_future.done():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)

            if enable_watchdog:
                now = time.monotonic()
                if (now - self._last_progress_time) > stuck_timeout and (now - self._last_stuck_event_time) > stuck_cooldown:
                    self._last_stuck_event_time = now
                    self.publish_event(make_event("stuck_watchdog"))

                    if cancel_on_stuck and not cancel_sent:
                        cancel_sent = True
                        self.publish_event(make_event("stuck_watchdog_cancel"))
                        try:
                            cancel_future = goal_handle.cancel_goal_async()
                            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=max(0.1, cancel_wait))
                        except Exception:
                            pass

        res = result_future.result()
        if res is None:
            return (False, -3, "FollowPath result missing")

        status = int(res.status)
        error_code = getattr(res.result, "error_code", 0)
        error_msg = getattr(res.result, "error_msg", "")

        if status == GoalStatus.STATUS_SUCCEEDED and int(error_code) == 0:
            return (True, 0, "")

        # Treat cancel as failure with a distinct code (helps analysis)
        if status == GoalStatus.STATUS_CANCELED and cancel_sent:
            return (False, -1001, "Canceled by stuck_watchdog")

        return (False, safe_int(error_code, -4) or -4, str(error_msg))

    # ----------------------------
    # Main execution loop (override): adds trim+jump logic
    # ----------------------------
    def run(self) -> None:
        plan_path = self.get_parameter("plan_path").get_parameter_value().string_value
        if not plan_path:
            self.get_logger().error("Parameter plan_path is empty.")
            return
        if not os.path.exists(plan_path):
            self.get_logger().error(f"plan_path does not exist: {plan_path}")
            return

        # Wait for Nav2 FollowPath server
        if not self.wait_for_nav2_ready():
            return

        # Wait for sim time to become active (critical!)
        self.wait_for_clock_active()

        plan = load_plan(plan_path)
        segments = list(plan.get("segments", []))
        plan_frame = str(plan.get("frame_id", "map"))

        plan_sha = sha256_file(plan_path)

        state_file = self.get_parameter("state_file").get_parameter_value().string_value
        if not state_file:
            state_file = default_state_path(plan_sha)
        store = StateStore(state_file)

        log_file = self.get_parameter("log_file").get_parameter_value().string_value
        if not log_file:
            base = os.path.expanduser("~/.ros")
            os.makedirs(base, exist_ok=True)
            log_file = os.path.join(base, f"coverage_executor_log_{plan_sha[:10]}.jsonl")
        try:
            self._event_log_fp = open(log_file, "a", encoding="utf-8")
            self.get_logger().info(f"Event log file: {log_file}")
        except Exception as e:
            self.get_logger().warn(f"Cannot open event log file {log_file}: {e}")
            self._event_log_fp = None

        resume = bool(self.get_parameter("resume_from_state").value)
        start_index = 0
        if resume and store.load() and store.state.plan_sha256 == plan_sha:
            start_index = int(store.state.current_index)
            self.get_logger().info(f"Resuming from state file {state_file}, segment index {start_index}")
        else:
            store.state.plan_sha256 = plan_sha
            store.state.current_index = 0
            store.state.segments = {}
            self.get_logger().info(f"Starting fresh (state file: {state_file})")

        if bool(self.get_parameter("merge_short_segments").value):
            segments = merge_short_segments(
                segments,
                float(self.get_parameter("merge_length_threshold_m").value),
                int(self.get_parameter("merge_min_poses").value),
            )

        # Ensure segment states exist
        for idx, seg in enumerate(segments):
            seg_id = str(seg.get("id", f"seg_{idx:04d}"))
            if seg_id and seg_id not in store.state.segments:
                store.state.segments[seg_id] = SegmentState()
        store.save()

        self.publish_event(
            make_event(
                "executor_started",
                plan_path=plan_path,
                plan_sha256=plan_sha,
                total_segments=len(segments),
                start_index=start_index,
                mode="NoBTRejoin",
            )
        )

        retry_max = int(self.get_parameter("retry_max").value)
        skip_on_failure = bool(self.get_parameter("skip_on_failure").value)
        wait_after_fail = float(self.get_parameter("wait_after_fail_sec").value)
        clear_costmaps = bool(self.get_parameter("clear_costmaps_on_fail").value)

        i = int(start_index)
        while rclpy.ok() and i < len(segments):
            # --- select segment/path based on current robot pose ---
            selected_i, path_msg, dbg = self._select_segment_and_path(segments, i, plan_frame)

            if selected_i != i:
                # Mark intermediate segments as skipped (baseline jump)
                for k in range(i, selected_i):
                    segk = segments[k]
                    segk_id = str(segk.get("id", f"seg_{k:04d}"))
                    ssk = store.state.segments.get(segk_id, SegmentState())
                    ssk.status = "skipped"
                    store.state.segments[segk_id] = ssk
                    self.publish_event(
                        make_event(
                            "segment_skipped",
                            segment_index=k,
                            segment_id=segk_id,
                            attempts=ssk.attempts,
                            reason="jump_to_future_segment",
                        )
                    )
                store.state.current_index = int(selected_i)
                store.save()
                self.publish_event(make_event("nobt_jump", from_index=int(i), to_index=int(selected_i), debug=dbg))
                i = int(selected_i)
                continue

            # Now executing segment i
            seg = segments[i]
            seg_id = str(seg.get("id", f"seg_{i:04d}"))
            ss = store.state.segments.get(seg_id, SegmentState())

            attempt_limit = 1 + max(0, retry_max)
            seg_success = False

            for attempt in range(1, attempt_limit + 1):
                # Publish the path that we will actually follow (trimmed+prepended)
                self._current_path_pub.publish(path_msg)

                ss.attempts += 1
                store.state.segments[seg_id] = ss
                store.save()

                self.publish_event(
                    make_event(
                        "segment_start",
                        segment_index=i,
                        segment_id=seg_id,
                        attempt=attempt,
                        kind=str(seg.get("kind", "")),
                        nobt_trim_debug=dbg,
                    )
                )

                ok, err_code, err_msg = self.follow_path(path_msg)

                if ok:
                    seg_success = True
                    ss.status = "success"
                    ss.last_error_code = None
                    ss.last_error_msg = ""
                    store.state.current_index = i + 1
                    store.state.segments[seg_id] = ss
                    store.save()

                    self.publish_event(make_event("segment_success", segment_index=i, segment_id=seg_id, attempt=attempt))
                    break

                ss.status = "failed"
                ss.last_error_code = int(err_code)
                ss.last_error_msg = str(err_msg)
                store.state.segments[seg_id] = ss
                store.save()

                self.publish_event(
                    make_event(
                        "segment_failed",
                        segment_index=i,
                        segment_id=seg_id,
                        attempt=attempt,
                        error_code=int(err_code),
                        error_msg=str(err_msg),
                    )
                )

                if attempt < attempt_limit:
                    if wait_after_fail > 0.0:
                        time.sleep(wait_after_fail)
                    if clear_costmaps:
                        self.clear_costmaps_best_effort()

                    # Recompute trimmed path for the next retry (robot may have moved)
                    selected_i2, path_msg2, dbg2 = self._select_segment_and_path(segments, i, plan_frame)
                    if selected_i2 != i:
                        # If we would jump on retry, do it immediately
                        self.publish_event(
                            make_event(
                                "nobt_jump_on_retry",
                                from_index=int(i),
                                to_index=int(selected_i2),
                                debug=dbg2,
                            )
                        )
                        i = int(selected_i2)
                        break
                    path_msg, dbg = path_msg2, dbg2
                    continue

            if not seg_success:
                # If we broke out of attempts because we jumped, continue with new i.
                if i != int(store.state.current_index):
                    continue

                if skip_on_failure:
                    ss.status = "skipped"
                    store.state.current_index = i + 1
                    store.state.segments[seg_id] = ss
                    store.save()
                    self.publish_event(
                        make_event(
                            "segment_skipped",
                            segment_index=i,
                            segment_id=seg_id,
                            attempts=ss.attempts,
                            last_error_code=ss.last_error_code,
                            last_error_msg=ss.last_error_msg,
                        )
                    )
                    i += 1
                    continue

                self.publish_event(
                    make_event(
                        "executor_aborted",
                        segment_index=i,
                        segment_id=seg_id,
                        attempts=ss.attempts,
                        last_error_code=ss.last_error_code,
                        last_error_msg=ss.last_error_msg,
                    )
                )
                self.get_logger().error(f"Aborted at segment {i} ({seg_id})")
                return

            i += 1

        self.publish_event(make_event("executor_done", total_segments=len(segments)))
        self.get_logger().info("NoBTRejoinCoverageExecutor: all segments processed.")

        if self._event_log_fp is not None:
            try:
                self._event_log_fp.close()
            except Exception:
                pass
            self._event_log_fp = None


def main() -> None:
    rclpy.init()
    node: Node = NoBTRejoinCoverageExecutor()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
