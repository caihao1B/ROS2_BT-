import os
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from builtin_interfaces.msg import Time

from std_msgs.msg import String
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus

from nav2_msgs.action import FollowPath
try:
    from nav2_msgs.action import SmoothPath
except Exception:  # pragma: no cover
    SmoothPath = None  # type: ignore

try:
    from nav2_msgs.srv import ClearEntireCostmap
except Exception:  # pragma: no cover
    ClearEntireCostmap = None  # type: ignore

from .plan_io import load_plan, sha256_file, merge_short_segments, segment_to_path
from .state_store import StateStore, SegmentState, default_state_path
from .event_utils import make_event, dumps_event, safe_int


class CoverageExecutor(Node):
    def __init__(self) -> None:
        super().__init__("coverage_executor")

        # Parameters
        self.declare_parameter("plan_path", "")
        self.declare_parameter("resume_from_state", True)
        self.declare_parameter("state_file", "")
        self.declare_parameter("log_file", "")  # empty => ~/.ros/coverage_executor_log_<sha>.jsonl

        self.declare_parameter("use_smoother", False)
        self.declare_parameter("smoother_action", "/smooth_path")
        self.declare_parameter("smoother_id", "")
        self.declare_parameter("max_smoothing_duration_sec", 0.5)
        self.declare_parameter("check_smoother_collisions", False)

        self.declare_parameter("follow_action", "/follow_path")
        self.declare_parameter("controller_id", "")
        self.declare_parameter("goal_checker_id", "")
        self.declare_parameter("progress_checker_id", "")

        self.declare_parameter("merge_short_segments", True)
        self.declare_parameter("merge_length_threshold_m", 1.0)
        self.declare_parameter("merge_min_poses", 3)

        self.declare_parameter("retry_max", 2)  # retries AFTER the first attempt
        self.declare_parameter("skip_on_failure", True)
        self.declare_parameter("wait_after_fail_sec", 3.0)

        self.declare_parameter("clear_costmaps_on_fail", True)
        self.declare_parameter("clear_local_service", "/local_costmap/clear_entirely_local_costmap")
        self.declare_parameter("clear_global_service", "/global_costmap/clear_entirely_global_costmap")
        self.declare_parameter("clear_service_timeout_sec", 1.0)

        self.declare_parameter("enable_stuck_watchdog", True)
        self.declare_parameter("stuck_timeout_sec", 25.0)
        self.declare_parameter("stuck_cooldown_sec", 10.0)

        self.declare_parameter("current_path_topic", "/coverage/current_path")
        self.declare_parameter("events_topic", "/coverage/events")

        # Startup waits
        self.declare_parameter("startup_wait_for_nav2_sec", 60.0)
        self.declare_parameter("startup_wait_poll_sec", 1.0)
        self.declare_parameter("startup_wait_for_clock_sec", 30.0)
        self.declare_parameter("startup_wait_clock_poll_sec", 0.2)

        # Path stamping / frame normalization
        # - use_zero_stamp=True: set Path.header.stamp and every PoseStamped.header.stamp to 0
        #   so Nav2 will always use the latest TF (prevents "plan has 0 poses" caused by stale timestamps).
        self.declare_parameter("use_zero_stamp", True)
        # Force Path/PoseStamped frame_id to plan["frame_id"] (default: "map")
        self.declare_parameter("force_plan_frame_id", True)

        # Publishers
        self._current_path_pub = self.create_publisher(
            Path, self.get_parameter("current_path_topic").get_parameter_value().string_value, 1
        )
        self._events_pub = self.create_publisher(
            String, self.get_parameter("events_topic").get_parameter_value().string_value, 10
        )

        self._event_log_fp = None

        # Action clients (lazy)
        self._follow_client: Optional[ActionClient] = None
        self._smooth_client: Optional[ActionClient] = None

        # Stuck watchdog state
        self._last_progress_time = time.monotonic()
        self._last_waypoint = None
        self._last_stuck_event_time = 0.0

    # ----------------------------
    # Events
    # ----------------------------
    def publish_event(self, ev: Dict[str, Any]) -> None:
        msg = String()
        msg.data = dumps_event(ev)
        self._events_pub.publish(msg)
        if self._event_log_fp is not None:
            try:
                self._event_log_fp.write(msg.data + "\n")
                self._event_log_fp.flush()
            except Exception:
                pass

    # ----------------------------
    # Nav2 interactions
    # ----------------------------
    def _ensure_follow_client(self) -> ActionClient:
        if self._follow_client is None:
            name = self.get_parameter("follow_action").get_parameter_value().string_value
            self._follow_client = ActionClient(self, FollowPath, name)
        return self._follow_client

    def _ensure_smooth_client(self) -> Optional[ActionClient]:
        if SmoothPath is None:
            return None
        if self._smooth_client is None:
            name = self.get_parameter("smoother_action").get_parameter_value().string_value
            self._smooth_client = ActionClient(self, SmoothPath, name)
        return self._smooth_client

    def _follow_feedback_cb(self, fb_msg) -> None:
        try:
            fb = fb_msg.feedback
            if hasattr(fb, "current_waypoint"):
                idx = int(getattr(fb, "current_waypoint"))
                if self._last_waypoint is None or idx != self._last_waypoint:
                    self._last_waypoint = idx
                    self._last_progress_time = time.monotonic()
            else:
                self._last_progress_time = time.monotonic()
        except Exception:
            pass

    def wait_for_nav2_ready(self) -> bool:
        """Wait until FollowPath action server is available."""
        wait_total = float(self.get_parameter("startup_wait_for_nav2_sec").value)
        poll = float(self.get_parameter("startup_wait_poll_sec").value)
        poll = max(0.1, poll)

        client = self._ensure_follow_client()
        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < wait_total:
            if client.wait_for_server(timeout_sec=poll):
                self.get_logger().info("FollowPath action server is available.")
                return True
            self.get_logger().info("Waiting for FollowPath action server...")

        self.get_logger().error(f"FollowPath action server not available after {wait_total:.1f}s; aborting.")
        return False

    def wait_for_clock_active(self) -> None:
        """Wait until ROS time becomes non-zero when use_sim_time is enabled.

        IMPORTANT: /clock updates require spinning; sleep alone will keep now() at 0.
        """
        wait_total = float(self.get_parameter("startup_wait_for_clock_sec").value)
        poll = float(self.get_parameter("startup_wait_clock_poll_sec").value)
        poll = max(0.05, poll)

        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < wait_total:
            rclpy.spin_once(self, timeout_sec=poll)
            if self.get_clock().now().nanoseconds > 0:
                self.get_logger().info("Sim clock is active (now() > 0).")
                return

        self.get_logger().warn(
            f"Sim clock still 0 after {wait_total:.1f}s. Continuing anyway (may cause Path stamp issues)."
        )

    def normalize_path_headers(self, path: Path, plan_frame: str) -> Path:
        """Normalize Path headers so Nav2 can transform/crop reliably.

        - Ensures Path.header.frame_id == plan_frame and each PoseStamped.header.frame_id == plan_frame
        - Optionally forces stamp==0 (latest TF) to avoid stale timestamp issues during long runs
        """
        force_frame = bool(self.get_parameter("force_plan_frame_id").value)
        use_zero_stamp = bool(self.get_parameter("use_zero_stamp").value)

        if force_frame:
            path.header.frame_id = plan_frame
            for ps in path.poses:
                ps.header.frame_id = plan_frame

        if use_zero_stamp:
            z = Time(sec=0, nanosec=0)
            path.header.stamp = z
            for ps in path.poses:
                ps.header.stamp = z

        return path

    def smooth_path_best_effort(self, path: Path) -> Path:
        use_smoother = self.get_parameter("use_smoother").get_parameter_value().bool_value
        if not use_smoother:
            return path

        client = self._ensure_smooth_client()
        if client is None:
            self.get_logger().warn("SmoothPath not available; using raw path.")
            return path

        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("SmoothPath server not available; using raw path.")
            return path

        goal = SmoothPath.Goal()
        if hasattr(goal, "path"):
            goal.path = path

        smoother_id = self.get_parameter("smoother_id").get_parameter_value().string_value
        if hasattr(goal, "smoother_id"):
            goal.smoother_id = smoother_id

        max_dur = float(self.get_parameter("max_smoothing_duration_sec").value)
        if hasattr(goal, "max_smoothing_duration"):
            try:
                goal.max_smoothing_duration = Duration(seconds=max_dur).to_msg()
            except Exception:
                pass

        if hasattr(goal, "check_for_collisions"):
            goal.check_for_collisions = bool(self.get_parameter("check_smoother_collisions").value)

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("SmoothPath goal rejected; using raw path.")
            return path

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result()
        if res is None:
            self.get_logger().warn("SmoothPath result missing; using raw path.")
            return path

        try:
            smoothed = res.result.path
            if smoothed.poses:
                return smoothed
        except Exception:
            pass

        self.get_logger().warn("SmoothPath returned empty/invalid path; using raw path.")
        return path

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

        send_future = client.send_goal_async(goal, feedback_callback=self._follow_feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return (False, -2, "FollowPath goal rejected")

        result_future = goal_handle.get_result_async()

        enable_watchdog = bool(self.get_parameter("enable_stuck_watchdog").value)
        stuck_timeout = float(self.get_parameter("stuck_timeout_sec").value)
        stuck_cooldown = float(self.get_parameter("stuck_cooldown_sec").value)

        while not result_future.done():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)

            if enable_watchdog:
                now = time.monotonic()
                if (now - self._last_progress_time) > stuck_timeout and (now - self._last_stuck_event_time) > stuck_cooldown:
                    self._last_stuck_event_time = now
                    self.publish_event(make_event("stuck_watchdog"))

        res = result_future.result()
        if res is None:
            return (False, -3, "FollowPath result missing")

        status = int(res.status)
        error_code = getattr(res.result, "error_code", 0)
        error_msg = getattr(res.result, "error_msg", "")

        if status == GoalStatus.STATUS_SUCCEEDED and int(error_code) == 0:
            return (True, 0, "")
        return (False, safe_int(error_code, -4) or -4, str(error_msg))

    def clear_costmaps_best_effort(self) -> None:
        if ClearEntireCostmap is None:
            self.get_logger().warn("ClearEntireCostmap import failed; cannot clear costmaps.")
            return

        timeout = float(self.get_parameter("clear_service_timeout_sec").value)
        local_name = self.get_parameter("clear_local_service").get_parameter_value().string_value
        global_name = self.get_parameter("clear_global_service").get_parameter_value().string_value

        for name in [local_name, global_name]:
            if not name:
                continue
            try:
                client = self.create_client(ClearEntireCostmap, name)
                if not client.wait_for_service(timeout_sec=timeout):
                    self.get_logger().warn(f"Costmap clear service not available: {name}")
                    continue
                req = ClearEntireCostmap.Request()
                fut = client.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
                self.get_logger().info(f"Cleared costmap via {name}")
            except Exception as e:
                self.get_logger().warn(f"Failed to clear costmap via {name}: {e}")

    # ----------------------------
    # Main execution loop
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

        for seg in segments:
            seg_id = str(seg.get("id", ""))
            if seg_id and seg_id not in store.state.segments:
                store.state.segments[seg_id] = SegmentState()

        store.save()

        self.publish_event(make_event(
            "executor_started",
            plan_path=plan_path,
            plan_sha256=plan_sha,
            total_segments=len(segments),
            start_index=start_index,
        ))

        retry_max = int(self.get_parameter("retry_max").value)
        skip_on_failure = bool(self.get_parameter("skip_on_failure").value)
        wait_after_fail = float(self.get_parameter("wait_after_fail_sec").value)
        clear_costmaps = bool(self.get_parameter("clear_costmaps_on_fail").value)

        for i in range(start_index, len(segments)):
            seg = segments[i]
            seg_id = str(seg.get("id", f"seg_{i:04d}"))
            ss = store.state.segments.get(seg_id, SegmentState())

            attempt_limit = 1 + max(0, retry_max)
            seg_success = False

            for attempt in range(1, attempt_limit + 1):
                # Refresh Path on every attempt
                use_zero_stamp = bool(self.get_parameter("use_zero_stamp").value)
                stamp = Time(sec=0, nanosec=0) if use_zero_stamp else self.get_clock().now().to_msg()
                path_msg = segment_to_path(seg, stamp, default_frame=plan_frame)
                path_msg = self.normalize_path_headers(path_msg, plan_frame)
                self._current_path_pub.publish(path_msg)

                ss.attempts += 1
                store.state.segments[seg_id] = ss
                store.save()

                self.publish_event(make_event(
                    "segment_start",
                    segment_index=i,
                    segment_id=seg_id,
                    attempt=attempt,
                    kind=str(seg.get("kind", "")),
                ))

                path_to_follow = path_msg
                if bool(self.get_parameter("use_smoother").value):
                    path_to_follow = self.smooth_path_best_effort(path_msg)
                    path_to_follow = self.normalize_path_headers(path_to_follow, plan_frame)

                ok, err_code, err_msg = self.follow_path(path_to_follow)

                if ok:
                    seg_success = True
                    ss.status = "success"
                    ss.last_error_code = None
                    ss.last_error_msg = ""
                    store.state.current_index = i + 1
                    store.state.segments[seg_id] = ss
                    store.save()

                    self.publish_event(make_event(
                        "segment_success",
                        segment_index=i,
                        segment_id=seg_id,
                        attempt=attempt,
                    ))
                    break

                ss.status = "failed"
                ss.last_error_code = int(err_code)
                ss.last_error_msg = str(err_msg)
                store.state.segments[seg_id] = ss
                store.save()

                self.publish_event(make_event(
                    "segment_failed",
                    segment_index=i,
                    segment_id=seg_id,
                    attempt=attempt,
                    error_code=int(err_code),
                    error_msg=str(err_msg),
                ))

                if attempt < attempt_limit:
                    if wait_after_fail > 0.0:
                        time.sleep(wait_after_fail)
                    if clear_costmaps:
                        self.clear_costmaps_best_effort()
                    continue

            if not seg_success:
                if skip_on_failure:
                    ss.status = "skipped"
                    store.state.current_index = i + 1
                    store.state.segments[seg_id] = ss
                    store.save()
                    self.publish_event(make_event(
                        "segment_skipped",
                        segment_index=i,
                        segment_id=seg_id,
                        attempts=ss.attempts,
                        last_error_code=ss.last_error_code,
                        last_error_msg=ss.last_error_msg,
                    ))
                    continue

                self.publish_event(make_event(
                    "executor_aborted",
                    segment_index=i,
                    segment_id=seg_id,
                    attempts=ss.attempts,
                    last_error_code=ss.last_error_code,
                    last_error_msg=ss.last_error_msg,
                ))
                self.get_logger().error(f"Aborted at segment {i} ({seg_id})")
                return

        self.publish_event(make_event("executor_done", total_segments=len(segments)))
        self.get_logger().info("CoverageExecutor: all segments processed.")

        if self._event_log_fp is not None:
            try:
                self._event_log_fp.close()
            except Exception:
                pass
            self._event_log_fp = None


def main() -> None:
    rclpy.init()
    node = CoverageExecutor()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

