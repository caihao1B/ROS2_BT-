import copy
import json
import math
import os
import time
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav2_msgs.action import FollowPath, NavigateToPose, ComputePathToPose
from nav2_msgs.msg import SpeedLimit
from action_msgs.msg import GoalStatus

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener  # type: ignore
try:
    from tf2_ros import TransformException  # type: ignore
except Exception:  # pragma: no cover
    TransformException = Exception  # type: ignore

from coverage_msgs.msg import StrategyDecisionStamped
from coverage_msgs.msg import ObstaclePredictionArray, ObstacleTrackArray
from coverage_msgs.srv import D1Evaluate, D2Replan

try:
    # Optional: used only for lightweight "map confidence" checks.
    # If unavailable, the supervisor will simply skip those checks.
    from nav2_msgs.msg import Costmap  # type: ignore
except Exception:  # pragma: no cover
    Costmap = None  # type: ignore


try:
    from nav2_msgs.srv import ClearEntireCostmap  # type: ignore
except Exception:  # pragma: no cover
    ClearEntireCostmap = None  # type: ignore


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


@dataclass
class _Seg:
    sid: str
    cluster_id: int
    frame_id: str
    kind: str
    poses: List[PoseStamped]


@dataclass
class _DetourCandidate:
    """A candidate detour target + how to resume coverage after reaching it."""

    target_pose: PoseStamped
    resume_seg_idx: int
    resume_pose_idx: int
    # Coverage that will be left for "cleanup" later
    skipped: List[Tuple[int, int, int]]  # list of (seg_idx, start_pose_i, end_pose_i)
    note: str = ""


class CoverageBTSupervisor(Node):
    """
    A BT-like reactive supervisor that uses FollowPath as primitive action and executes A–E.
    Inputs:
      - /coverage/strategy_decision  (StrategyDecisionStamped)
    Publishes:
      - /coverage/current_path       (Path)  current executing segment path for RViz + selector
      - /speed_limit                (nav2_msgs/SpeedLimit) for Mode B
    Actions:
      - /follow_path                (FollowPath)
      - /compute_path_to_pose       (ComputePathToPose) used for Mode D detour
      - /navigate_to_pose           (NavigateToPose) optional for large rejoin (unused by default)
    """

    def __init__(self):
        super().__init__('coverage_bt_supervisor')

        # plan + topics
        self.declare_parameter('plan_path', '')
        self.declare_parameter('decision_topic', '/coverage/strategy_decision')
        self.declare_parameter('current_path_topic', '/coverage/current_path')

        # actions
        self.declare_parameter('follow_action', '/follow_path')
        self.declare_parameter('compute_path_action', '/compute_path_to_pose')
        self.declare_parameter('navigate_action', '/navigate_to_pose')

        # D1 / D2 advisor integration
        self.declare_parameter('use_d12_advisors', True)
        self.declare_parameter('region_state_topic', '/coverage/region_state/summary')
        self.declare_parameter('recovery_event_topic', '/coverage/recovery_event')
        self.declare_parameter('d1_service', '/coverage/d1_advisor/evaluate')
        self.declare_parameter('d2_service', '/coverage/d2_advisor/replan')
        self.declare_parameter('d12_rpc_timeout_sec', 3.5)
        self.declare_parameter('d1_score_threshold', 4.5)
        self.declare_parameter('d2_score_threshold', 4.5)
        self.declare_parameter('d2_enable', True)
        self.declare_parameter('d2_candidate_limit', 5)
        self.declare_parameter('d2_blocked_zone_pad_m', 0.12)
        self.declare_parameter('d2_blocked_zone_radius_cap_m', 0.60)
        self.declare_parameter('d_plan_bbox_pad_m', 0.45)

        # D2 deterministic lab hooks (used only for explicit chain tests)
        self.declare_parameter('d2_test_force_once_enable', False)
        self.declare_parameter('d2_test_force_snapshot_path', '')
        self.declare_parameter('d2_test_force_active_seg_min', -1)
        self.declare_parameter('d2_test_force_active_seg_max', -1)
        self.declare_parameter('d2_test_force_use_live_robot_pose', True)
        self.declare_parameter('d2_test_force_use_live_cluster_id', True)
        self.declare_parameter('d2_disable_after_first_accept', False)
        self.declare_parameter('d2_disable_on_runtime_segments', False)
        self.declare_parameter('d2_require_hard_d2', False)

        # frames
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('use_latest_tf', True)

        # behavior thresholds
        self.declare_parameter('wait_clear_hold_sec', 0.6)
        self.declare_parameter('wait_max_sec', 20.0)
        self.declare_parameter('defer_max_times', 2)
        self.declare_parameter('replan_max_times', 3)

        # --- C / D upgrades (segment slicing + cluster-aware defers) ---
        # Topics
        self.declare_parameter('tracks_topic', '/coverage/obstacles/tracks')
        self.declare_parameter('predictions_topic', '/coverage/obstacles/predictions')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('clear_local_service', '/local_costmap/clear_entirely_local_costmap')
        self.declare_parameter('clear_global_service', '/global_costmap/clear_entirely_global_costmap')
        self.declare_parameter('clear_service_timeout_sec', 0.8)
        self.declare_parameter('stale_shadow_cleanup_enable', True)
        self.declare_parameter('stale_shadow_no_live_sec', 1.0)
        self.declare_parameter('stale_shadow_clear_cooldown_sec', 4.0)
        self.declare_parameter('require_live_evidence_for_blocking_modes', True)
        self.declare_parameter('action_watchdog_enable', True)
        self.declare_parameter('action_watchdog_grace_sec', 5.0)
        self.declare_parameter('action_watchdog_timeout_sec', 18.0)
        self.declare_parameter('action_watchdog_min_progress_m', 0.12)
        self.declare_parameter('action_watchdog_segment_max_sec', 35.0)

        # Robot geometry (used for conservative rejoin selection)
        self.declare_parameter('robot_radius_m', 0.22)

        # General scheduling / robustness
        self.declare_parameter('enable_trim_on_segment_start', True)
        self.declare_parameter('min_remaining_poses_after_trim', 3)
        self.declare_parameter('cluster_strict_defer', True)  # keep defers within cluster when possible
        self.declare_parameter('defer_cooldown_sec', 12.0)  # avoid thrashing the same segment
        self.declare_parameter('skip_after_failures', 4)  # hard stop to avoid infinite loops

        # L2: steadiness / anti-thrashing knobs
        self.declare_parameter('preserve_plan_order', True)  # keep queue order; don't rotate deferred segments to the tail
        self.declare_parameter('decision_stable_sec', 0.25)  # decision mode must be stable for this long before acting (C/D/E)
        self.declare_parameter('mode_min_interval_sec', 0.60)  # minimum time between mode-trigger actions (C/D/E) to avoid jitter
        self.declare_parameter('c_trigger_cooldown_sec', 1.5)  # per-segment min interval between C triggers
        self.declare_parameter('d_trigger_cooldown_sec', 3.0)  # per-segment min interval between D triggers
        self.declare_parameter('d_requires_c_first', True)  # when selector asks for D, try C first (keeps plan order)
        self.declare_parameter('d_bypass_c_if_region_hard', True)  # when region_state says the cluster is structurally bad, let D skip the C-first policy
        # Cross-segment policy: once a segment has been substantially executed, a successful
        # jump to another segment should normally *close* the origin residual instead of
        # putting it back into the queue. This avoids seg001 -> seg002 -> seg001 bouncing.
        self.declare_parameter('cross_seg_close_origin_enable', True)
        self.declare_parameter('cross_seg_close_origin_progress_ratio', 0.70)
        self.declare_parameter('cross_seg_close_origin_remaining_m', 1.20)
        # Similar policy for defer: if a segment is already mostly done when it is deferred,
        # treat the residual as an accepted skip (mission-closed, not re-queued).
        self.declare_parameter('defer_close_on_major_progress', True)
        self.declare_parameter('defer_close_progress_ratio', 0.70)
        self.declare_parameter('defer_close_remaining_m', 1.20)

        # L2: FollowPath ABORTED(6) recovery
        self.declare_parameter('abort_use_d_replan_first', True)  # ABORTED -> try D first, then defer
        self.declare_parameter('abort_replan_cooldown_sec', 6.0)  # avoid repeated abort->D loops
        self.declare_parameter('abort_cluster_jump_after_failures', 2)  # allow cluster-jump only after N failures on same seg

        # L2: D fallback rejoin candidates (when no tracks / no decision rejoin pose)
        self.declare_parameter('d_fallback_rejoin_min_ahead_m', 1.5)
        self.declare_parameter('d_fallback_rejoin_step_m', 1.0)
        self.declare_parameter('d_fallback_rejoin_max_trials', 6)


        # Mode C: slice-skip + detour + resume
        self.declare_parameter('c_enable_slice', True)
        self.declare_parameter('c_corridor_radius_m', 0.90)  # match selector/path corridor
        self.declare_parameter('c_skip_margin_m', 0.35)  # extra safety margin around obstacle radius
        self.declare_parameter('c_rejoin_min_ahead_m', 1.20)  # how far ahead of block point we try to rejoin
        self.declare_parameter('c_rejoin_step_m', 0.75)  # if first rejoin fails, try further ahead
        self.declare_parameter('c_rejoin_max_trials', 8)
        self.declare_parameter('c_max_attempts_per_segment', 3)
        self.declare_parameter('c_allow_rejoin_next_segment', True)
        self.declare_parameter('c_revisit_origin_on_rejoin_next_segment', False)  # False: keep planned order; don't push origin seg back after cross-seg rejoin
        # C1 / C2 split:
        #  - C1: same-segment local rejoin only
        #  - C2: ordered exit recovery onto the *next* segment, optionally a little ahead on that next segment
        self.declare_parameter('c2_enable_ordered_exit_rejoin', True)
        self.declare_parameter('c2_next_segment_points_per_segment', 4)  # start + N-1 points ahead (ordered)
        self.declare_parameter('c2_next_segment_step_m', 0.60)
        self.declare_parameter('c2_next_segment_start_offset_m', 0.00)
        # Turn connectors in the offline plan may carry cluster_id=-1. Allow C2 to still
        # target the immediate next queued segment in plan order instead of escalating to D.
        self.declare_parameter('c2_turn_use_next_queue_segment', True)
        self.declare_parameter('c2_allow_ordered_fallback_without_cluster', True)
        # Finish-first fallback: when D has no candidate on a short turn, try C2 ordered-exit once
        # before deferring. This reduces seg_0013-style oscillation on micro maps.
        self.declare_parameter('d_turn_try_c2_before_defer', True)

        # Mode D: cluster recovery (jump to another reachable segment in current cluster)
        self.declare_parameter('d_enable_cluster_recovery', True)
        self.declare_parameter('d_candidate_max_segments', 3)
        self.declare_parameter('d_cluster_jump_sort', 'queue')  # 'queue' (keep segment order) or 'distance'
        self.declare_parameter('d_max_attempts_per_segment', 3)
        # Finite jump window: allow only local forward jumps, not large or backward jumps.
        self.declare_parameter('jump_limit_enable', True)
        self.declare_parameter('jump_limit_max_segment_delta', 2)
        self.declare_parameter('jump_limit_max_linear_m', 4.5)
        # Main-pass monotonicity: keep only a small revisit window for deferred segments.
        self.declare_parameter('main_pass_revisit_window_segments', 1)
        self.declare_parameter('main_pass_revisit_window_m', 4.5)
        self.declare_parameter('main_pass_cleanup_far_deferred', True)

        # Active observe (for rejoin / planner failures)
        self.declare_parameter('observe_enable', True)
        self.declare_parameter('observe_spin_rad', 2.0 * math.pi)
        self.declare_parameter('observe_angular_vel', 0.60)
        self.declare_parameter('observe_backup_sec', 0.0)
        self.declare_parameter('observe_backup_vel', -0.08)
        self.declare_parameter('observe_settle_sec', 0.60)
        self.declare_parameter('observe_side_step_enable', False)
        self.declare_parameter('observe_side_step_dist_m', 0.35)

        # Detour stop-hold before computing (avoid creeping into obstacle)
        self.declare_parameter('detour_stop_hold_sec', 0.35)
        self.declare_parameter('detour_delay_compute_sec', 0.20)

        # Detour watchdog (avoid long stalls waiting for Nav2 progress checker)
        self.declare_parameter('detour_watchdog_enable', True)
        self.declare_parameter('detour_watchdog_grace_sec', 6.0)
        self.declare_parameter('detour_watchdog_timeout_sec', 25.0)
        self.declare_parameter('detour_watchdog_min_progress_m', 0.15)

        # Optional "confidence" gating using costmap (best-effort)
        self.declare_parameter('costmap_max_age_sec', 2.0)
        self.declare_parameter('costmap_unknown_bad_frac', 0.30)
        self.declare_parameter('costmap_min_obstacle_frac', 0.002)

        # Cleanup logging (skipped slices / segments)
        self.declare_parameter('cleanup_log_path', '~/.ros/coverage_cleanup_todo.jsonl')

        # B_YIELD speed limit topic
        self.declare_parameter('speed_limit_topic', '/speed_limit')
        # Turn skill hint: slow down planned 'turn' segments so MPPI + RotationShim can track them
        # with less corner-cutting near static obstacles.
        self.declare_parameter('turn_segment_enable_speed_limit', True)
        self.declare_parameter('turn_segment_speed_limit_ratio', 0.40)
        # Turn-skill pre-align: for very short turn connectors near obstacles, rotate in place
        # to the segment heading before handing control to FollowPath. This avoids many
        # MPPI/RotationShim ABORTED(6) cases at row->turn transitions.
        self.declare_parameter('turn_prealign_enable', True)
        self.declare_parameter('turn_prealign_heading_error_rad', 0.90)
        self.declare_parameter('turn_prealign_yaw_tolerance_rad', 0.18)
        self.declare_parameter('turn_prealign_angular_vel', 0.85)
        self.declare_parameter('turn_prealign_timeout_sec', 6.0)
        self.declare_parameter('turn_prealign_timeout_continue_if_err_below_rad', 0.35)
        # ABORTED(6) routing: prefer local C-recovery before D-replan.
        self.declare_parameter('abort_use_c_recovery_first', True)
        self.declare_parameter('abort_c_escalate_after_failures', 2)
        self.declare_parameter('abort_short_segment_length_m', 1.50)
        self.declare_parameter('abort_turn_prefer_c2', True)
        # Tiny residuals: if only a very small tail is left, don't re-queue the segment.
        self.declare_parameter('tiny_residual_ignore_enable', True)
        self.declare_parameter('tiny_residual_length_m', 0.25)
        self.declare_parameter('tiny_residual_turn_length_m', 0.45)
        # Endgame finish-first closeout: on the last one or two small residual segments, prefer
        # closing the task over repeatedly deferring tiny leftovers.
        self.declare_parameter('endgame_finish_enable', True)
        self.declare_parameter('endgame_remaining_segments_le', 1)
        self.declare_parameter('endgame_close_remaining_m', 0.60)
        # C-focused refinements for static/manual obstacle tests:
        #  - short backup before computing C1/C2 when the blocker is very close or the candidate set is tiny
        #  - reject obviously bad/long detour paths so C does not wander far away from the local area
        self.declare_parameter('c_prebackup_enable', True)
        self.declare_parameter('c_prebackup_if_candidates_le', 2)
        self.declare_parameter('c_prebackup_obs_dist_m', 0.70)
        self.declare_parameter('c_prebackup_obs_ahead_m', 1.20)
        self.declare_parameter('c_prebackup_backup_sec', 0.90)
        self.declare_parameter('c_prebackup_backup_vel', -0.08)
        self.declare_parameter('c_prebackup_settle_sec', 0.35)
        self.declare_parameter('c_detour_max_length_m', 6.0)
        self.declare_parameter('c2_detour_max_length_m', 8.0)
        self.declare_parameter('c_detour_max_stretch_ratio', 3.5)
        self.declare_parameter('c_detour_max_initial_backtrack_m', 0.40)
        # C dynamic refresh: while following a C1/C2 detour, if a new obstacle appears and
        # the robot stops making progress, recompute a path to the *same* rejoin target first
        # (similar in spirit to NavigateToPose periodic replanning) before escalating / rotating.
        self.declare_parameter('c_dynamic_refresh_enable', True)
        self.declare_parameter('c_dynamic_refresh_no_progress_sec', 6.0)
        self.declare_parameter('c_dynamic_refresh_cooldown_sec', 4.0)
        self.declare_parameter('c_dynamic_refresh_max_per_candidate', 2)
        self.declare_parameter('c_dynamic_refresh_backup_sec', 0.60)
        self.declare_parameter('c_dynamic_refresh_backup_vel', -0.06)
        self.declare_parameter('c_dynamic_refresh_settle_sec', 0.30)
        self.declare_parameter('c_wait_resume_recompute', True)
        self.declare_parameter('c_to_d_preempt_enable', True)
        self.declare_parameter('c_to_d_preempt_refresh_ge', 2)
        self.declare_parameter('c_to_d_preempt_path_rejected_ge', 1)
        self.declare_parameter('c_to_d_preempt_allow_cluster_jump', False)
        self.declare_parameter('c_to_d_preempt_force_cluster_jump', False)

        plan_path = self.get_parameter('plan_path').value
        if not plan_path:
            self.get_logger().error("Parameter plan_path is empty.")
            raise RuntimeError("plan_path empty")

        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)

        self._decision: Optional[StrategyDecisionStamped] = None
        # L2: decision stability tracking (to avoid mode jitter)
        now_mono = time.monotonic()
        self._decision_mode_last: str = "NONE"
        self._decision_mode_since_mono: float = now_mono
        self._last_mode_action_mono: float = 0.0

        self.create_subscription(StrategyDecisionStamped, self.get_parameter('decision_topic').value, self._cb_decision, 10)

        self._region_state_cache: Dict[str, Any] = {}
        self.create_subscription(String, self.get_parameter('region_state_topic').value, self._cb_region_state, 10)
        self._pub_recovery_event = self.create_publisher(String, self.get_parameter('recovery_event_topic').value, 50)
        # D1/D2 RPC calls are issued from the periodic _tick() timer callback. In rclpy, waiting
        # synchronously for a service future from inside a callback can deadlock if the client and
        # the waiting callback share the same MutuallyExclusive callback group. Keep these clients
        # in a dedicated Reentrant callback group and use a MultiThreadedExecutor in main().
        self._rpc_cb_group = ReentrantCallbackGroup()
        self._d1_client = self.create_client(
            D1Evaluate,
            self.get_parameter('d1_service').value,
            callback_group=self._rpc_cb_group,
        )
        self._d2_client = self.create_client(
            D2Replan,
            self.get_parameter('d2_service').value,
            callback_group=self._rpc_cb_group,
        )

        # Optional perception inputs (used only for upgraded C/D)
        self._tracks: Optional[ObstacleTrackArray] = None
        self._preds: Optional[ObstaclePredictionArray] = None
        self._costmap = None
        self._costmap_recv_time = 0.0
        self.create_subscription(ObstacleTrackArray, self.get_parameter('tracks_topic').value, self._cb_tracks, 10)
        self.create_subscription(ObstaclePredictionArray, self.get_parameter('predictions_topic').value, self._cb_preds, 10)
        if Costmap is not None:
            try:
                self.create_subscription(Costmap, self.get_parameter('costmap_topic').value, self._cb_costmap, 10)
            except Exception:
                # If the topic type mismatches, we keep running without costmap gating.
                self.get_logger().warn("[supervisor] cannot subscribe costmap_topic (type mismatch?) -> costmap gating disabled")

        self._tracks_recv_mono: float = -1.0e9
        self._preds_recv_mono: float = -1.0e9
        self._last_nonempty_tracks_mono: float = -1.0e9
        self._last_nonempty_preds_mono: float = -1.0e9
        self._last_shadow_clear_mono: float = -1.0e9
        self._last_d2_rpc_status: str = 'idle'
        self._last_d2_rpc_message: str = ''
        self._last_d2_rpc_elapsed_sec: float = 0.0
        if ClearEntireCostmap is not None:
            self._clear_local_cli = self.create_client(
                ClearEntireCostmap,
                self.get_parameter('clear_local_service').value,
                callback_group=self._rpc_cb_group,
            )
            self._clear_global_cli = self.create_client(
                ClearEntireCostmap,
                self.get_parameter('clear_global_service').value,
                callback_group=self._rpc_cb_group,
            )
        else:
            self._clear_local_cli = None
            self._clear_global_cli = None

        self._pub_cmd = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)

        self._pub_path = self.create_publisher(Path, self.get_parameter('current_path_topic').value, 10)

        speed_topic = self.get_parameter('speed_limit_topic').value
        self._pub_speed = self.create_publisher(SpeedLimit, speed_topic, 10)

        # action clients
        self._follow = ActionClient(self, FollowPath, self.get_parameter('follow_action').value)
        self._compute = ActionClient(self, ComputePathToPose, self.get_parameter('compute_path_action').value)
        self._nav = ActionClient(self, NavigateToPose, self.get_parameter('navigate_action').value)

        # load plan
        self._segments: List[_Seg] = self._load_plan(plan_path)
        self._base_plan_segment_count: int = len(self._segments)
        self._queue: List[int] = list(range(len(self._segments)))
        self._retired_segments: Set[int] = set()
        self._pending_d2_scope_supersede: Set[int] = set()
        self._d2_test_force_fired: bool = False
        self._d2_test_force_snapshot_cache: Optional[Dict[str, Any]] = None
        self._d2_test_force_snapshot_cache_path: str = ''
        self._d2_one_shot_consumed: bool = False
        self._d2_accept_count: int = 0
        self._last_d2_guard_skip: str = ''
        self._main_pass_frontier_idx: int = -1
        self._defer_count = {i: 0 for i in range(len(self._segments))}
        self._replan_count = {i: 0 for i in range(len(self._segments))}
        self._c_slice_count: Dict[int, int] = {i: 0 for i in range(len(self._segments))}
        self._d_recovery_count: Dict[int, int] = {i: 0 for i in range(len(self._segments))}

        # Cooldowns / failure guards
        self._cooldown_until: Dict[int, float] = {i: 0.0 for i in range(len(self._segments))}
        self._fail_count: Dict[int, int] = {i: 0 for i in range(len(self._segments))}

        # L2: per-segment mode cooldown timestamps (monotonic seconds)
        self._c_last_trigger_mono: Dict[int, float] = {i: -1e9 for i in range(len(self._segments))}
        self._d_last_trigger_mono: Dict[int, float] = {i: -1e9 for i in range(len(self._segments))}
        self._abort_last_replan_mono: Dict[int, float] = {i: -1e9 for i in range(len(self._segments))}
        self._mission_complete_announced: bool = False

        # Detour state (used for both C and D)
        self._follow_kind: str = "SEGMENT"  # SEGMENT | DETOUR | OBSERVE
        self._detour_reason: str = ""  # C or D
        self._detour_candidates: List[_DetourCandidate] = []
        self._detour_cand_i: int = 0
        self._detour_attempt_started_t: Optional[float] = None
        self._detour_observe_used: int = 0
        self._detour_active: Optional[_DetourCandidate] = None
        self._detour_origin_seg_idx: Optional[int] = None
        self._detour_origin_pose_idx: int = 0
        self._detour_origin_total_len: float = 0.0
        self._detour_origin_remaining_len: float = -1.0
        self._detour_origin_progress_ratio: float = 0.0
        # Detour stop-hold / delay (avoid creeping into obstacle while we switch modes)
        self._detour_stop_hold_until: float = 0.0
        self._detour_delay_until: float = 0.0

        # Detour watchdog state (monotonic time + last xy)
        self._detour_wd_started_t: float = 0.0
        self._detour_wd_last_progress_t: float = 0.0
        self._detour_wd_last_xy: Optional[Tuple[float, float]] = None
        # Generic segment action watchdog (anti-stall governor for nominal FollowPath)
        self._action_wd_started_t: float = 0.0
        self._action_wd_last_progress_t: float = 0.0
        self._action_wd_last_xy: Optional[Tuple[float, float]] = None
        # C dynamic detour refresh (recompute same target if a newly appeared obstacle blocks detour)
        self._detour_refresh_count: int = 0
        self._detour_last_refresh_t: float = -1e9


        # Observe state machine
        self._observe_steps: List[Tuple[Twist, float]] = []  # (cmd, duration_sec)
        self._observe_step_deadline: float = 0.0
        self._observe_step_i: int = 0
        self._observe_after_state: str = ""  # where to go after observe completes
        self._observe_after_follow_kind: str = "SEGMENT"
        self._observe_note: str = ""

        # state
        self._active_seg_idx: Optional[int] = None
        self._active_path: Optional[Path] = None
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._cancel_future = None

        # compute_path action state
        self._compute_future = None
        self._compute_goal_handle = None
        self._compute_result_future = None

        self._mode_state = "IDLE"
        self._wait_started_t = None
        self._clear_ok_since = None
        self._wait_resume_kind: str = 'SEGMENT'
        self._turn_prealign_target_yaw: float = 0.0
        self._turn_prealign_deadline: float = 0.0

        self.get_logger().info(f"[supervisor] loaded segments={len(self._segments)} from {plan_path}")

        # periodic tick
        # Keep timer handle to avoid garbage collection stopping periodic ticks
        self._tick_timer = self.create_timer(0.10, self._tick)

    def _cb_decision(self, msg: StrategyDecisionStamped):
        self._decision = msg
        # L2: mode stability tracking (ignore very short mode flickers)
        try:
            mode = str(msg.decision.mode)
        except Exception:
            mode = "NONE"
        now = time.monotonic()
        if mode != self._decision_mode_last:
            self._decision_mode_last = mode
            self._decision_mode_since_mono = now

    def _cb_tracks(self, msg: ObstacleTrackArray):
        self._tracks = msg
        now = time.monotonic()
        self._tracks_recv_mono = now
        try:
            if len(list(msg.tracks)) > 0:
                self._last_nonempty_tracks_mono = now
        except Exception:
            pass

    def _cb_preds(self, msg: ObstaclePredictionArray):
        self._preds = msg
        now = time.monotonic()
        self._preds_recv_mono = now
        try:
            if len(list(msg.predictions)) > 0:
                self._last_nonempty_preds_mono = now
        except Exception:
            pass

    def _cb_costmap(self, msg):
        # msg is nav2_msgs/Costmap if available
        self._costmap = msg
        self._costmap_recv_time = time.monotonic()

    def _cb_region_state(self, msg: String):
        try:
            self._region_state_cache = json.loads(msg.data) if msg.data else {}
        except Exception:
            self._region_state_cache = {}

    def _has_live_obstacle_evidence(self, max_age_sec: float = 0.8) -> bool:
        now = time.monotonic()
        try:
            if self._tracks is not None and (now - self._tracks_recv_mono) <= max_age_sec and len(list(self._tracks.tracks)) > 0:
                return True
        except Exception:
            pass
        try:
            if self._preds is not None and (now - self._preds_recv_mono) <= max_age_sec and len(list(self._preds.predictions)) > 0:
                return True
        except Exception:
            pass
        return False

    def _no_live_obstacle_evidence_for(self, sec: float) -> bool:
        if self._has_live_obstacle_evidence(max_age_sec=max(0.3, min(1.5, sec))):
            return False
        last = max(float(self._last_nonempty_tracks_mono), float(self._last_nonempty_preds_mono))
        if last <= 0.0:
            return False
        return (time.monotonic() - last) >= sec

    def _clear_costmaps_best_effort(self, reason: str) -> bool:
        if ClearEntireCostmap is None:
            return False
        timeout = float(self.get_parameter('clear_service_timeout_sec').value)
        ok = False
        for name, cli in [
            (str(self.get_parameter('clear_local_service').value), self._clear_local_cli),
            (str(self.get_parameter('clear_global_service').value), self._clear_global_cli),
        ]:
            if cli is None or not name:
                continue
            try:
                if not cli.wait_for_service(timeout_sec=timeout):
                    continue
                fut = cli.call_async(ClearEntireCostmap.Request())
                if self._wait_future_with_event(fut, timeout_sec=timeout, label=f'clear:{name}') and fut.done():
                    try:
                        fut.result()
                    except Exception:
                        continue
                    ok = True
            except Exception:
                continue
        if ok:
            self.get_logger().warn(f"[supervisor] cleared costmaps ({reason})")
        return ok

    def _maybe_stale_shadow_cleanup(self, reason: str) -> bool:
        if not bool(self.get_parameter('stale_shadow_cleanup_enable').value):
            return False
        no_live_sec = float(self.get_parameter('stale_shadow_no_live_sec').value)
        if not self._no_live_obstacle_evidence_for(no_live_sec):
            return False
        now = time.monotonic()
        cooldown = float(self.get_parameter('stale_shadow_clear_cooldown_sec').value)
        if (now - self._last_shadow_clear_mono) < cooldown:
            return False
        self._last_shadow_clear_mono = now
        ok = self._clear_costmaps_best_effort(f"stale_shadow:{reason}")
        self._publish_recovery_event('stale_shadow_clear', reason=str(reason), cleared=bool(ok))
        return ok

    def _init_action_watchdog(self):
        if not bool(self.get_parameter('action_watchdog_enable').value):
            return
        now = time.monotonic()
        self._action_wd_started_t = now
        self._action_wd_last_progress_t = now
        self._action_wd_last_xy = None
        try:
            frame = self._active_path.header.frame_id if self._active_path is not None else 'map'
            stamp = self.get_clock().now().to_msg()
            self._action_wd_last_xy = self._get_robot_xy(frame, stamp)
        except Exception:
            self._action_wd_last_xy = None

    def _tick_action_watchdog(self):
        if self._follow_kind != 'SEGMENT' or self._mode_state != 'FOLLOWING':
            return
        if not bool(self.get_parameter('action_watchdog_enable').value):
            return
        if self._result_future is None or self._goal_handle is None:
            return
        now = time.monotonic()
        if self._action_wd_started_t <= 0.0:
            self._init_action_watchdog()
            return
        grace = float(self.get_parameter('action_watchdog_grace_sec').value)
        if now - self._action_wd_started_t < grace:
            return
        frame = self._active_path.header.frame_id if self._active_path is not None else 'map'
        stamp = self.get_clock().now().to_msg()
        xy = self._get_robot_xy(frame, stamp)
        if xy is None:
            return
        if self._action_wd_last_xy is None:
            self._action_wd_last_xy = xy
            return
        dist = math.hypot(float(xy[0]) - float(self._action_wd_last_xy[0]), float(xy[1]) - float(self._action_wd_last_xy[1]))
        min_prog = float(self.get_parameter('action_watchdog_min_progress_m').value)
        if dist >= min_prog:
            self._action_wd_last_xy = xy
            self._action_wd_last_progress_t = now
            return
        hard_cap = float(self.get_parameter('action_watchdog_segment_max_sec').value)
        timeout = float(self.get_parameter('action_watchdog_timeout_sec').value)
        stalled = (now - self._action_wd_last_progress_t) >= timeout
        expired = (now - self._action_wd_started_t) >= hard_cap
        if not stalled and not expired:
            return
        why = 'segment_max_age' if expired else 'segment_no_progress'
        self.get_logger().warn(f"[supervisor] action watchdog fired: {why} -> cancel + escalate")
        self._publish_recovery_event('stall_watchdog', reason=str(why), seg_id=self._segments[int(self._active_seg_idx)].sid if self._active_seg_idx is not None else '')
        self._maybe_stale_shadow_cleanup(why)
        self._cancel_follow()
        self._stop_cmd_vel()
        self._send_future = None
        self._result_future = None
        self._goal_handle = None
        self._cancel_future = None
        self._action_wd_started_t = 0.0
        self._action_wd_last_progress_t = 0.0
        self._action_wd_last_xy = None
        self._handle_segment_follow_failure(-101)

    def _publish_recovery_event(self, event_type: str, **fields):
        rec: Dict[str, Any] = {'type': str(event_type), 'time': time.time()}
        rec.update(fields)
        if 'cluster_id' not in rec and self._active_seg_idx is not None:
            try:
                rec['cluster_id'] = int(self._segments[int(self._active_seg_idx)].cluster_id)
            except Exception:
                pass
        if 'seg_id' not in rec and self._active_seg_idx is not None:
            try:
                rec['seg_id'] = str(self._segments[int(self._active_seg_idx)].sid)
            except Exception:
                pass
        msg = String()
        try:
            msg.data = json.dumps(rec, ensure_ascii=False)
            self._pub_recovery_event.publish(msg)
        except Exception:
            pass

    def _active_cluster_id(self) -> Optional[int]:
        if self._active_seg_idx is None:
            return None
        try:
            return int(self._segments[int(self._active_seg_idx)].cluster_id)
        except Exception:
            return None

    def _robot_pose_map(self) -> Optional[Dict[str, float]]:
        stamp = self.get_clock().now().to_msg()
        base = str(self.get_parameter('robot_base_frame').value)
        tf2d = self._lookup_tf2d('map', base, stamp)
        if tf2d is None:
            return None
        return {'x': float(tf2d[0]), 'y': float(tf2d[1]), 'yaw': float(tf2d[2])}

    def _is_original_plan_segment(self, seg_idx: Optional[int]) -> bool:
        try:
            i = int(seg_idx)
        except Exception:
            return False
        return 0 <= i < int(self._base_plan_segment_count)

    def _mark_main_pass_frontier(self, seg_idx: Optional[int]):
        if not self._is_original_plan_segment(seg_idx):
            return
        try:
            i = int(seg_idx)
        except Exception:
            return
        if i > int(self._main_pass_frontier_idx):
            self._main_pass_frontier_idx = i

    def _segment_min_robot_distance_m(self, seg_idx: int) -> float:
        robot = self._robot_pose_map()
        if robot is None:
            return -1.0
        try:
            seg = self._segments[int(seg_idx)]
        except Exception:
            return -1.0
        rx = float(robot['x'])
        ry = float(robot['y'])
        best = float('inf')
        try:
            for ps in seg.poses:
                px = float(ps.pose.position.x)
                py = float(ps.pose.position.y)
                best = min(best, math.hypot(px - rx, py - ry))
        except Exception:
            return -1.0
        return float(best) if math.isfinite(best) else -1.0

    def _within_forward_jump_limits(self, seg_idx: int, origin_seg_idx: Optional[int]) -> bool:
        if not bool(self.get_parameter('jump_limit_enable').value):
            return True
        try:
            target = int(seg_idx)
        except Exception:
            return False
        if origin_seg_idx is not None:
            try:
                origin = int(origin_seg_idx)
            except Exception:
                origin = None
            else:
                if self._is_original_plan_segment(target) and self._is_original_plan_segment(origin):
                    if target < origin:
                        return False
                    max_delta = max(0, int(self.get_parameter('jump_limit_max_segment_delta').value))
                    if (target - origin) > max_delta:
                        return False
        max_linear_m = float(self.get_parameter('jump_limit_max_linear_m').value)
        if max_linear_m > 0.0:
            dist = self._segment_min_robot_distance_m(target)
            if dist >= 0.0 and dist > max_linear_m:
                return False
        return True

    def _should_move_deferred_to_cleanup(self, seg_idx: int) -> bool:
        if not bool(self.get_parameter('main_pass_cleanup_far_deferred').value):
            return False
        if not self._is_original_plan_segment(seg_idx):
            return False
        frontier = int(self._main_pass_frontier_idx)
        idx = int(seg_idx)
        if frontier < 0 or idx >= frontier:
            return False
        max_back_seg = max(0, int(self.get_parameter('main_pass_revisit_window_segments').value))
        max_back_m = max(0.0, float(self.get_parameter('main_pass_revisit_window_m').value))
        too_many_segments = (frontier - idx) > max_back_seg
        too_far_m = False
        if max_back_m > 0.0:
            dist = self._segment_min_robot_distance_m(idx)
            too_far_m = dist >= 0.0 and dist > max_back_m
        return bool(too_many_segments or too_far_m)

    def _move_segment_to_cleanup_only(self, seg_idx: int, reason: str):
        seg_idx = int(seg_idx)
        if seg_idx in self._retired_segments:
            self._queue = [int(q) for q in self._queue if int(q) != seg_idx]
            return
        self._retired_segments.add(seg_idx)
        self._queue = [int(q) for q in self._queue if int(q) != seg_idx]
        seg = self._segments[seg_idx]
        self._append_cleanup_log({
            'type': 'cleanup_only_segment',
            'reason': str(reason),
            'seg_id': seg.sid,
            'cluster_id': int(seg.cluster_id),
            'frontier_idx': int(self._main_pass_frontier_idx),
            'time': time.time(),
        })
        self._publish_recovery_event(
            'cleanup_only_segment',
            cluster_id=self._seg_cluster(seg_idx),
            seg_id=seg.sid,
            reason=str(reason),
        )
        self.get_logger().info(f"[supervisor] main-pass cleanup-only {seg.sid} ({reason})")

    def _retire_scope_segments_after_d2(self, seg_indices: Set[int], reason: str):
        for seg_idx in sorted(int(i) for i in set(seg_indices)):
            if seg_idx < 0 or seg_idx >= len(self._segments):
                continue
            if seg_idx in self._retired_segments:
                self._queue = [int(q) for q in self._queue if int(q) != seg_idx]
                continue
            seg = self._segments[seg_idx]
            self._retired_segments.add(seg_idx)
            self._queue = [int(q) for q in self._queue if int(q) != seg_idx]
            self._append_cleanup_log({
                'type': 'superseded_segment',
                'reason': str(reason),
                'seg_id': seg.sid,
                'cluster_id': int(seg.cluster_id),
                'time': time.time(),
            })
            self._publish_recovery_event(
                'superseded_segment',
                cluster_id=self._seg_cluster(seg_idx),
                seg_id=seg.sid,
                reason=str(reason),
            )
            self.get_logger().info(f"[supervisor] superseded old seg {seg.sid} ({reason})")

    def _tracks_in_map(self) -> List[Dict[str, float]]:
        out: List[Dict[str, float]] = []
        if self._tracks is None or not self._tracks.tracks:
            return out
        tf = self._lookup_tf2d('map', str(self._tracks.header.frame_id), self._tracks.header.stamp)
        if tf is None:
            return out
        radius_cap = max(0.10, float(self.get_parameter('d2_blocked_zone_radius_cap_m').value))
        for tr in self._tracks.tracks:
            try:
                xw, yw = _tf2d_apply(float(tf[0]), float(tf[1]), float(tf[2]), float(tr.pose.position.x), float(tr.pose.position.y))
                # Export a capped raw track radius here. The runtime planner already applies its own
                # blocked-zone pad, so padding again here would double-inflate obstacles and
                # starve D2 on micro maps. Large tracker radii are clipped to keep replanning viable.
                raw_r = float(tr.radius)
                eff_r = min(raw_r, radius_cap)
                out.append({
                    'id': str(tr.id),
                    'x': float(xw),
                    'y': float(yw),
                    'r': float(eff_r),
                    'radius': float(raw_r),
                })
            except Exception:
                continue
        return out

    def _segment_snapshot(self, seg_idx: int, entry_pose_idx: int = 0, queue_rank: int = 0) -> Optional[Dict[str, Any]]:
        if seg_idx < 0 or seg_idx >= len(self._segments):
            return None
        seg = self._segments[int(seg_idx)]
        poses = seg.poses[max(0, int(entry_pose_idx)):]
        if not poses:
            return None
        pts: List[Dict[str, Any]] = []
        for i, ps in enumerate(poses):
            yaw = 0.0
            if i + 1 < len(poses):
                p = poses[i].pose.position
                q = poses[i + 1].pose.position
                yaw = math.atan2(float(q.y) - float(p.y), float(q.x) - float(p.x))
            else:
                try:
                    q = poses[i].pose.orientation
                    yaw = _yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
                except Exception:
                    yaw = 0.0
            pts.append({'x': float(ps.pose.position.x), 'y': float(ps.pose.position.y), 'yaw': float(yaw)})
        path_len = 0.0
        for a, b in zip(pts[:-1], pts[1:]):
            path_len += math.hypot(float(b['x']) - float(a['x']), float(b['y']) - float(a['y']))
        return {
            'seg_idx': int(seg_idx),
            'seg_id': str(seg.sid),
            'cluster_id': int(seg.cluster_id),
            'kind': str(seg.kind),
            'queue_rank': int(queue_rank),
            'entry_pose_idx': int(entry_pose_idx),
            'path_length_m': float(path_len),
            'start_pose': dict(pts[0]),
            'end_pose': dict(pts[-1]),
            'poses': pts,
        }

    def _reorder_queue_prefix(self, order: List[int]):
        cleaned: List[int] = []
        seen = set()
        for idx in order:
            idx = int(idx)
            if idx in seen:
                continue
            seen.add(idx)
            cleaned.append(idx)
        rest = [int(q) for q in self._queue if int(q) not in seen]
        front = [int(q) for q in cleaned if int(q) in self._queue]
        self._queue = front + rest

    def _build_d_snapshot(self) -> Optional[Dict[str, Any]]:
        if self._active_seg_idx is None:
            return None
        cid = self._active_cluster_id()
        if cid is None:
            return None
        robot_pose = self._robot_pose_map()
        if robot_pose is None:
            return None
        region = {}
        try:
            region = dict(self._region_state_cache.get('clusters', {}).get(str(cid), {}))
        except Exception:
            region = {}
        origin_i = int(self._current_progress_index())
        rem: List[Dict[str, Any]] = []
        active_snap = self._segment_snapshot(int(self._active_seg_idx), entry_pose_idx=origin_i, queue_rank=0)
        if active_snap is not None:
            rem.append(active_snap)
        rank = 1
        for qidx in self._queue:
            if self._seg_cluster(int(qidx)) != cid:
                continue
            snap = self._segment_snapshot(int(qidx), entry_pose_idx=0, queue_rank=rank)
            if snap is not None:
                rem.append(snap)
                rank += 1
        if not rem:
            return None
        pts = []
        for seg in rem:
            for p in seg.get('poses', []):
                pts.append((float(p['x']), float(p['y'])))
        if pts:
            pad = float(self.get_parameter('d_plan_bbox_pad_m').value)
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            bbox = [min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad]
        else:
            bbox = list(region.get('bbox', [0.0, 0.0, 0.0, 0.0]))
        return {
            'cluster_id': int(cid),
            'robot_pose': robot_pose,
            'active_seg_idx': int(self._active_seg_idx),
            'remaining_segments': rem,
            'region_summary': region,
            'blocked_zones': self._tracks_in_map(),
            'cluster_bbox': bbox,
            'coverage_width': 0.45,
            'candidate_limit': int(self.get_parameter('d2_candidate_limit').value),
        }


    def _build_d2_test_forced_snapshot(self) -> Optional[Dict[str, Any]]:
        path = str(self.get_parameter('d2_test_force_snapshot_path').value or '').strip()
        if not path:
            return self._build_d_snapshot()
        p = os.path.expanduser(path)
        if not os.path.isfile(p):
            self.get_logger().warn(f"[supervisor] D2 TEST FORCE snapshot file not found: {p}")
            return None
        raw: Optional[Dict[str, Any]] = None
        if self._d2_test_force_snapshot_cache is not None and self._d2_test_force_snapshot_cache_path == p:
            raw = self._d2_test_force_snapshot_cache
        else:
            try:
                with open(p, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                if isinstance(data, dict) and isinstance(data.get('snapshot'), dict):
                    data = data.get('snapshot')
                if not isinstance(data, dict):
                    raise ValueError('snapshot root is not a JSON object')
                self._d2_test_force_snapshot_cache = data
                self._d2_test_force_snapshot_cache_path = p
                raw = data
            except Exception as e:
                self.get_logger().warn(f"[supervisor] D2 TEST FORCE failed to load snapshot {p}: {e}")
                return None
        snap: Dict[str, Any] = copy.deepcopy(raw or {})
        live_pose = self._robot_pose_map()
        if bool(self.get_parameter('d2_test_force_use_live_robot_pose').value) and live_pose is not None:
            snap['robot_pose'] = dict(live_pose)
        if bool(self.get_parameter('d2_test_force_use_live_cluster_id').value):
            cid = self._active_cluster_id()
            if cid is not None:
                snap['cluster_id'] = int(cid)
        if self._active_seg_idx is not None:
            snap['active_seg_idx'] = int(self._active_seg_idx)
        snap['candidate_limit'] = int(self.get_parameter('d2_candidate_limit').value)
        return snap

    def _maybe_force_d2_test_once(self) -> bool:
        if not bool(self.get_parameter('d2_test_force_once_enable').value):
            return False
        if self._d2_test_force_fired:
            return False
        if self._active_seg_idx is None:
            return False
        seg_idx = int(self._active_seg_idx)
        seg_min = int(self.get_parameter('d2_test_force_active_seg_min').value)
        seg_max = int(self.get_parameter('d2_test_force_active_seg_max').value)
        if seg_min >= 0 and seg_idx < seg_min:
            return False
        if seg_max >= 0 and seg_idx > seg_max:
            return False

        snapshot = self._build_d2_test_forced_snapshot()
        if snapshot is None:
            self.get_logger().warn('[supervisor] D2 TEST FORCE aborted: snapshot unavailable')
            return False

        guard_reason = self._d2_guard_reason(snapshot, forced=True)
        if guard_reason:
            self._log_d2_guard_skip(guard_reason, forced=True)
            return False
        self._log_d2_guard_skip('', forced=True)
        self._d2_test_force_fired = True

        region = dict(snapshot.get('region_summary', {}) or {})
        self.get_logger().warn(
            f"[supervisor] D2 TEST FORCE active_seg={seg_idx} cluster={snapshot.get('cluster_id')} "
            f"score={float(region.get('score_d2', 0.0)):.2f} hard_d2={bool(region.get('hard_d2', False))}")
        self._publish_recovery_event(
            'd2_test_forced_request',
            cluster_id=snapshot.get('cluster_id'),
            active_seg=seg_idx,
            score=float(region.get('score_d2', 0.0)),
            hard_d2=bool(region.get('hard_d2', False)),
        )

        resp = self._call_d2_service(snapshot)
        if resp is not None and bool(getattr(resp, 'replace_plan', False)):
            self.get_logger().warn(
                f"[supervisor] D2 replace accepted backend={getattr(resp, 'selected_backend', 'unknown')} "
                f"improvement={float(getattr(resp, 'improvement', 0.0)):.2f} (forced-test)")
            if self._apply_d2_plan_response(resp, snapshot):
                return True
            self.get_logger().warn('[supervisor] D2 TEST FORCE apply failed after replace=True')
            return False

        self.get_logger().warn(
            f"[supervisor] D2 TEST FORCE no replace; rpc_status={self._last_d2_rpc_status} "
            f"latency={self._last_d2_rpc_elapsed_sec:.2f}s message={self._last_d2_rpc_message}")
        return False

    def _region_prefers_structural_d(self) -> bool:
        if not bool(self.get_parameter('use_d12_advisors').value):
            return False
        snapshot = self._build_d_snapshot()
        if snapshot is None:
            return False
        region = dict(snapshot.get('region_summary', {}))
        if bool(region.get('hard_d1', False) or region.get('hard_d2', False)):
            return True
        try:
            score_d1 = float(region.get('score_d1', 0.0))
            score_d2 = float(region.get('score_d2', 0.0))
        except Exception:
            return False
        return bool(
            score_d1 >= float(self.get_parameter('d1_score_threshold').value)
            or score_d2 >= float(self.get_parameter('d2_score_threshold').value)
        )

    def _maybe_preempt_c_with_d(self, trigger_reason: str) -> bool:
        if not bool(self.get_parameter('c_to_d_preempt_enable').value):
            return False
        if not self._is_c_reason(self._detour_reason):
            return False
        if self._active_seg_idx is None:
            return False
        snapshot = self._build_d_snapshot()
        if snapshot is None:
            return False
        region = dict(snapshot.get('region_summary', {}))
        if not (self._region_prefers_structural_d() or bool(region.get('hard_d1', False)) or bool(region.get('hard_d2', False))):
            return False
        path_rej_ge = int(self.get_parameter('c_to_d_preempt_path_rejected_ge').value)
        refresh_ge = int(self.get_parameter('c_to_d_preempt_refresh_ge').value)
        ok = False
        if str(trigger_reason) == 'compute_path_rejected':
            ok = int(region.get('n_path_rejected', 0)) >= path_rej_ge
        elif str(trigger_reason).startswith('refresh_'):
            ok = int(self._detour_refresh_count) >= refresh_ge
        if not ok:
            return False

        self.get_logger().warn(f"[supervisor] preempt C detour -> D due to {trigger_reason}")
        self._publish_recovery_event('c_to_d_preempt', cluster_id=self._active_cluster_id(), reason=str(trigger_reason))
        self._cancel_follow()
        self._stop_cmd_vel()
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._cancel_future = None
        self._reset_detour_state()
        self._mode_state = 'FOLLOWING'
        self._follow_kind = 'SEGMENT'
        allow_cluster = bool(self.get_parameter('c_to_d_preempt_allow_cluster_jump').value)
        force_cluster = bool(self.get_parameter('c_to_d_preempt_force_cluster_jump').value)
        self._start_mode_d(force_cluster_jump=force_cluster, allow_cluster_jump=allow_cluster, bypass_cooldown=True)
        return True

    def _wait_future_with_event(self, fut, timeout_sec: float, label: str) -> bool:
        """Wait for an rclpy future without recursively spinning the current callback.

        The supervisor issues D1/D2 service calls from inside the periodic timer callback. Using
        rclpy.spin_until_future_complete(self, fut, ...) from there can deadlock or time out when
        the client response callback cannot be scheduled. Instead, attach a done-callback, wait on
        a threading.Event, and let the node's MultiThreadedExecutor drive the client response on a
        separate callback group.
        """
        timeout_sec = max(0.0, float(timeout_sec))
        done_evt = threading.Event()

        def _mark_done(_future):
            done_evt.set()

        try:
            fut.add_done_callback(_mark_done)
        except Exception:
            pass

        if fut.done():
            return True

        deadline = time.monotonic() + timeout_sec
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return fut.done()
            if done_evt.wait(timeout=min(0.05, remaining)):
                return True
            if fut.done():
                return True

    def _call_d1_service(self, snapshot: Dict[str, Any]):
        if not self._d1_client.wait_for_service(timeout_sec=0.15):
            return None
        req = D1Evaluate.Request()
        req.snapshot_json = json.dumps(snapshot, ensure_ascii=False)
        fut = self._d1_client.call_async(req)
        timeout_sec = float(self.get_parameter('d12_rpc_timeout_sec').value)
        if self._wait_future_with_event(fut, timeout_sec=timeout_sec, label='D1'):
            try:
                return fut.result()
            except Exception:
                return None
        try:
            fut.cancel()
        except Exception:
            pass
        return None

    def _call_d2_service(self, snapshot: Dict[str, Any]):
        self._last_d2_rpc_status = 'idle'
        self._last_d2_rpc_message = ''
        self._last_d2_rpc_elapsed_sec = 0.0
        if not self._d2_client.wait_for_service(timeout_sec=0.15):
            self._last_d2_rpc_status = 'service_unavailable'
            self._last_d2_rpc_message = 'service wait timeout'
            return None
        req = D2Replan.Request()
        req.snapshot_json = json.dumps(snapshot, ensure_ascii=False)
        fut = self._d2_client.call_async(req)
        timeout_sec = float(self.get_parameter('d12_rpc_timeout_sec').value)
        t0 = time.monotonic()
        if not self._wait_future_with_event(fut, timeout_sec=timeout_sec, label='D2'):
            self._last_d2_rpc_elapsed_sec = max(0.0, time.monotonic() - t0)
            try:
                fut.cancel()
            except Exception:
                pass
            self._last_d2_rpc_status = 'timeout'
            self._last_d2_rpc_message = f'timeout>{timeout_sec:.2f}s'
            self.get_logger().warn(
                f"[supervisor] D2 rpc timeout after {self._last_d2_rpc_elapsed_sec:.2f}s (limit={timeout_sec:.2f}s)")
            return None

        self._last_d2_rpc_elapsed_sec = max(0.0, time.monotonic() - t0)
        if fut.done():
            try:
                resp = fut.result()
            except Exception as e:
                self._last_d2_rpc_status = 'exception'
                self._last_d2_rpc_message = str(e)
                self.get_logger().warn(
                    f"[supervisor] D2 rpc exception after {self._last_d2_rpc_elapsed_sec:.2f}s: {e}")
                return None
            self._last_d2_rpc_status = 'response'
            self._last_d2_rpc_message = str(getattr(resp, 'message', ''))
            try:
                dbg = json.loads(getattr(resp, 'ranking_json', '') or '{}')
                self.get_logger().warn(
                    '[supervisor] D2 result '
                    f"backend={getattr(resp, 'selected_backend', 'unknown')} "
                    f"replace={bool(getattr(resp, 'replace_plan', False))} "
                    f"improvement={float(dbg.get('improvement', 0.0)):.2f} "
                    f"blocked_relief={float(dbg.get('blocked_relief_m', 0.0)):.2f} "
                    f"current_exposure={float(dbg.get('current_blocked_exposure_m', 0.0)):.2f} "
                    f"best_exposure={float(dbg.get('best_blocked_exposure_m', 0.0)):.2f} "
                    f"scope_n={len(list(dbg.get('scope_seg_indices', []) or []))} "
                    f"latency={self._last_d2_rpc_elapsed_sec:.2f}s "
                    f"message={str(getattr(resp, 'message', ''))}")
            except Exception:
                self.get_logger().warn(
                    '[supervisor] D2 result '
                    f"backend={getattr(resp, 'selected_backend', 'unknown')} "
                    f"replace={bool(getattr(resp, 'replace_plan', False))} "
                    f"latency={self._last_d2_rpc_elapsed_sec:.2f}s "
                    f"message={str(getattr(resp, 'message', ''))}")
            return resp

        # Extremely defensive: the wait helper reported completion but the future still does not
        # look done from here. Treat this as a timeout-like failure to avoid falsely accepting an
        # incomplete plan.
        try:
            fut.cancel()
        except Exception:
            pass
        self._last_d2_rpc_status = 'timeout'
        self._last_d2_rpc_message = f'timeout>{timeout_sec:.2f}s'
        self.get_logger().warn(
            f"[supervisor] D2 rpc timeout after {self._last_d2_rpc_elapsed_sec:.2f}s (future not done)")
        return None

    def _add_runtime_segment_from_json(self, s: Dict[str, Any], default_cluster_id: int) -> Optional[int]:
        poses_in = list(s.get('poses', []))
        if len(poses_in) < 2:
            return None
        poses: List[PoseStamped] = []
        frame_id = str(s.get('frame_id', 'map'))
        for p in poses_in:
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(p['x'])
            ps.pose.position.y = float(p['y'])
            ps.pose.position.z = 0.0
            ps.pose.orientation.x = float(p.get('qx', 0.0))
            ps.pose.orientation.y = float(p.get('qy', 0.0))
            ps.pose.orientation.z = float(p.get('qz', 0.0))
            ps.pose.orientation.w = float(p.get('qw', 1.0))
            poses.append(ps)
        idx = len(self._segments)
        sid = str(s.get('id', f'd2_seg_{idx:04d}'))
        seg = _Seg(
            sid=sid,
            cluster_id=int(s.get('cluster_id', default_cluster_id)),
            frame_id=frame_id,
            kind=str(s.get('kind', 'row')),
            poses=poses,
        )
        self._segments.append(seg)
        self._defer_count[idx] = 0
        self._replan_count[idx] = 0
        self._c_slice_count[idx] = 0
        self._d_recovery_count[idx] = 0
        self._cooldown_until[idx] = 0.0
        self._fail_count[idx] = 0
        self._c_last_trigger_mono[idx] = -1e9
        self._d_last_trigger_mono[idx] = -1e9
        self._abort_last_replan_mono[idx] = -1e9
        return idx

    def _apply_d1_route_response(self, resp, snapshot: Dict[str, Any]) -> bool:
        try:
            entries = json.loads(resp.ordered_segment_entries_json)
        except Exception:
            return False
        if not entries:
            return False
        self._reorder_queue_prefix([int(e.get('seg_idx', -1)) for e in entries])
        origin_i = int(self._current_progress_index())
        cands: List[_DetourCandidate] = []
        origin_seg_idx = int(self._active_seg_idx) if self._active_seg_idx is not None else None
        for e in entries:
            seg_idx = int(e.get('seg_idx', -1))
            pose_idx = int(e.get('pose_idx', 0))
            if seg_idx < 0 or seg_idx >= len(self._segments):
                continue
            if not self._within_forward_jump_limits(seg_idx, origin_seg_idx):
                continue
            seg = self._segments[seg_idx]
            pose_idx = max(0, min(len(seg.poses) - 1, pose_idx))
            cands.append(_DetourCandidate(
                target_pose=seg.poses[pose_idx],
                resume_seg_idx=seg_idx,
                resume_pose_idx=pose_idx,
                skipped=[],
                note='d1_route'))
        if not cands:
            return False
        self._publish_recovery_event('d1_applied', cluster_id=snapshot.get('cluster_id'), improvement=float(getattr(resp, 'improvement', 0.0)))
        self._start_detour('D1', cands, origin_pose_idx=origin_i)
        self._send_compute_for_candidate(cands[0])
        return True

    def _apply_d2_plan_response(self, resp, snapshot: Dict[str, Any]) -> bool:
        try:
            plan = json.loads(resp.selected_plan_json)
        except Exception:
            return False
        cid = int(snapshot.get('cluster_id', self._active_cluster_id() or -1))
        meta = dict(plan.get('meta', {}) or {})
        scope_seg_indices = [int(x) for x in list(meta.get('scope_seg_indices', []) or []) if int(x) >= 0]
        new_indices: List[int] = []
        for s in list(plan.get('segments', [])):
            idx = self._add_runtime_segment_from_json(s, default_cluster_id=cid)
            if idx is not None:
                new_indices.append(int(idx))
        if not new_indices:
            return False
        scope_set: Set[int] = set(int(x) for x in scope_seg_indices)
        if scope_set:
            self._pending_d2_scope_supersede = set(int(x) for x in scope_set)
        else:
            self._pending_d2_scope_supersede = set(int(q) for q in self._queue if self._seg_cluster(int(q)) == cid)
        # Keep the old scope segments in the queue until the first D2 path actually wins.
        # New D2 segments are simply prepended so they are tried first; old scope segments are
        # retired only in _on_detour_succeeded once the D2 replacement is truly accepted.
        self._queue = list(new_indices) + [int(q) for q in self._queue if int(q) not in set(new_indices)]
        self._mission_complete_announced = False
        self._publish_recovery_event('d2_replace', cluster_id=cid, new_segment_count=len(new_indices), scope_segment_count=len(scope_seg_indices), backend=str(getattr(resp, 'selected_backend', 'unknown')), improvement=float(getattr(resp, 'improvement', 0.0)))
        origin_i = int(self._current_progress_index())
        cands: List[_DetourCandidate] = []
        for seg_idx in new_indices:
            seg = self._segments[int(seg_idx)]
            cands.append(_DetourCandidate(
                target_pose=seg.poses[0],
                resume_seg_idx=int(seg_idx),
                resume_pose_idx=0,
                skipped=[],
                note='d2_plan'))
        self._start_detour('D2', cands, origin_pose_idx=origin_i)
        self._send_compute_for_candidate(cands[0])
        self._d2_accept_count += 1
        if bool(self.get_parameter('d2_disable_after_first_accept').value) and (not self._d2_one_shot_consumed):
            self._d2_one_shot_consumed = True
            self.get_logger().warn(
                f"[supervisor] D2 one-shot latch consumed after accept #{self._d2_accept_count} backend={getattr(resp, 'selected_backend', 'unknown')}")
        return True

    def _maybe_use_d12_advisors(self, force_cluster_jump: bool = False, allow_cluster_jump: bool = True) -> bool:
        if not bool(self.get_parameter('use_d12_advisors').value):
            return False
        if self._maybe_force_d2_test_once():
            return True
        snapshot = self._build_d_snapshot()
        if snapshot is None:
            return False
        region = dict(snapshot.get('region_summary', {}))
        score_d1 = float(region.get('score_d1', 0.0))
        score_d2 = float(region.get('score_d2', 0.0))
        hard_d1 = bool(region.get('hard_d1', False))
        hard_d2 = bool(region.get('hard_d2', False))
        d1_thr = float(self.get_parameter('d1_score_threshold').value)
        d2_thr = float(self.get_parameter('d2_score_threshold').value)
        d2_guard_reason = self._d2_guard_reason(snapshot, forced=False)
        d2_allowed_now = (d2_guard_reason == '')
        if d2_allowed_now:
            self._log_d2_guard_skip('', forced=False)

        d2_triggered = bool(hard_d2 or score_d2 >= d2_thr)
        if bool(self.get_parameter('d2_enable').value) and d2_triggered and d2_allowed_now:
            self.get_logger().warn(
                f"[supervisor] D2 check cluster={snapshot.get('cluster_id')} score={score_d2:.2f} hard_d2={hard_d2}")
            self._publish_recovery_event('d2_request', cluster_id=snapshot.get('cluster_id'), score=score_d2)
            resp = self._call_d2_service(snapshot)
            if resp is not None and bool(getattr(resp, 'replace_plan', False)):
                self.get_logger().warn(
                    f"[supervisor] D2 replace accepted backend={getattr(resp, 'selected_backend', 'unknown')} improvement={float(getattr(resp, 'improvement', 0.0)):.2f}")
                if self._apply_d2_plan_response(resp, snapshot):
                    return True
            if resp is None:
                self.get_logger().warn(
                    f"[supervisor] D2 {self._last_d2_rpc_status} ({self._last_d2_rpc_message}) after {self._last_d2_rpc_elapsed_sec:.2f}s -> fall back to D1/local D")
            else:
                self.get_logger().warn(
                    f"[supervisor] D2 result replace=False backend={getattr(resp, 'selected_backend', 'unknown')} message={str(getattr(resp, 'message', ''))} -> fall back to D1/local D")
            self._publish_recovery_event('d2_fail', cluster_id=snapshot.get('cluster_id'), score=score_d2, status=str(self._last_d2_rpc_status), detail=str(self._last_d2_rpc_message))
        elif bool(self.get_parameter('d2_enable').value) and d2_triggered and (not d2_allowed_now):
            self._log_d2_guard_skip(d2_guard_reason, forced=False)
            self._publish_recovery_event('d2_guard_skip', cluster_id=snapshot.get('cluster_id'), score=score_d2, detail=str(d2_guard_reason))

        run_d1 = bool(force_cluster_jump or hard_d1 or score_d1 >= d1_thr)
        if run_d1:
            self._publish_recovery_event('d1_request', cluster_id=snapshot.get('cluster_id'), score=score_d1)
            resp = self._call_d1_service(snapshot)
            if resp is not None and bool(getattr(resp, 'replace_route', False)):
                if self._apply_d1_route_response(resp, snapshot):
                    return True
            self._publish_recovery_event('d1_fail', cluster_id=snapshot.get('cluster_id'), score=score_d1)
        return False

    def _load_plan(self, plan_path: str) -> List[_Seg]:
        with open(plan_path, 'r', encoding='utf-8') as f:
            plan = json.load(f)

        segs = []
        for s in plan.get('segments', []):
            poses = []
            for p in s.get('poses', []):
                ps = PoseStamped()
                ps.header.frame_id = p.get('frame_id', s.get('frame_id', plan.get('frame_id', 'map')))
                ps.pose.position.x = float(p['x'])
                ps.pose.position.y = float(p['y'])
                ps.pose.position.z = 0.0
                ps.pose.orientation.x = float(p.get('qx', 0.0))
                ps.pose.orientation.y = float(p.get('qy', 0.0))
                ps.pose.orientation.z = float(p.get('qz', 0.0))
                ps.pose.orientation.w = float(p.get('qw', 1.0))
                poses.append(ps)
            segs.append(_Seg(
                sid=str(s.get('id', 'seg')),
                cluster_id=int(s.get('cluster_id', -1)),
                frame_id=str(s.get('frame_id', plan.get('frame_id', 'map'))),
                kind=str(s.get('kind', s.get('meta', {}).get('type', 'segment'))),
                poses=poses
            ))
        if not segs:
            raise RuntimeError("Plan has no segments")
        return segs

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

    def _get_robot_xy(self, frame_id: str, stamp) -> Optional[Tuple[float, float]]:
        base = self.get_parameter('robot_base_frame').value
        tf2d = self._lookup_tf2d(frame_id, base, stamp)
        if tf2d is None:
            return None
        tx, ty, _ = tf2d
        return tx, ty

    def _path_from_segment(self, seg: _Seg, stamp) -> Path:
        path = Path()
        path.header.frame_id = seg.frame_id
        path.header.stamp = stamp  # can be 0 or now
        path.poses = []
        for ps in seg.poses:
            q = PoseStamped()
            q.header.frame_id = seg.frame_id
            q.header.stamp.sec = 0
            q.header.stamp.nanosec = 0
            q.pose = ps.pose
            path.poses.append(q)
        return path

    def _trim_path_to_robot(self, path: Path, stamp) -> Path:
        if len(path.poses) < 2:
            return path
        r = self._get_robot_xy(path.header.frame_id, stamp)
        if r is None:
            return path
        rx, ry = r

        best_i = 0
        best_d = float("inf")
        for i, ps in enumerate(path.poses):
            x = ps.pose.position.x
            y = ps.pose.position.y
            d = math.hypot(x - rx, y - ry)
            if d < best_d:
                best_d = d
                best_i = i

        out = Path()
        out.header = path.header
        out.poses = path.poses[best_i:]
        return out

    def _nearest_pose_index(self, path: Path, rx: float, ry: float) -> Tuple[int, float]:
        """Return (index, distance) of the closest pose in path to (rx, ry)."""
        best_i = 0
        best_d = float('inf')
        for i, ps in enumerate(path.poses):
            x = ps.pose.position.x
            y = ps.pose.position.y
            d = math.hypot(x - rx, y - ry)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i, best_d

    def _path_length_from(self, path: Path, start_i: int = 0) -> float:
        if len(path.poses) < 2:
            return 0.0
        s = 0.0
        i0 = max(0, min(len(path.poses) - 1, int(start_i)))
        for i in range(i0, len(path.poses) - 1):
            p = path.poses[i].pose.position
            q = path.poses[i + 1].pose.position
            s += math.hypot(q.x - p.x, q.y - p.y)
        return s

    def _index_ahead_by_dist(self, path: Path, start_i: int, dist_m: float) -> int:
        """Walk forward along path from start_i by dist_m and return resulting index."""
        if len(path.poses) < 2:
            return 0
        if dist_m <= 0.0:
            return max(0, min(len(path.poses) - 1, int(start_i)))
        i = max(0, min(len(path.poses) - 1, int(start_i)))
        remaining = float(dist_m)
        while i < len(path.poses) - 1 and remaining > 1e-6:
            p = path.poses[i].pose.position
            q = path.poses[i + 1].pose.position
            ds = math.hypot(q.x - p.x, q.y - p.y)
            if ds <= remaining:
                remaining -= ds
                i += 1
            else:
                break
        return i

    def _slice_path(self, path: Path, start_i: int) -> Path:
        out = Path()
        out.header = path.header
        out.poses = path.poses[max(0, min(len(path.poses), int(start_i))):]
        return out

    def _publish_cmd_vel(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self._pub_cmd.publish(msg)

    def _stop_cmd_vel(self):
        self._publish_cmd_vel(0.0, 0.0)

    def _init_detour_watchdog(self):
        """Initialize detour watchdog timers (called when a detour FollowPath goal is accepted)."""
        if not bool(self.get_parameter('detour_watchdog_enable').value):
            return
        now = time.monotonic()
        self._detour_wd_started_t = now
        self._detour_wd_last_progress_t = now
        self._detour_wd_last_xy = None
        try:
            frame = self._active_path.header.frame_id if self._active_path is not None else 'map'
            stamp = self.get_clock().now().to_msg()
            self._detour_wd_last_xy = self._get_robot_xy(frame, stamp)
        except Exception:
            self._detour_wd_last_xy = None

    def _tick_detour_watchdog(self):
        """Abort detour FollowPath early if the robot is not making translational progress."""
        if self._follow_kind != 'DETOUR' or self._mode_state != 'FOLLOWING':
            return
        if not bool(self.get_parameter('detour_watchdog_enable').value):
            return
        # only valid after goal accepted
        if self._result_future is None or self._goal_handle is None:
            return
        now = time.monotonic()
        if self._detour_wd_started_t <= 0.0:
            self._init_detour_watchdog()
            return
        grace = float(self.get_parameter('detour_watchdog_grace_sec').value)
        if now - self._detour_wd_started_t < grace:
            return
        frame = self._active_path.header.frame_id if self._active_path is not None else 'map'
        stamp = self.get_clock().now().to_msg()
        xy = self._get_robot_xy(frame, stamp)
        if xy is None:
            return
        if self._detour_wd_last_xy is None:
            self._detour_wd_last_xy = xy
            return
        dx = float(xy[0]) - float(self._detour_wd_last_xy[0])
        dy = float(xy[1]) - float(self._detour_wd_last_xy[1])
        dist = math.hypot(dx, dy)
        min_prog = float(self.get_parameter('detour_watchdog_min_progress_m').value)
        if dist >= min_prog:
            self._detour_wd_last_xy = xy
            self._detour_wd_last_progress_t = now
            return
        timeout = float(self.get_parameter('detour_watchdog_timeout_sec').value)
        if now - self._detour_wd_last_progress_t >= timeout:
            self.get_logger().warn(
                f"[supervisor] detour watchdog fired: no translation >= {min_prog:.2f}m for {now - self._detour_wd_last_progress_t:.1f}s -> cancel + next candidate"
            )
            self._maybe_stale_shadow_cleanup('detour_watchdog')
            # cancel current FollowPath goal and move on
            self._cancel_follow()
            self._stop_cmd_vel()
            # clear futures to prevent double-advance when the cancel result arrives
            self._send_future = None
            self._result_future = None
            self._goal_handle = None
            self._advance_detour_candidate('watchdog_no_progress')
            if self._detour_exhausted():
                self._handle_detour_exhausted()
                return
            # if observe started, keep OBSERVING, else compute next candidate
            if self._mode_state != 'OBSERVING':
                self._mode_state = 'DET_COMPUTING'
            return

    # -------- queue helpers --------

    def _tick_detour_compute(self):
        """ComputePathToPose pipeline for the current detour candidate."""
        if self._mode_state != 'DET_COMPUTING':
            return
        if self._detour_exhausted():
            self._handle_detour_exhausted()
            return
        now_mono = time.monotonic()
        if now_mono < self._detour_stop_hold_until or now_mono < self._detour_delay_until:
            self._stop_cmd_vel()
            return

        # ensure we have an active candidate and a compute request
        if self._compute_future is None and self._detour_active is None:
            cand = self._detour_candidates[self._detour_cand_i]
            self._send_compute_for_candidate(cand)

        # handle goal response
        if self._compute_future is not None and self._compute_goal_handle is None and self._compute_future.done():
            gh = self._compute_future.result()
            self._compute_future = None
            if gh is None or not gh.accepted:
                self._advance_detour_candidate('compute_goal_rejected')
                return
            self._compute_goal_handle = gh
            self._compute_result_future = gh.get_result_async()

        # handle result
        if self._compute_result_future is not None and self._compute_result_future.done():
            res = self._compute_result_future.result()
            self._compute_result_future = None
            self._compute_goal_handle = None
            if res is None or res.result is None or not hasattr(res.result, 'path'):
                self._advance_detour_candidate('compute_no_result')
                return
            detour_path = res.result.path
            if detour_path is None or len(detour_path.poses) < 2:
                self._advance_detour_candidate('compute_empty_path')
                return
            if self._detour_active is not None and (not self._detour_path_acceptable(detour_path, self._detour_active)):
                self._advance_detour_candidate('compute_path_rejected')
                if self._detour_exhausted():
                    self._handle_detour_exhausted()
                return
            # publish + follow detour
            self._active_path = detour_path
            self._pub_path.publish(detour_path)
            self._follow_kind = 'DETOUR'
            self._mode_state = 'FOLLOWING'
            # Reset detour watchdog for this candidate
            self._detour_wd_started_t = 0.0
            self._detour_wd_last_progress_t = 0.0
            self._detour_wd_last_xy = None
            self._send_follow(detour_path)

    def _pop_next_ready(self) -> Optional[int]:
        """Pop next segment idx that is not in cooldown.

        Main-pass policy:
          1) Prefer forward progress over revisiting older deferred segments.
          2) Keep only a small revisit window during the main pass.
          3) Segments that fall too far behind are converted to cleanup-only.
        """
        if not self._queue:
            return None
        now = time.monotonic()
        preserve = bool(self.get_parameter('preserve_plan_order').value)

        if preserve:
            ready_forward: List[int] = []
            ready_near_backtrack: List[int] = []
            stale_backtrack: List[int] = []
            purge_retired: List[int] = []
            frontier = int(self._main_pass_frontier_idx)

            for idx in list(self._queue):
                idx = int(idx)
                if idx in self._retired_segments:
                    purge_retired.append(idx)
                    continue
                if self._should_move_deferred_to_cleanup(idx):
                    stale_backtrack.append(idx)
                    continue
                if now < float(self._cooldown_until.get(int(idx), 0.0)):
                    continue
                if frontier >= 0 and self._is_original_plan_segment(idx) and idx <= frontier:
                    ready_near_backtrack.append(idx)
                else:
                    ready_forward.append(idx)

            for idx in purge_retired:
                self._queue = [int(q) for q in self._queue if int(q) != int(idx)]
            for idx in stale_backtrack:
                self._move_segment_to_cleanup_only(int(idx), reason=f'main_pass_window(frontier={frontier})')

            pick: Optional[int] = None
            if ready_forward:
                pick = int(ready_forward[0])
            elif ready_near_backtrack:
                pick = int(ready_near_backtrack[0])

            if pick is None:
                return None
            try:
                self._queue.remove(int(pick))
            except ValueError:
                return None
            return int(pick)

        # Legacy behaviour: rotate cooldown segments to the tail. Retired / stale deferred
        # segments are still purged so they cannot reappear unexpectedly.
        for idx in list(self._queue):
            if int(idx) in self._retired_segments:
                self._queue = [int(q) for q in self._queue if int(q) != int(idx)]
                continue
            if self._should_move_deferred_to_cleanup(int(idx)):
                self._move_segment_to_cleanup_only(int(idx), reason=f'main_pass_window(frontier={int(self._main_pass_frontier_idx)})')

        n = len(self._queue)
        for _ in range(n):
            idx = int(self._queue.pop(0))
            if now >= float(self._cooldown_until.get(idx, 0.0)):
                return idx
            self._queue.append(idx)
        return None

    def _queue_insert_defer(self, idx: int):
        """Insert idx back into queue.

        If `preserve_plan_order` is True, insert it by the original plan order (segment index),
        instead of appending to the tail. This keeps the execution order close to the input plan
        and avoids 'all deferred segments at the end' behaviour.
        """
        if idx in self._retired_segments:
            return
        if idx in self._queue:
            return

        preserve = bool(self.get_parameter('preserve_plan_order').value)
        if preserve:
            pos = 0
            while pos < len(self._queue) and int(self._queue[pos]) < int(idx):
                pos += 1
            self._queue.insert(pos, int(idx))
            return

        # Legacy: cluster-aware insertion (may move segment to the tail)
        cluster_strict = bool(self.get_parameter('cluster_strict_defer').value)
        cid = int(self._segments[idx].cluster_id)
        if (not cluster_strict) or cid < 0:
            self._queue.append(idx)
            return
        last_pos = -1
        for i, qidx in enumerate(self._queue):
            if int(self._segments[qidx].cluster_id) == cid:
                last_pos = i
        if last_pos >= 0:
            self._queue.insert(last_pos + 1, idx)
        else:
            self._queue.append(idx)


    def _seg_cluster(self, seg_idx: Optional[int]) -> Optional[int]:
        """Return cluster_id for a segment index, or None if unknown."""
        if seg_idx is None:
            return None
        try:
            i = int(seg_idx)
            if i < 0 or i >= len(self._segments):
                return None
            cid = int(getattr(self._segments[i], 'cluster_id', -1))
            return cid if cid >= 0 else None
        except Exception:
            return None

    def _seg_kind(self, seg_idx: Optional[int]) -> Optional[str]:
        if seg_idx is None:
            return None
        try:
            i = int(seg_idx)
            if i < 0 or i >= len(self._segments):
                return None
            return str(getattr(self._segments[i], 'kind', 'segment')).lower()
        except Exception:
            return None

    def _seg_id(self, seg_idx: Optional[int]) -> Optional[str]:
        if seg_idx is None:
            return None
        try:
            i = int(seg_idx)
            if i < 0 or i >= len(self._segments):
                return None
            return str(getattr(self._segments[i], 'sid', ''))
        except Exception:
            return None

    def _seg_is_runtime_or_geom_plan(self, seg_idx: Optional[int]) -> bool:
        if seg_idx is None:
            return False
        try:
            i = int(seg_idx)
        except Exception:
            return False
        if i < 0 or i >= len(self._segments):
            return False
        if i >= int(self._base_plan_segment_count):
            return True
        sid = str(self._seg_id(i) or '').lower()
        kind = str(self._seg_kind(i) or '').lower()
        return (
            sid.startswith('geom_plan_')
            or sid.startswith('d2_seg_')
            or sid.startswith('runtime_')
            or kind.startswith('geom_')
            or kind in ('runtime', 'runtime_path', 'd2', 'd2_runtime')
        )

    def _d2_guard_reason(self, snapshot: Optional[Dict[str, Any]], *, forced: bool = False) -> str:
        if snapshot is None:
            return 'snapshot_unavailable'
        if bool(self.get_parameter('d2_disable_after_first_accept').value) and self._d2_one_shot_consumed:
            return 'one_shot_consumed'

        active_idx = snapshot.get('active_seg_idx', self._active_seg_idx)
        if bool(self.get_parameter('d2_disable_on_runtime_segments').value) and self._seg_is_runtime_or_geom_plan(active_idx):
            sid = str(self._seg_id(active_idx) or f'seg_{int(active_idx):04d}')
            return f'runtime_segment:{sid}'

        region = dict(snapshot.get('region_summary', {}) or {})
        hard_d2 = bool(region.get('hard_d2', False))
        if bool(self.get_parameter('d2_require_hard_d2').value) and not hard_d2:
            try:
                score = float(region.get('score_d2', 0.0))
            except Exception:
                score = 0.0
            return f'hard_d2_required(score={score:.2f})'
        return ''

    def _log_d2_guard_skip(self, reason: str, *, forced: bool = False):
        reason = str(reason or '').strip()
        if not reason:
            self._last_d2_guard_skip = ''
            return
        if reason == self._last_d2_guard_skip:
            return
        tag = 'D2 TEST FORCE skipped' if forced else 'D2 skipped'
        self.get_logger().warn(f'[supervisor] {tag}: {reason}')
        self._last_d2_guard_skip = reason

    def _is_c_reason(self, reason: str) -> bool:
        rr = str(reason).upper()
        return rr == 'C' or rr.startswith('C1') or rr.startswith('C2')

    def _wrap_pi(self, a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def _segment_nominal_yaw(self, seg_idx: int) -> Optional[float]:
        try:
            seg = self._segments[int(seg_idx)]
            if seg.poses:
                q = seg.poses[0].pose.orientation
                return _yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        except Exception:
            pass
        return None

    def _active_remaining_length(self) -> float:
        if self._active_path is None:
            return 0.0
        try:
            cur_i = int(self._current_progress_index())
            return float(self._path_length_from(self._active_path, cur_i))
        except Exception:
            return 0.0

    def _segment_total_length(self, seg_idx: Optional[int]) -> float:
        if seg_idx is None:
            return 0.0
        try:
            seg = self._segments[int(seg_idx)]
        except Exception:
            return 0.0
        try:
            full = self._path_from_segment(seg, self.get_clock().now().to_msg())
            return float(self._path_length_from(full, 0))
        except Exception:
            return 0.0

    def _active_progress_ratio(self) -> float:
        if self._active_seg_idx is None:
            return 0.0
        total = self._segment_total_length(self._active_seg_idx)
        rem = self._active_remaining_length()
        if total <= 1e-6:
            return 1.0
        return max(0.0, min(1.0, (float(total) - float(rem)) / float(total)))

    def _close_segment_without_revisit(self, seg_idx: int, reason: str):
        seg_idx = int(seg_idx)
        self._retired_segments.add(seg_idx)
        self._queue = [int(q) for q in self._queue if int(q) != seg_idx]
        seg = self._segments[seg_idx]
        self._append_cleanup_log({
            'type': 'accepted_skip_segment',
            'reason': str(reason),
            'seg_id': seg.sid,
            'cluster_id': int(seg.cluster_id),
            'time': time.time(),
        })
        self._publish_recovery_event(
            'skip_segment', cluster_id=self._seg_cluster(seg_idx), seg_id=seg.sid,
            reason=str(reason), accepted_skip=True)
        self.get_logger().info(f"[supervisor] close seg {seg.sid} without revisit ({reason})")

    def _should_close_active_on_defer(self, reason: str, rem_len: float) -> bool:
        if not bool(self.get_parameter('defer_close_on_major_progress').value):
            return False
        # Only apply this to late-stage defers. Early defers should remain recoverable.
        if str(reason) not in ('wait_timeout', 'D_attempts_exceeded', 'D_no_candidates'):
            return False
        prog = self._active_progress_ratio()
        prog_thr = float(self.get_parameter('defer_close_progress_ratio').value)
        rem_thr = float(self.get_parameter('defer_close_remaining_m').value)
        return bool(prog >= prog_thr or (rem_len >= 0.0 and rem_len <= rem_thr))

    def _should_close_origin_after_cross_seg(self) -> bool:
        if not bool(self.get_parameter('cross_seg_close_origin_enable').value):
            return False
        prog = float(self._detour_origin_progress_ratio)
        rem = float(self._detour_origin_remaining_len)
        prog_thr = float(self.get_parameter('cross_seg_close_origin_progress_ratio').value)
        rem_thr = float(self.get_parameter('cross_seg_close_origin_remaining_m').value)
        return bool(prog >= prog_thr or (rem >= 0.0 and rem <= rem_thr))

    def _tiny_residual_threshold(self, seg_idx: Optional[int]) -> float:
        if not bool(self.get_parameter('tiny_residual_ignore_enable').value):
            return -1.0
        kind = self._seg_kind(seg_idx)
        if kind == 'turn':
            return float(self.get_parameter('tiny_residual_turn_length_m').value)
        return float(self.get_parameter('tiny_residual_length_m').value)

    def _should_finish_close_active(self, reason: str, rem_len: float) -> bool:
        if self._active_seg_idx is None:
            return False
        if not bool(self.get_parameter('endgame_finish_enable').value):
            return False
        if len(self._queue) > int(self.get_parameter('endgame_remaining_segments_le').value):
            return False
        if rem_len < 0.0:
            return False
        if rem_len <= float(self.get_parameter('endgame_close_remaining_m').value):
            return True
        turn_thr = float(self.get_parameter('tiny_residual_turn_length_m').value)
        return bool(self._seg_kind(self._active_seg_idx) == 'turn' and rem_len <= max(turn_thr, float(self.get_parameter('endgame_close_remaining_m').value)))

    def _announce_mission_complete_if_idle(self):
        if self._mission_complete_announced:
            return
        if self._active_seg_idx is not None:
            return
        if self._mode_state != 'IDLE':
            return
        if self._queue:
            return
        self._mission_complete_announced = True
        self._clear_speed_limit()
        self._publish_recovery_event('mission_complete', remaining_queue=0)
        self.get_logger().info('[supervisor] mission complete')

    def _start_short_backup(self, after_state: str, note: str = '',
                            backup_sec: Optional[float] = None,
                            backup_vel: Optional[float] = None,
                            settle_sec: Optional[float] = None,
                            after_follow_kind: Optional[str] = None) -> bool:
        """Short backup+settle helper.

        Used both before the initial C1/C2 compute and as a lightweight mid-detour recovery
        before recomputing a path to the same rejoin target.
        """
        if backup_sec is None:
            backup_sec = float(self.get_parameter('c_prebackup_backup_sec').value)
        if backup_vel is None:
            backup_vel = float(self.get_parameter('c_prebackup_backup_vel').value)
        if settle_sec is None:
            settle_sec = float(self.get_parameter('c_prebackup_settle_sec').value)
        steps: List[Tuple[Twist, float]] = []
        if backup_sec > 1e-3:
            t = Twist(); t.linear.x = float(backup_vel)
            steps.append((t, max(0.0, backup_sec)))
        if settle_sec > 1e-3:
            t = Twist()
            steps.append((t, max(0.0, settle_sec)))
        if not steps:
            return False
        self._observe_steps = steps
        self._observe_step_i = 0
        self._observe_step_deadline = time.monotonic() + float(steps[0][1])
        self._observe_after_state = after_state
        if after_follow_kind is None:
            after_follow_kind = self._follow_kind if self._follow_kind != 'OBSERVE' else 'SEGMENT'
        self._observe_after_follow_kind = str(after_follow_kind)
        self._observe_note = note
        self._follow_kind = 'OBSERVE'
        self._mode_state = 'OBSERVING'
        self.get_logger().warn(f"[supervisor] short backup before detour -> {after_state} {note}")
        return True

    def _should_prebackup_for_c(self, cands: List[_DetourCandidate], origin_pose_idx: int) -> bool:
        if not bool(self.get_parameter('c_prebackup_enable').value):
            return False
        if self._active_path is None or self._active_seg_idx is None:
            return False
        if len(cands) <= int(self.get_parameter('c_prebackup_if_candidates_le').value):
            return True
        if self._seg_kind(self._active_seg_idx) == 'turn':
            return True
        try:
            stamp = self.get_clock().now().to_msg()
            obs = self._find_blocking_track(self._active_path, int(origin_pose_idx), stamp)
        except Exception:
            obs = None
        if obs is None:
            return False
        if float(obs.get('dist', 1e9)) <= float(self.get_parameter('c_prebackup_obs_dist_m').value):
            return True
        if float(obs.get('s_ahead', 1e9)) <= float(self.get_parameter('c_prebackup_obs_ahead_m').value):
            return True
        return False

    def _detour_path_acceptable(self, detour_path: Path, cand: _DetourCandidate) -> bool:
        """Filter obviously poor C1/C2 detours (very long or strongly backtracking)."""
        reason = str(self._detour_reason).upper()
        if not self._is_c_reason(reason):
            return True
        if detour_path is None or len(detour_path.poses) < 2:
            return False
        max_len = float(self.get_parameter('c2_detour_max_length_m').value) if reason.startswith('C2') else float(self.get_parameter('c_detour_max_length_m').value)
        max_ratio = float(self.get_parameter('c_detour_max_stretch_ratio').value)
        max_back = float(self.get_parameter('c_detour_max_initial_backtrack_m').value)
        plen = float(self._path_length_from(detour_path, 0))
        p0 = detour_path.poses[0].pose.position
        goal = cand.target_pose.pose.position
        direct = math.hypot(float(goal.x) - float(p0.x), float(goal.y) - float(p0.y))
        if plen > max_len:
            self.get_logger().warn(
                f"[supervisor] reject {self._detour_reason} detour: path too long ({plen:.2f}m > {max_len:.2f}m)")
            return False
        if direct > 0.25 and (plen / max(0.05, direct)) > max_ratio:
            self.get_logger().warn(
                f"[supervisor] reject {self._detour_reason} detour: stretch ratio too high ({plen/direct:.2f} > {max_ratio:.2f})")
            return False
        if direct > 0.10:
            ux = (float(goal.x) - float(p0.x)) / direct
            uy = (float(goal.y) - float(p0.y)) / direct
            n = min(len(detour_path.poses), 10)
            min_proj = 0.0
            for i in range(n):
                pi = detour_path.poses[i].pose.position
                proj = (float(pi.x) - float(p0.x)) * ux + (float(pi.y) - float(p0.y)) * uy
                if proj < min_proj:
                    min_proj = proj
            if min_proj < -max_back:
                self.get_logger().warn(
                    f"[supervisor] reject {self._detour_reason} detour: initial backtrack {abs(min_proj):.2f}m > {max_back:.2f}m")
                return False
        return True

    def _schedule_c_detour_refresh(self, why: str) -> bool:
        """Recompute the current C1/C2 detour target after a short backup.

        This mirrors NavigateToPose's 'same goal, refresh the path' behavior, but for the
        current local rejoin target. It is intentionally lighter than escalating to D or doing
        a full observe-spin recovery.
        """
        if not bool(self.get_parameter('c_dynamic_refresh_enable').value):
            return False
        if not self._is_c_reason(self._detour_reason):
            return False
        if self._detour_cand_i >= len(self._detour_candidates):
            return False
        max_refresh = int(self.get_parameter('c_dynamic_refresh_max_per_candidate').value)
        if self._detour_refresh_count >= max_refresh:
            return False
        now = time.monotonic()
        cd = float(self.get_parameter('c_dynamic_refresh_cooldown_sec').value)
        if (now - float(self._detour_last_refresh_t)) < cd:
            return False

        # Cancel current follow / clear futures and recompute to the SAME candidate after a small backup.
        self._cancel_follow()
        self._stop_cmd_vel()
        self._send_future = None
        self._result_future = None
        self._goal_handle = None
        self._cancel_future = None
        self._compute_future = None
        self._compute_goal_handle = None
        self._compute_result_future = None
        self._detour_active = None  # let DET_COMPUTING resend current candidate
        self._mode_state = 'DET_COMPUTING'
        self._follow_kind = 'DETOUR'
        self._detour_refresh_count += 1
        self._detour_last_refresh_t = now
        self._detour_stop_hold_until = now + max(0.0, float(self.get_parameter('detour_stop_hold_sec').value))
        self._detour_delay_until = now + max(0.0, float(self.get_parameter('detour_delay_compute_sec').value))

        self.get_logger().warn(
            f"[supervisor] C detour refresh #{self._detour_refresh_count}: recompute current target ({why})")
        self._publish_recovery_event('c_refresh', reason=str(why), refresh_count=int(self._detour_refresh_count))
        if self._maybe_preempt_c_with_d(f'refresh_{self._detour_refresh_count}'):
            return True

        return self._start_short_backup(
            after_state='DET_COMPUTING',
            note=f'c_refresh:{why}',
            backup_sec=float(self.get_parameter('c_dynamic_refresh_backup_sec').value),
            backup_vel=float(self.get_parameter('c_dynamic_refresh_backup_vel').value),
            settle_sec=float(self.get_parameter('c_dynamic_refresh_settle_sec').value),
            after_follow_kind='DETOUR')

    def _tick_c_detour_refresh(self):
        """If a C1/C2 detour stalls for a short time, refresh the path to the same target.

        This is the key improvement borrowed from NavigateToPose-style navigation: when the
        environment changes during execution, first replan to the same goal instead of waiting
        for a long watchdog timeout and escalating to a heavier recovery.
        """
        if self._follow_kind != 'DETOUR' or self._mode_state != 'FOLLOWING':
            return
        if not self._is_c_reason(self._detour_reason):
            return
        if not bool(self.get_parameter('c_dynamic_refresh_enable').value):
            return
        if self._result_future is None or self._goal_handle is None:
            return
        if self._detour_wd_started_t <= 0.0:
            return
        now = time.monotonic()
        grace = float(self.get_parameter('detour_watchdog_grace_sec').value)
        if now - self._detour_wd_started_t < grace:
            return
        no_prog = float(self.get_parameter('c_dynamic_refresh_no_progress_sec').value)
        if (now - float(self._detour_wd_last_progress_t)) < no_prog:
            return
        if self._schedule_c_detour_refresh(f'no_progress_{now - float(self._detour_wd_last_progress_t):.1f}s'):
            return

    def _start_segment_follow(self, path: Path, seg_idx: int):
        """Start segment following, optionally with a short turn pre-align phase."""
        self._mission_complete_announced = False
        self._mark_main_pass_frontier(seg_idx)
        self._active_path = path
        self._pub_path.publish(path)
        self._follow_kind = 'SEGMENT'

        if bool(self.get_parameter('turn_prealign_enable').value) and self._seg_kind(seg_idx) == 'turn':
            stamp = self.get_clock().now().to_msg()
            tf2d = self._lookup_tf2d(path.header.frame_id, self.get_parameter('robot_base_frame').value, stamp)
            tyaw = self._segment_nominal_yaw(seg_idx)
            if tf2d is not None and tyaw is not None:
                _tx, _ty, ryaw = tf2d
                err = self._wrap_pi(float(tyaw) - float(ryaw))
                if abs(err) >= float(self.get_parameter('turn_prealign_heading_error_rad').value):
                    self._turn_prealign_target_yaw = float(tyaw)
                    self._turn_prealign_deadline = time.monotonic() + float(self.get_parameter('turn_prealign_timeout_sec').value)
                    self._mode_state = 'TURN_PREALIGN'
                    self._stop_cmd_vel()
                    self.get_logger().info(
                        f"[supervisor] turn-prealign seg {self._segments[int(seg_idx)].sid} err={err:.2f}rad")
                    return

        self._mode_state = 'FOLLOWING'
        self._send_follow(path)

    def _tick_turn_prealign(self):
        if self._active_seg_idx is None or self._active_path is None:
            self._stop_cmd_vel()
            self._mode_state = 'IDLE'
            return

        stamp = self.get_clock().now().to_msg()
        tf2d = self._lookup_tf2d(self._active_path.header.frame_id, self.get_parameter('robot_base_frame').value, stamp)
        if tf2d is None:
            if time.monotonic() >= float(self._turn_prealign_deadline):
                self.get_logger().warn('[supervisor] turn-prealign TF timeout -> continue with FollowPath')
                self._stop_cmd_vel()
                self._mode_state = 'FOLLOWING'
                self._send_follow(self._active_path)
            return

        _tx, _ty, ryaw = tf2d
        err = self._wrap_pi(float(self._turn_prealign_target_yaw) - float(ryaw))
        tol = float(self.get_parameter('turn_prealign_yaw_tolerance_rad').value)
        if abs(err) <= tol:
            self._stop_cmd_vel()
            self._mode_state = 'FOLLOWING'
            self._send_follow(self._active_path)
            self.get_logger().info(
                f"[supervisor] turn-prealign aligned on {self._segments[int(self._active_seg_idx)].sid}; start FollowPath")
            return

        if time.monotonic() >= float(self._turn_prealign_deadline):
            self._stop_cmd_vel()
            continue_if_err = float(self.get_parameter('turn_prealign_timeout_continue_if_err_below_rad').value)
            sid = self._segments[int(self._active_seg_idx)].sid
            if abs(err) <= max(tol, continue_if_err):
                self._mode_state = 'FOLLOWING'
                self.get_logger().warn(
                    f"[supervisor] turn-prealign timeout on {sid} but residual err={err:.2f}rad <= {max(tol, continue_if_err):.2f}; start FollowPath")
                self._send_follow(self._active_path)
                return

            self.get_logger().warn(
                f"[supervisor] turn-prealign timeout on {sid} with err={err:.2f}rad -> treat as ABORTED before FollowPath")
            self._publish_recovery_event(
                'turn_prealign_timeout',
                cluster_id=int(self._segments[int(self._active_seg_idx)].cluster_id),
                seg_id=sid,
                err_rad=float(err))
            self._mode_state = 'IDLE'
            self._handle_segment_follow_failure(GoalStatus.STATUS_ABORTED)
            return

        ang = float(self.get_parameter('turn_prealign_angular_vel').value)
        cmd = Twist()
        cmd.angular.z = math.copysign(max(0.1, abs(ang)), err)
        self._pub_cmd.publish(cmd)

    def _set_cooldown(self, idx: int):
        cd = float(self.get_parameter('defer_cooldown_sec').value)
        if cd <= 0.0:
            return
        self._cooldown_until[idx] = time.monotonic() + cd

    # -------- cleanup logging --------
    def _append_cleanup_log(self, rec: Dict[str, Any]):
        path = str(self.get_parameter('cleanup_log_path').value)
        if not path:
            return
        if path.startswith('~'):
            path = os.path.expanduser(path)
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
        except Exception:
            pass
        try:
            with open(path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except Exception as e:
            self.get_logger().warn(f"[supervisor] failed to write cleanup_log: {e}")
        try:
            typ = str(rec.get('type', 'cleanup'))
            evt = dict(rec)
            evt.pop('time', None)
            self._publish_recovery_event(typ, **evt)
        except Exception:
            pass

    # -------- optional costmap checks (best-effort) --------
    def _costmap_is_fresh(self) -> bool:
        if self._costmap is None:
            return False
        max_age = float(self.get_parameter('costmap_max_age_sec').value)
        if max_age <= 0.0:
            return True
        return (time.monotonic() - float(self._costmap_recv_time)) <= max_age

    def _costmap_meta(self):
        cm = self._costmap
        if cm is None:
            return None
        # nav2_msgs/Costmap has 'metadata'
        meta = getattr(cm, 'metadata', None)
        if meta is None:
            meta = getattr(cm, 'meta_data', None)
        return meta

    def _costmap_world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        cm = self._costmap
        if cm is None:
            return None
        meta = self._costmap_meta()
        if meta is None:
            return None
        try:
            res = float(getattr(meta, 'resolution'))
            origin = getattr(meta, 'origin')
            ox = float(origin.position.x)
            oy = float(origin.position.y)
            size_x = int(getattr(meta, 'size_x', getattr(meta, 'width')))
            size_y = int(getattr(meta, 'size_y', getattr(meta, 'height')))
        except Exception:
            return None
        if res <= 0.0:
            return None
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if mx < 0 or my < 0 or mx >= size_x or my >= size_y:
            return None
        return mx, my

    def _costmap_cost_at(self, x: float, y: float) -> Optional[int]:
        cm = self._costmap
        if cm is None:
            return None
        ij = self._costmap_world_to_map(x, y)
        if ij is None:
            return None
        mx, my = ij
        meta = self._costmap_meta()
        if meta is None:
            return None
        try:
            size_x = int(getattr(meta, 'size_x', getattr(meta, 'width')))
            data = getattr(cm, 'data')
            v = int(data[my * size_x + mx])
            return v
        except Exception:
            return None

    def _costmap_point_ok(self, x: float, y: float) -> bool:
        """Return True if point is not UNKNOWN/lethal (best-effort)."""
        if (self._costmap is None) or (not self._costmap_is_fresh()):
            return True  # fail-open
        v = self._costmap_cost_at(x, y)
        if v is None:
            return True
        # nav2 costmap convention: 255 unknown, >=253 lethal-ish
        if v >= 253:
            return False
        return True

    # -------- obstacle helpers (for C/D) --------
    def _dist_along_path(self, path: Path, i0: int, i1: int) -> float:
        if len(path.poses) < 2:
            return 0.0
        a = max(0, min(len(path.poses) - 1, int(i0)))
        b = max(0, min(len(path.poses) - 1, int(i1)))
        if b <= a:
            return 0.0
        s = 0.0
        for i in range(a, b):
            p = path.poses[i].pose.position
            q = path.poses[i + 1].pose.position
            s += math.hypot(q.x - p.x, q.y - p.y)
        return s

    def _find_blocking_track(self, path: Path, cur_i: int, stamp) -> Optional[Dict[str, Any]]:
        """Find the closest (along-path) track that intersects the path corridor ahead."""
        if self._tracks is None or not self._tracks.tracks:
            return None

        corridor = float(self.get_parameter('c_corridor_radius_m').value)
        tracks_frame = str(self._tracks.header.frame_id)
        tf = self._lookup_tf2d(path.header.frame_id, tracks_frame, stamp)
        if tf is None:
            return None

        best: Optional[Dict[str, Any]] = None
        for tr in self._tracks.tracks:
            x0 = float(tr.pose.position.x)
            y0 = float(tr.pose.position.y)
            tx, ty, tyaw = tf
            xw, yw = _tf2d_apply(tx, ty, tyaw, x0, y0)

            # find nearest path pose index ahead of cur_i
            best_j = cur_i
            best_d = float('inf')
            for j in range(cur_i, len(path.poses)):
                px = float(path.poses[j].pose.position.x)
                py = float(path.poses[j].pose.position.y)
                d = math.hypot(px - xw, py - yw)
                if d < best_d:
                    best_d = d
                    best_j = j
            if best_d > corridor:
                continue

            s_ahead = self._dist_along_path(path, cur_i, best_j)
            if best is None or s_ahead < float(best['s_ahead']):
                # ObstacleTrack message uses `velocity` (geometry_msgs/Twist).
                # Keep backward-compatibility with older fields if present.
                vx = 0.0
                vy = 0.0
                try:
                    vx = float(tr.velocity.linear.x)
                    vy = float(tr.velocity.linear.y)
                except Exception:
                    try:
                        vx = float(tr.twist.linear.x)  # type: ignore[attr-defined]
                        vy = float(tr.twist.linear.y)  # type: ignore[attr-defined]
                    except Exception:
                        vx = float(getattr(tr, 'vx', 0.0))
                        vy = float(getattr(tr, 'vy', 0.0))
                speed = math.hypot(vx, vy)
                best = {
                    'id': str(tr.id),
                    'x': xw,
                    'y': yw,
                    'radius': float(tr.radius),
                    'speed': speed,
                    'pose_i': int(best_j),
                    's_ahead': float(s_ahead),
                    'dist': float(best_d),
                }
        return best

    def _rejoin_candidates_on_active_segment(self, reason: str) -> List[_DetourCandidate]:
        """Build *same-segment* rejoin candidates along the current active segment.

        C1 uses this for local recovery inside the same segment.
        D also uses this as its first, order-preserving fallback before any broader recovery.
        """
        if self._active_path is None or self._active_seg_idx is None:
            return []
        path = self._active_path
        stamp = self.get_clock().now().to_msg()
        r = self._get_robot_xy(path.header.frame_id, stamp)
        if r is None:
            return []
        rx, ry = r
        cur_i, _ = self._nearest_pose_index(path, rx, ry)

        obs = self._find_blocking_track(path, cur_i, stamp)
        if obs is None:
            # No track:
            #   - For C/C1: we require a concrete blocker to compute a meaningful slice+skip window.
            #   - For D: still try to recover by rejoining further ahead on the same segment.
            if str(reason).upper() == 'D':
                candidates: List[_DetourCandidate] = []
                used_idx: set = set()

                # 1) Optional: selector-provided decision rejoin pose (if provided)
                if self._decision is not None:
                    rp = self._decision.decision.rejoin_pose
                    if str(rp.header.frame_id):
                        best_j = 0
                        best_d = float('inf')
                        for j, ps in enumerate(path.poses):
                            d = math.hypot(ps.pose.position.x - rp.pose.position.x,
                                           ps.pose.position.y - rp.pose.position.y)
                            if d < best_d:
                                best_d = d
                                best_j = j
                        if best_j > cur_i:
                            candidates.append(_DetourCandidate(
                                target_pose=rp,
                                resume_seg_idx=int(self._active_seg_idx),
                                resume_pose_idx=int(best_j),
                                skipped=[],
                                note='decision_rejoin_pose'))
                            used_idx.add(int(best_j))

                # 2) Generic "ahead on path" fallbacks (do not require a track)
                try:
                    min_ahead = float(self.get_parameter('d_fallback_rejoin_min_ahead_m').value)
                    step = float(self.get_parameter('d_fallback_rejoin_step_m').value)
                    max_trials = int(self.get_parameter('d_fallback_rejoin_max_trials').value)
                except Exception:
                    min_ahead = 1.5
                    step = 1.0
                    max_trials = 6
                min_remain = int(self.get_parameter('min_remaining_poses_after_trim').value)

                for k in range(max_trials):
                    ahead = min_ahead + k * step
                    idx = self._index_ahead_by_dist(path, cur_i, ahead)
                    if idx <= cur_i:
                        continue
                    if idx in used_idx:
                        continue
                    if (len(path.poses) - idx) < max(2, min_remain):
                        break
                    px = float(path.poses[idx].pose.position.x)
                    py = float(path.poses[idx].pose.position.y)
                    if not self._costmap_point_ok(px, py):
                        continue
                    used_idx.add(idx)
                    candidates.append(_DetourCandidate(
                        target_pose=path.poses[idx],
                        resume_seg_idx=int(self._active_seg_idx),
                        resume_pose_idx=int(idx),
                        skipped=[],
                        note='fallback_ahead'))
                return candidates

            return []

        robot_r = float(self.get_parameter('robot_radius_m').value)
        margin = float(self.get_parameter('c_skip_margin_m').value)
        min_ahead = float(self.get_parameter('c_rejoin_min_ahead_m').value)
        step = float(self.get_parameter('c_rejoin_step_m').value)
        max_trials = int(self.get_parameter('c_rejoin_max_trials').value)
        min_remain = int(self.get_parameter('min_remaining_poses_after_trim').value)

        safe_r = max(0.0, float(obs['radius'])) + robot_r + margin
        block_i = int(obs['pose_i'])

        candidates: List[_DetourCandidate] = []
        used_idx = set()
        for k in range(max_trials):
            ahead = min_ahead + k * step
            idx = self._index_ahead_by_dist(path, block_i, ahead)
            if idx <= cur_i:
                continue
            if idx in used_idx:
                continue
            if (len(path.poses) - idx) < max(2, min_remain):
                break

            px = float(path.poses[idx].pose.position.x)
            py = float(path.poses[idx].pose.position.y)
            if math.hypot(px - float(obs['x']), py - float(obs['y'])) <= safe_r:
                continue
            if not self._costmap_point_ok(px, py):
                continue
            used_idx.add(idx)
            skipped = []
            if self._is_c_reason(reason):
                skipped = [(int(self._active_seg_idx), int(cur_i), int(idx))]
            candidates.append(_DetourCandidate(
                target_pose=path.poses[idx],
                resume_seg_idx=int(self._active_seg_idx),
                resume_pose_idx=int(idx),
                skipped=skipped,
                note=f"track={obs['id']}"))
        return candidates

    def _cluster_jump_candidates(self) -> List[_DetourCandidate]:
        """Pick a few alternative segments inside the *same* cluster when C has no good rejoin.

        Two sort policies are supported (param: d_cluster_jump_sort):
          - 'queue'    : keep the planned execution order (recommended for coverage).
          - 'distance' : pick the closest start pose to the robot (legacy behaviour).
        """
        if self._active_seg_idx is None:
            return []
        cid = self._seg_cluster(self._active_seg_idx)
        if cid is None:
            return []

        sort_mode = str(self.get_parameter('d_cluster_jump_sort').value).lower()
        max_seg = int(self.get_parameter('d_candidate_max_segments').value)

        # 1) Queue-order: keep original segment order as much as possible
        if sort_mode.startswith('queue'):
            out: List[_DetourCandidate] = []
            origin_seg_idx = int(self._active_seg_idx) if self._active_seg_idx is not None else None
            for seg_idx in self._queue:
                if len(out) >= max_seg:
                    break
                if seg_idx == self._active_seg_idx:
                    continue
                if self._seg_cluster(seg_idx) != cid:
                    continue
                if not self._within_forward_jump_limits(int(seg_idx), origin_seg_idx):
                    continue
                seg = self._segments[seg_idx]
                out.append(_DetourCandidate(
                    target_pose=seg.poses[0],
                    resume_seg_idx=int(seg_idx),
                    resume_pose_idx=0,
                    skipped=[],
                    note='cluster_jump'))
            return out

        # 2) Distance-order (legacy): pick closest start poses
        if self._last_odom_stamp is None:
            return []
        tf = self._lookup_tf('map', 'base_link', self._last_odom_stamp)
        if tf is None:
            return []
        bx, by = self._base_pose_map(tf)
        cands: List[Tuple[float, int]] = []
        origin_seg_idx = int(self._active_seg_idx) if self._active_seg_idx is not None else None
        for seg_idx in self._queue:
            if self._seg_cluster(seg_idx) != cid or seg_idx == self._active_seg_idx:
                continue
            if not self._within_forward_jump_limits(int(seg_idx), origin_seg_idx):
                continue
            seg = self._segments[seg_idx]
            sx, sy = float(seg.poses[0].pose.position.x), float(seg.poses[0].pose.position.y)
            dx, dy = sx - bx, sy - by
            cands.append((math.hypot(dx, dy), seg_idx))
        cands.sort(key=lambda t: t[0])

        out: List[_DetourCandidate] = []
        for _, seg_idx in cands[:max_seg]:
            seg = self._segments[seg_idx]
            out.append(_DetourCandidate(
                target_pose=seg.poses[0],
                resume_seg_idx=int(seg_idx),
                resume_pose_idx=0,
                skipped=[],
                note='cluster_jump'))
        return out
    def _c2_ordered_exit_candidates(self, origin_pose_idx: int) -> List[_DetourCandidate]:
        """C2: ordered exit recovery.

        Only consider the *next* queued segment in the same cluster and generate ordered
        rejoin points on that segment (start point + a few forward points). This preserves
        segment order much better than a generic cluster jump while still allowing us to
        bypass a blocked turn/segment exit.
        """
        if self._active_seg_idx is None or self._active_path is None:
            return []
        if not bool(self.get_parameter('c2_enable_ordered_exit_rejoin').value):
            return []

        cur_seg_idx = int(self._active_seg_idx)
        cid = self._seg_cluster(cur_seg_idx)
        cur_kind = self._seg_kind(cur_seg_idx)

        next_idx: Optional[int] = None
        if str(cur_kind) == 'turn' and bool(self.get_parameter('c2_turn_use_next_queue_segment').value):
            # For short turn connectors, preserve execution order by targeting the immediate
            # next queued segment, even if the turn segment itself has cluster_id=-1.
            for qidx in self._queue:
                if int(qidx) > cur_seg_idx:
                    next_idx = int(qidx)
                    break
            if next_idx is None and self._queue:
                next_idx = int(self._queue[0])
        else:
            if cid is not None:
                for qidx in self._queue:
                    if self._seg_cluster(int(qidx)) == cid:
                        next_idx = int(qidx)
                        break
            if next_idx is None and bool(self.get_parameter('c2_allow_ordered_fallback_without_cluster').value):
                for qidx in self._queue:
                    if int(qidx) > cur_seg_idx:
                        next_idx = int(qidx)
                        break
                if next_idx is None and self._queue:
                    next_idx = int(self._queue[0])
        if next_idx is None:
            return []
        if not self._within_forward_jump_limits(int(next_idx), cur_seg_idx):
            self.get_logger().info(
                f"[supervisor] C2 ordered-exit suppressed: next_idx={int(next_idx)} exceeds jump window from seg {self._segments[cur_seg_idx].sid}")
            return []

        next_seg = self._segments[next_idx]
        stamp = self.get_clock().now().to_msg()
        next_path = self._path_from_segment(next_seg, stamp)
        if len(next_path.poses) < 1:
            return []

        pts_per_seg = max(1, int(self.get_parameter('c2_next_segment_points_per_segment').value))
        step_m = max(0.05, float(self.get_parameter('c2_next_segment_step_m').value))
        start_offset = max(0.0, float(self.get_parameter('c2_next_segment_start_offset_m').value))
        min_remain = int(self.get_parameter('min_remaining_poses_after_trim').value)

        candidate_indices: List[int] = []
        idx0 = 0
        if start_offset > 1e-6 and len(next_path.poses) >= 2:
            idx0 = self._index_ahead_by_dist(next_path, 0, start_offset)
        candidate_indices.append(int(idx0))
        for k in range(1, pts_per_seg):
            idx = self._index_ahead_by_dist(next_path, int(idx0), float(k) * step_m)
            candidate_indices.append(int(idx))

        # ordered, unique, within bounds
        ordered_unique: List[int] = []
        seen = set()
        for idx in candidate_indices:
            idx = max(0, min(int(idx), len(next_path.poses) - 1))
            if idx in seen:
                continue
            if (len(next_path.poses) - idx) < max(2, min_remain):
                continue
            seen.add(idx)
            ordered_unique.append(idx)

        out: List[_DetourCandidate] = []
        for idx in ordered_unique:
            px = float(next_path.poses[idx].pose.position.x)
            py = float(next_path.poses[idx].pose.position.y)
            if not self._costmap_point_ok(px, py):
                continue
            skipped: List[Tuple[int, int, int]] = [
                (cur_seg_idx, int(origin_pose_idx), len(self._active_path.poses))
            ]
            if idx > 0:
                skipped.append((next_idx, 0, int(idx)))
            out.append(_DetourCandidate(
                target_pose=next_path.poses[idx],
                resume_seg_idx=int(next_idx),
                resume_pose_idx=int(idx),
                skipped=skipped,
                note='rejoin_next_segment'))
        return out

    def _start_observe(self, after_state: str, note: str = ""):
        if not bool(self.get_parameter('observe_enable').value):
            return
        spin_rad = float(self.get_parameter('observe_spin_rad').value)
        ang = float(self.get_parameter('observe_angular_vel').value)
        backup_sec = float(self.get_parameter('observe_backup_sec').value)
        backup_vel = float(self.get_parameter('observe_backup_vel').value)
        settle_sec = float(self.get_parameter('observe_settle_sec').value)

        steps: List[Tuple[Twist, float]] = []
        if abs(ang) > 1e-3 and abs(spin_rad) > 1e-3:
            t = Twist(); t.angular.z = float(math.copysign(abs(ang), spin_rad))
            steps.append((t, abs(spin_rad / ang)))
        if backup_sec > 1e-3:
            t = Twist(); t.linear.x = float(backup_vel)
            steps.append((t, max(0.0, backup_sec)))
        if settle_sec > 1e-3:
            t = Twist()  # zeros
            steps.append((t, max(0.0, settle_sec)))

        self._observe_steps = steps
        self._observe_step_i = 0
        self._observe_step_deadline = time.monotonic() + (steps[0][1] if steps else 0.0)
        self._observe_after_state = after_state
        # restore whatever we were doing (detour compute vs normal segment)
        self._observe_after_follow_kind = self._follow_kind if self._follow_kind != 'OBSERVE' else 'SEGMENT'
        self._observe_note = note
        self._follow_kind = 'OBSERVE'
        self._mode_state = 'OBSERVING'
        self.get_logger().warn(f"[supervisor] observe start -> {after_state} {note}")

    def _tick_observe(self):
        if self._mode_state != 'OBSERVING':
            return
        if not self._observe_steps:
            self._stop_cmd_vel()
            self._mode_state = self._observe_after_state
            self._follow_kind = self._observe_after_follow_kind
            return
        now = time.monotonic()
        if now >= float(self._observe_step_deadline):
            self._observe_step_i += 1
            if self._observe_step_i >= len(self._observe_steps):
                self._stop_cmd_vel()
                # continue
                self._mode_state = self._observe_after_state
                self._follow_kind = self._observe_after_follow_kind
                self.get_logger().info(f"[supervisor] observe done -> {self._observe_after_state}")
                return
            # next step
            self._observe_step_deadline = now + float(self._observe_steps[self._observe_step_i][1])
        # publish current step cmd
        cmd, _dur = self._observe_steps[self._observe_step_i]
        self._pub_cmd.publish(cmd)

    def _start_detour(self, reason: str, candidates: List[_DetourCandidate], origin_pose_idx: int):
        if not candidates:
            return
        self._cancel_follow()
        self._stop_cmd_vel()
        # drop any pending follow futures to avoid mixing with detour follow
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._cancel_future = None
        self._detour_reason = str(reason)
        self._detour_candidates = list(candidates)
        self._detour_cand_i = 0
        self._detour_observe_used = 0
        self._detour_active = None
        self._detour_refresh_count = 0
        self._detour_last_refresh_t = -1e9
        self._detour_origin_seg_idx = int(self._active_seg_idx) if self._active_seg_idx is not None else None
        self._detour_origin_pose_idx = int(origin_pose_idx)
        try:
            self._detour_origin_total_len = float(self._segment_total_length(self._active_seg_idx))
            self._detour_origin_remaining_len = float(self._active_remaining_length()) if self._follow_kind == 'SEGMENT' else -1.0
            if self._detour_origin_total_len > 1e-6 and self._detour_origin_remaining_len >= 0.0:
                self._detour_origin_progress_ratio = max(
                    0.0, min(1.0,
                             (self._detour_origin_total_len - self._detour_origin_remaining_len)
                             / self._detour_origin_total_len))
            else:
                self._detour_origin_progress_ratio = 0.0
        except Exception:
            self._detour_origin_total_len = 0.0
            self._detour_origin_remaining_len = -1.0
            self._detour_origin_progress_ratio = 0.0
        now_mono = time.monotonic()
        self._detour_attempt_started_t = now_mono
        stop_hold_sec = float(self.get_parameter('detour_stop_hold_sec').value)
        delay_sec = float(self.get_parameter('detour_delay_compute_sec').value)
        self._detour_stop_hold_until = now_mono + max(0.0, stop_hold_sec)
        self._detour_delay_until = now_mono + max(0.0, delay_sec)
        self._mode_state = 'DET_COMPUTING'
        self._follow_kind = 'DETOUR'
        self._compute_future = None
        self._compute_goal_handle = None
        self._compute_result_future = None
        self.get_logger().warn(f"[supervisor] start detour ({reason}) candidates={len(candidates)}")

    def _send_compute_for_candidate(self, cand: _DetourCandidate):
        goal = ComputePathToPose.Goal()
        goal.goal = cand.target_pose
        goal.planner_id = ""
        goal.use_start = False
        self._compute_future = self._compute.send_goal_async(goal)
        self._compute_goal_handle = None
        self._compute_result_future = None
        self._detour_active = cand

    def _advance_detour_candidate(self, why: str):
        self.get_logger().warn(f"[supervisor] detour candidate failed: {why}")
        if str(why) == 'compute_path_rejected':
            self._publish_recovery_event('path_rejected', cluster_id=self._active_cluster_id(), reason=str(self._detour_reason))
            if self._maybe_preempt_c_with_d('compute_path_rejected'):
                return
        # For C1/C2, first try a lightweight 'replan to the same rejoin target' before
        # rotating / switching candidate. This handles newly appeared obstacles on the detour path
        # much better than waiting for a full observe-spin cycle.
        if self._is_c_reason(self._detour_reason) and (why == 'watchdog_no_progress' or str(why).startswith('follow_failed_')):
            if self._schedule_c_detour_refresh(f'candidate_fail:{why}'):
                return
        # reset any in-flight compute state
        self._compute_future = None
        self._compute_goal_handle = None
        self._compute_result_future = None
        self._detour_active = None
        # optional observe once before trying further
        if bool(self.get_parameter('observe_enable').value) and self._detour_observe_used < 1:
            self._detour_observe_used += 1
            self._start_observe(after_state='DET_COMPUTING', note=f"detour_fail:{why}")
            return
        self._detour_cand_i += 1
        self._detour_refresh_count = 0
        self._detour_last_refresh_t = -1e9

    def _detour_exhausted(self) -> bool:
        return self._detour_cand_i >= len(self._detour_candidates)

    # -------- FollowPath actions --------
    def _ensure_servers(self) -> bool:
        if not self._follow.wait_for_server(timeout_sec=0.0):
            return False
        if not self._compute.wait_for_server(timeout_sec=0.0):
            return False
        return True

    def _reset_detour_state(self):
        self._detour_reason = ""
        self._detour_candidates = []
        self._detour_cand_i = 0
        self._detour_observe_used = 0
        self._detour_active = None
        self._detour_origin_seg_idx = None
        self._detour_origin_pose_idx = 0
        self._detour_origin_total_len = 0.0
        self._detour_origin_remaining_len = -1.0
        self._detour_origin_progress_ratio = 0.0
        self._detour_stop_hold_until = 0.0
        self._detour_delay_until = 0.0
        self._detour_wd_started_t = 0.0
        self._detour_wd_last_progress_t = 0.0
        self._detour_wd_last_xy = None
        self._detour_refresh_count = 0
        self._detour_last_refresh_t = -1e9
        self._pending_d2_scope_supersede = set()
        self._compute_future = None
        self._compute_goal_handle = None
        self._compute_result_future = None

    def _current_progress_index(self) -> int:
        """Nearest pose index on current active_path to the robot."""
        if self._active_path is None:
            return 0
        stamp = self.get_clock().now().to_msg()
        r = self._get_robot_xy(self._active_path.header.frame_id, stamp)
        if r is None:
            return 0
        rx, ry = r
        i, _ = self._nearest_pose_index(self._active_path, rx, ry)
        return int(i)

    def _start_skip_active(self, reason: str):
        if self._active_seg_idx is None:
            return
        idx = int(self._active_seg_idx)
        seg = self._segments[idx]
        self.get_logger().warn(f"[supervisor] skip seg {seg.sid} ({reason})")
        self._retired_segments.add(idx)
        self._append_cleanup_log({
            'type': 'skip_segment',
            'reason': str(reason),
            'seg_id': seg.sid,
            'cluster_id': int(seg.cluster_id),
            'time': time.time(),
        })
        self._cancel_follow()
        self._stop_cmd_vel()
        # drop any pending follow futures to avoid mixing with new tasks
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._cancel_future = None
        self._active_seg_idx = None
        self._active_path = None
        self._mode_state = 'IDLE'
        self._follow_kind = 'SEGMENT'

    def _defer_active(self, reason: str):
        if self._active_seg_idx is None:
            return
        idx = int(self._active_seg_idx)
        seg = self._segments[idx]
        self._defer_count[idx] = int(self._defer_count.get(idx, 0)) + 1
        self._set_cooldown(idx)
        max_times = int(self.get_parameter('defer_max_times').value)
        if self._defer_count[idx] > max_times:
            self.get_logger().warn(f"[supervisor] defer_max_times exceeded -> skip {seg.sid}")
            self._start_skip_active(reason=f"defer_max_times:{reason}")
            return
        rem_len = self._active_remaining_length() if self._follow_kind == 'SEGMENT' else -1.0
        tiny_thr = self._tiny_residual_threshold(idx)
        if rem_len >= 0.0 and tiny_thr > 0.0 and rem_len <= tiny_thr:
            self.get_logger().warn(
                f"[supervisor] tiny residual {rem_len:.2f}m on {seg.sid} ({reason}) -> close without re-queue")
            self._append_cleanup_log({
                'type': 'tiny_residual_drop',
                'reason': str(reason),
                'seg_id': seg.sid,
                'cluster_id': int(seg.cluster_id),
                'remaining_length_m': float(rem_len),
                'time': time.time(),
            })
            self._cancel_follow()
            self._stop_cmd_vel()
            self._goal_handle = None
            self._send_future = None
            self._result_future = None
            self._cancel_future = None
            self._close_segment_without_revisit(idx, reason=f'tiny_residual:{reason}')
            self._active_seg_idx = None
            self._active_path = None
            self._mode_state = 'IDLE'
            self._follow_kind = 'SEGMENT'
            return

        if self._should_finish_close_active(reason, rem_len):
            self._cancel_follow()
            self._stop_cmd_vel()
            self._goal_handle = None
            self._send_future = None
            self._result_future = None
            self._cancel_future = None
            self._close_segment_without_revisit(idx, reason=f'endgame:{reason}')
            self._active_seg_idx = None
            self._active_path = None
            self._mode_state = 'IDLE'
            self._follow_kind = 'SEGMENT'
            return

        if self._should_close_active_on_defer(reason, rem_len):
            self._cancel_follow()
            self._stop_cmd_vel()
            self._goal_handle = None
            self._send_future = None
            self._result_future = None
            self._cancel_future = None
            self._close_segment_without_revisit(idx, reason=f'major_progress:{reason}')
            self._active_seg_idx = None
            self._active_path = None
            self._mode_state = 'IDLE'
            self._follow_kind = 'SEGMENT'
            return

        self._append_cleanup_log({
            'type': 'defer_segment',
            'reason': str(reason),
            'seg_id': seg.sid,
            'cluster_id': int(seg.cluster_id),
            'defer_count': int(self._defer_count[idx]),
            'time': time.time(),
        })
        self.get_logger().warn(f"[supervisor] defer seg {seg.sid} ({reason})")
        self._cancel_follow()
        self._stop_cmd_vel()
        # drop any pending follow futures to avoid mixing with new tasks
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._cancel_future = None
        self._queue_insert_defer(idx)
        self._active_seg_idx = None
        self._active_path = None
        self._mode_state = 'IDLE'
        self._follow_kind = 'SEGMENT'

    def _start_mode_c2(self, origin_pose_idx: Optional[int] = None) -> bool:
        if self._active_seg_idx is None:
            return False
        if origin_pose_idx is None:
            origin_pose_idx = self._current_progress_index()
        cands = self._c2_ordered_exit_candidates(int(origin_pose_idx))
        if not cands:
            return False
        self.get_logger().warn(
            f"[supervisor] C2 ordered-exit recovery on seg {self._segments[int(self._active_seg_idx)].sid} -> candidates={len(cands)}")
        self._start_detour('C2', cands, origin_pose_idx=int(origin_pose_idx))
        if self._should_prebackup_for_c(cands, int(origin_pose_idx)):
            self._start_short_backup(after_state='DET_COMPUTING', note='c2_prebackup')
        else:
            self._send_compute_for_candidate(cands[0])
        return True

    def _start_mode_c(self, bypass_cooldown: bool = False):
        if self._active_seg_idx is None:
            return
        if not bool(self.get_parameter('c_enable_slice').value):
            self._defer_active('C_disabled')
            return
        idx = int(self._active_seg_idx)

        # L2: per-segment cooldown to avoid C/C oscillation on noisy decisions
        now_mono = time.monotonic()
        cd = float(self.get_parameter('c_trigger_cooldown_sec').value)
        last = float(self._c_last_trigger_mono.get(idx, -1e9))
        if (not bypass_cooldown) and cd > 0.0 and (now_mono - last) < cd:
            self.get_logger().info(
                f"[supervisor] C suppressed by cooldown on seg {self._segments[idx].sid} ({cd:.1f}s)")
            return
        self._c_last_trigger_mono[idx] = now_mono
        self._last_mode_action_mono = now_mono

        self._c_slice_count[idx] = int(self._c_slice_count.get(idx, 0)) + 1
        if self._c_slice_count[idx] > int(self.get_parameter('c_max_attempts_per_segment').value):
            self.get_logger().warn(f"[supervisor] C attempts exceeded on seg {self._segments[idx].sid} -> use D")
            self._start_mode_d(force_cluster_jump=True, allow_cluster_jump=True, bypass_cooldown=True)
            return
        origin_i = self._current_progress_index()

        # C1: same-segment local rejoin only
        c1 = self._rejoin_candidates_on_active_segment('C1')
        if c1:
            self.get_logger().warn(
                f"[supervisor] C1 local rejoin on seg {self._segments[idx].sid} -> candidates={len(c1)}")
            self._start_detour('C1', c1, origin_pose_idx=origin_i)
            if self._should_prebackup_for_c(c1, origin_i):
                self._start_short_backup(after_state='DET_COMPUTING', note='c1_prebackup')
            else:
                self._send_compute_for_candidate(c1[0])
            return

        # C2: ordered exit recovery onto the next segment (start point + forward-ordered points)
        if self._start_mode_c2(origin_pose_idx=origin_i):
            return

        self.get_logger().warn("[supervisor] C1/C2: no candidates -> use D")
        self._publish_recovery_event('c_no_candidates', cluster_id=self._active_cluster_id())
        self._start_mode_d(force_cluster_jump=True, allow_cluster_jump=True, bypass_cooldown=True)

    def _start_mode_d(self, force_cluster_jump: bool = False, allow_cluster_jump: bool = True, bypass_cooldown: bool = False):
        if self._active_seg_idx is None:
            return
        idx = int(self._active_seg_idx)

        # L2: per-segment cooldown to avoid D/D oscillation (decision jitter, repeated abort callbacks, etc.)
        now_mono = time.monotonic()
        if (not bypass_cooldown) and (not force_cluster_jump):
            cd = float(self.get_parameter('d_trigger_cooldown_sec').value)
            last = float(self._d_last_trigger_mono.get(idx, -1e9))
            if cd > 0.0 and (now_mono - last) < cd:
                self.get_logger().info(
                    f"[supervisor] D suppressed by cooldown on seg {self._segments[idx].sid} ({cd:.1f}s)")
                return
        self._d_last_trigger_mono[idx] = now_mono
        self._last_mode_action_mono = now_mono

        self._d_recovery_count[idx] = int(self._d_recovery_count.get(idx, 0)) + 1
        if self._d_recovery_count[idx] > int(self.get_parameter('d_max_attempts_per_segment').value):
            self.get_logger().warn(f"[supervisor] D attempts exceeded on seg {self._segments[idx].sid} -> defer")
            self._defer_active('D_attempts_exceeded')
            return

        origin_i = self._current_progress_index()
        self._publish_recovery_event('d_request', cluster_id=self._active_cluster_id(), force_cluster_jump=bool(force_cluster_jump), allow_cluster_jump=bool(allow_cluster_jump))
        if self._maybe_use_d12_advisors(force_cluster_jump=force_cluster_jump, allow_cluster_jump=allow_cluster_jump):
            return

        cands: List[_DetourCandidate] = []
        if force_cluster_jump:
            # Cluster-only recovery
            if bool(self.get_parameter('d_enable_cluster_recovery').value):
                cands.extend(self._cluster_jump_candidates())
        else:
            # First try to rejoin further ahead on the SAME segment (preserves plan order)
            cands.extend(self._rejoin_candidates_on_active_segment('D'))
            # Only allow cluster jumps if explicitly allowed (or forced by upper logic)
            if allow_cluster_jump and bool(self.get_parameter('d_enable_cluster_recovery').value):
                cands.extend(self._cluster_jump_candidates())

        if not cands:
            if bool(self.get_parameter('d_turn_try_c2_before_defer').value) and self._seg_kind(idx) == 'turn':
                self.get_logger().warn('[supervisor] D: no candidates on turn -> try C2 ordered-exit before defer')
                self._publish_recovery_event('d_turn_fallback_c2', cluster_id=self._active_cluster_id(), seg_id=self._segments[idx].sid)
                if self._start_mode_c2(origin_pose_idx=origin_i):
                    return
            self.get_logger().warn('[supervisor] D: no candidates -> defer')
            self._publish_recovery_event('d_fail', cluster_id=self._active_cluster_id(), reason='D_no_candidates')
            self._defer_active('D_no_candidates')
            return

        self._start_detour('D', cands, origin_pose_idx=origin_i)
        self._send_compute_for_candidate(cands[0])

    def _handle_detour_exhausted(self):
        """No detour candidate worked."""
        reason = self._detour_reason
        origin = self._detour_origin_seg_idx
        seg_id = self._segments[origin].sid if origin is not None else 'n/a'
        self.get_logger().warn(f"[supervisor] detour exhausted ({reason}) origin={seg_id}")
        if self._is_c_reason(reason):
            self._publish_recovery_event('c_fail', cluster_id=self._active_cluster_id(), reason=str(reason), origin_seg=seg_id)
        elif str(reason) == 'D1':
            self._publish_recovery_event('d1_fail', cluster_id=self._active_cluster_id(), reason=str(reason), origin_seg=seg_id)
        else:
            self._publish_recovery_event('d_fail', cluster_id=self._active_cluster_id(), reason=str(reason), origin_seg=seg_id)

        # C1 exhausted -> try C2 ordered-exit recovery before escalating to D.
        if reason == 'C1':
            origin_pose_i = int(self._detour_origin_pose_idx)
            self._reset_detour_state()
            self._mode_state = 'FOLLOWING'  # temporary, so _start_mode_c2 can reuse the active segment context
            self._follow_kind = 'SEGMENT'
            if self._start_mode_c2(origin_pose_idx=origin_pose_i):
                return
            # fall through to D if C2 also has no candidates

        # For C/C2: escalate to D cluster-jump if enabled
        if self._is_c_reason(reason) and bool(self.get_parameter('d_enable_cluster_recovery').value):
            self._reset_detour_state()
            self._mode_state = 'FOLLOWING'  # temporary
            self._follow_kind = 'SEGMENT'
            self._start_mode_d(force_cluster_jump=True, allow_cluster_jump=True, bypass_cooldown=True)
            return

        # Otherwise: defer current active
        if self._active_seg_idx is not None:
            self._defer_active(f"detour_exhausted:{reason}")
        else:
            self._reset_detour_state()
            self._mode_state = 'IDLE'
            self._follow_kind = 'SEGMENT'

    def _on_detour_succeeded(self):
        cand = self._detour_active
        if cand is None:
            # nothing to resume, treat as done
            self.get_logger().warn('[supervisor] detour succeeded but no active candidate')
            self._reset_detour_state()
            self._active_seg_idx = None
            self._active_path = None
            self._mode_state = 'IDLE'
            self._follow_kind = 'SEGMENT'
            return

        # log skipped slices
        if cand.skipped:
            for seg_idx, a, b in cand.skipped:
                seg = self._segments[int(seg_idx)]
                self._append_cleanup_log({
                    'type': 'skipped_slice',
                    'seg_id': seg.sid,
                    'cluster_id': int(seg.cluster_id),
                    'start_pose_i': int(a),
                    'end_pose_i': int(b),
                    'reason': f"{self._detour_reason}:{cand.note}",
                    'time': time.time(),
                })

        # If this is a cluster jump (or cross-segment rejoin), decide whether to re-insert the origin segment.
        # Note: In Mode C + rejoin_next_segment, we often want to KEEP plan order and avoid retrying a persistently
        # blocked tail right away, so we support a policy knob to skip re-inserting the origin segment.
        if self._detour_origin_seg_idx is not None:
            origin_idx = int(self._detour_origin_seg_idx)
            # Avoid double-insert: if we resumed on the same segment, don't defer origin
            if origin_idx != int(cand.resume_seg_idx):
                if cand.note == 'cluster_jump' or cand.note == 'd1_route':
                    if self._should_close_origin_after_cross_seg():
                        evt_type = 'cluster_jump' if cand.note == 'cluster_jump' else 'd1_route_origin_close'
                        self._publish_recovery_event(evt_type, cluster_id=self._seg_cluster(origin_idx), seg_id=self._segments[origin_idx].sid)
                        self._close_segment_without_revisit(origin_idx, reason=f'{cand.note}:major_progress')
                    else:
                        self._queue_insert_defer(origin_idx)
                        self._set_cooldown(origin_idx)
                        evt_type = 'cluster_jump' if cand.note == 'cluster_jump' else 'd1_route_origin_defer'
                        self._publish_recovery_event(evt_type, cluster_id=self._seg_cluster(origin_idx), seg_id=self._segments[origin_idx].sid)
                        self.get_logger().info(
                            f"[supervisor] deferred origin seg {self._segments[origin_idx].sid} ({cand.note})")
                elif str(cand.note).startswith('rejoin_next_segment'):
                    revisit = bool(self.get_parameter('c_revisit_origin_on_rejoin_next_segment').value)
                    if self._is_c_reason(self._detour_reason) and (not revisit):
                        self.get_logger().info(
                            f"[supervisor] {self._detour_reason}: rejoin_next_segment -> not revisiting origin seg {self._segments[origin_idx].sid} (policy)")
                    elif self._should_close_origin_after_cross_seg():
                        self._close_segment_without_revisit(origin_idx, reason=f'{cand.note}:major_progress')
                    else:
                        self._queue_insert_defer(origin_idx)
                        self._set_cooldown(origin_idx)
                        self.get_logger().info(
                            f"[supervisor] deferred origin seg {self._segments[origin_idx].sid} ({cand.note})")
                elif str(cand.note).startswith('d2_plan'):
                    self._close_segment_without_revisit(origin_idx, reason=f'{cand.note}:replace')
                    if self._pending_d2_scope_supersede:
                        self._retire_scope_segments_after_d2(
                            set(self._pending_d2_scope_supersede),
                            reason=f'{cand.note}:scope_replace')
                    self.get_logger().info(
                        f"[supervisor] D2 plan resume -> not revisiting origin seg {self._segments[origin_idx].sid}")
        # Remove resume segment from queue if present (because we will execute it now)
        if int(cand.resume_seg_idx) in self._queue:
            try:
                self._queue.remove(int(cand.resume_seg_idx))
            except ValueError:
                pass

        # Resume coverage
        resume_idx = int(cand.resume_seg_idx)
        resume_pose_i = int(cand.resume_pose_idx)
        self._active_seg_idx = resume_idx
        self._mark_main_pass_frontier(resume_idx)
        # L2: recovery succeeded -> clear consecutive FollowPath failure counter for the resumed segment
        self._fail_count[resume_idx] = 0
        seg = self._segments[resume_idx]
        stamp = self.get_clock().now().to_msg()
        full = self._path_from_segment(seg, stamp)
        sliced = self._slice_path(full, resume_pose_i)
        # (optional) trim again to robot to reduce initial tracking error
        if bool(self.get_parameter('enable_trim_on_segment_start').value):
            trimmed = self._trim_path_to_robot(sliced, stamp)
            min_remain = int(self.get_parameter('min_remaining_poses_after_trim').value)
            path = trimmed if len(trimmed.poses) >= max(2, min_remain) else sliced
        else:
            path = sliced

        tiny_thr = self._tiny_residual_threshold(resume_idx)
        try:
            rem_len = float(self._path_length_from(path, 0))
        except Exception:
            rem_len = -1.0
        if rem_len >= 0.0 and tiny_thr > 0.0 and rem_len <= tiny_thr:
            self.get_logger().warn(
                f"[supervisor] tiny residual {rem_len:.2f}m on {seg.sid} after detour resume -> close without follow")
            self._append_cleanup_log({
                'type': 'tiny_residual_resume_drop',
                'reason': f"{self._detour_reason}:{cand.note}",
                'seg_id': seg.sid,
                'cluster_id': int(seg.cluster_id),
                'remaining_length_m': float(rem_len),
                'time': time.time(),
            })
            self._close_segment_without_revisit(resume_idx, reason=f'tiny_residual_resume:{self._detour_reason}:{cand.note}')
            self._reset_detour_state()
            self._active_seg_idx = None
            self._active_path = None
            self._mode_state = 'IDLE'
            self._follow_kind = 'SEGMENT'
            return

        self._reset_detour_state()
        self._start_segment_follow(path, int(resume_idx))

    def _handle_segment_follow_failure(self, status: int):
        if self._active_seg_idx is None:
            return
        idx = int(self._active_seg_idx)
        seg = self._segments[idx]
        self._fail_count[idx] = int(self._fail_count.get(idx, 0)) + 1
        fail_n = int(self._fail_count[idx])
        self.get_logger().warn(f"[supervisor] FollowPath failed seg={seg.sid} status={status} fail_n={fail_n}")
        if int(status) == GoalStatus.STATUS_ABORTED:
            self._publish_recovery_event('follow_aborted', cluster_id=int(seg.cluster_id), seg_id=seg.sid, fail_n=int(fail_n))

        if self._no_live_obstacle_evidence_for(float(self.get_parameter('stale_shadow_no_live_sec').value)):
            self.get_logger().warn('[supervisor] follow failure with no live obstacle evidence -> clear stale shadows + retry once')
            self._maybe_stale_shadow_cleanup(f'follow_fail_{status}')
            if fail_n <= 2:
                stamp = self.get_clock().now().to_msg()
                full = self._path_from_segment(seg, stamp)
                trimmed = self._trim_path_to_robot(full, stamp)
                min_remain = int(self.get_parameter('min_remaining_poses_after_trim').value)
                path = trimmed if len(trimmed.poses) >= max(2, min_remain) else full
                self._active_path = path
                self._pub_path.publish(path)
                self._send_follow(path)
                self._follow_kind = 'SEGMENT'
                self._mode_state = 'FOLLOWING'
                self._publish_recovery_event('stale_shadow_retry', cluster_id=int(seg.cluster_id), seg_id=seg.sid, status=int(status), fail_n=int(fail_n))
                return

        # ABORTED(6): prefer local C-recovery first, especially on turn / short segments.
        if status == GoalStatus.STATUS_ABORTED:
            now_mono = time.monotonic()
            cd = float(self.get_parameter('abort_replan_cooldown_sec').value)
            last = float(self._abort_last_replan_mono.get(idx, -1e9))
            if cd <= 0.0 or (now_mono - last) >= cd:
                self._abort_last_replan_mono[idx] = now_mono

                if bool(self.get_parameter('abort_use_c_recovery_first').value):
                    remaining_len = self._active_remaining_length()
                    short_len = float(self.get_parameter('abort_short_segment_length_m').value)
                    escalate_after = max(1, int(self.get_parameter('abort_c_escalate_after_failures').value))
                    prefer_c2 = bool(self.get_parameter('abort_turn_prefer_c2').value) and (
                        self._seg_kind(idx) == 'turn' or (remaining_len > 0.0 and remaining_len <= short_len)
                    )

                    if fail_n < escalate_after:
                        if prefer_c2:
                            self.get_logger().warn(
                                f"[supervisor] ABORTED(status=6) seg={seg.sid} -> try C2 exit-recovery first "
                                f"(fail_n={fail_n}, remaining={remaining_len:.2f}m)")
                            if self._start_mode_c2(origin_pose_idx=self._current_progress_index()):
                                return
                            # If C2 has no candidates, fall through to D on this tick.
                            self.get_logger().warn(
                                f"[supervisor] ABORTED seg={seg.sid}: C2 had no candidates -> escalate to D")
                        else:
                            self.get_logger().warn(
                                f"[supervisor] ABORTED(status=6) seg={seg.sid} -> try C1/C2 local recovery first "
                                f"(fail_n={fail_n}, remaining={remaining_len:.2f}m)")
                            self._start_mode_c(bypass_cooldown=True)
                            return

                    after_n = int(self.get_parameter('abort_cluster_jump_after_failures').value)
                    allow_cluster = bool(fail_n >= max(1, after_n))
                    self.get_logger().warn(
                        f"[supervisor] ABORTED(status=6) seg={seg.sid} -> escalate to D_REPLAN "
                        f"(fail_n={fail_n}, allow_cluster_jump={allow_cluster})")
                    self._start_mode_d(force_cluster_jump=False, allow_cluster_jump=allow_cluster, bypass_cooldown=True)
                    return

                if bool(self.get_parameter('abort_use_d_replan_first').value):
                    after_n = int(self.get_parameter('abort_cluster_jump_after_failures').value)
                    allow_cluster = bool(fail_n >= max(1, after_n))
                    self.get_logger().warn(
                        f"[supervisor] ABORTED(status=6) seg={seg.sid} -> try D_REPLAN before defer "
                        f"(fail_n={fail_n}, allow_cluster_jump={allow_cluster})")
                    self._start_mode_d(force_cluster_jump=False, allow_cluster_jump=allow_cluster, bypass_cooldown=True)
                    return
            else:
                self.get_logger().warn(
                    f"[supervisor] ABORTED seg={seg.sid} but abort recovery cooldown active ({cd:.1f}s) -> fall back")

        # One retry with trimmed path
        if fail_n <= 1 and bool(self.get_parameter('enable_trim_on_segment_start').value):
            stamp = self.get_clock().now().to_msg()
            full = self._path_from_segment(seg, stamp)
            trimmed = self._trim_path_to_robot(full, stamp)
            min_remain = int(self.get_parameter('min_remaining_poses_after_trim').value)
            if len(trimmed.poses) >= max(2, min_remain):
                self.get_logger().info(f"[supervisor] retry seg={seg.sid} with trimmed path")
                self._active_path = trimmed
                self._pub_path.publish(trimmed)
                self._send_follow(trimmed)
                self._follow_kind = 'SEGMENT'
                self._mode_state = 'FOLLOWING'
                return

        # if too many failures overall -> skip
        max_fail = int(self.get_parameter('skip_after_failures').value)
        if fail_n >= max_fail:
            self._start_skip_active(reason=f"follow_fail_{status}_x{fail_n}")
            return

        self._defer_active(reason=f"follow_fail_{status}_x{fail_n}")

    def _send_follow(self, path: Path):
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ""
        goal.goal_checker_id = ""
        self._send_future = self._follow.send_goal_async(goal)
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None

    def _cancel_follow(self):
        if self._goal_handle is None:
            return
        self._cancel_future = self._goal_handle.cancel_goal_async()

    def _publish_speed_limit(self, ratio: float):
        # nav2_msgs/SpeedLimit: percentage(bool) + speed_limit(float64)
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.percentage = True
        msg.speed_limit = float(max(0.05, min(1.0, ratio)) * 100.0)  # percent
        self._pub_speed.publish(msg)

    def _clear_speed_limit(self):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.percentage = True
        msg.speed_limit = 0.0  # 0 means "no limit"
        self._pub_speed.publish(msg)

    # -------- Mode handlers --------
    def _mode_from_decision(self) -> str:
        if self._decision is None:
            return "NONE"
        return str(self._decision.decision.mode)

    def _tick(self):
        # Wait for required action servers
        if not self._ensure_servers():
            return

        # Active observe (publish cmd_vel pattern)
        if self._mode_state == 'OBSERVING':
            self._tick_observe()
            return

        # Turn pre-align skill (short rotate-in-place before following a turn connector)
        if self._mode_state == 'TURN_PREALIGN':
            self._tick_turn_prealign()
            return

        # Detour compute pipeline (ComputePathToPose)
        if self._mode_state == 'DET_COMPUTING':
            self._tick_detour_compute()
            return

        # Start next segment if idle
        if self._active_seg_idx is None:
            idx = self._pop_next_ready()
            if idx is None:
                self._announce_mission_complete_if_idle()
                return
            self._mission_complete_announced = False
            self._active_seg_idx = int(idx)
            self._mark_main_pass_frontier(int(idx))
            self._fail_count[int(idx)] = 0

            seg = self._segments[int(idx)]
            stamp = self.get_clock().now().to_msg()
            full = self._path_from_segment(seg, stamp)
            if bool(self.get_parameter('enable_trim_on_segment_start').value):
                trimmed = self._trim_path_to_robot(full, stamp)
                min_remain = int(self.get_parameter('min_remaining_poses_after_trim').value)
                path = trimmed if len(trimmed.poses) >= max(2, min_remain) else full
            else:
                path = full

            tiny_thr = self._tiny_residual_threshold(int(idx))
            try:
                rem_len = float(self._path_length_from(path, 0))
            except Exception:
                rem_len = -1.0
            if rem_len >= 0.0 and tiny_thr > 0.0 and rem_len <= tiny_thr:
                self.get_logger().warn(
                    f"[supervisor] tiny residual {rem_len:.2f}m on {seg.sid} at activation -> close without follow")
                self._append_cleanup_log({
                    'type': 'tiny_residual_start_drop',
                    'reason': 'activation_trim',
                    'seg_id': seg.sid,
                    'cluster_id': int(seg.cluster_id),
                    'remaining_length_m': float(rem_len),
                    'time': time.time(),
                })
                self._close_segment_without_revisit(int(idx), reason='tiny_residual_activation')
                self._active_seg_idx = None
                self._active_path = None
                self._mode_state = 'IDLE'
                self._follow_kind = 'SEGMENT'
                self._action_wd_started_t = 0.0
                self._action_wd_last_progress_t = 0.0
                self._action_wd_last_xy = None
                return

            self._start_segment_follow(path, int(idx))
            return

        # FollowPath goal response
        if self._send_future is not None and self._send_future.done():
            gh = self._send_future.result()
            self._send_future = None
            if gh is None or not gh.accepted:
                self.get_logger().warn(f"[supervisor] FollowPath goal rejected kind={self._follow_kind}")
                if self._follow_kind == 'DETOUR':
                    self._advance_detour_candidate('follow_goal_rejected')
                    if self._detour_exhausted():
                        self._handle_detour_exhausted()
                    else:
                        self._mode_state = 'DET_COMPUTING'
                    return
                # segment follow rejected
                self._handle_segment_follow_failure(status=-2)
                return

            self._goal_handle = gh
            self._result_future = gh.get_result_async()

            if self._follow_kind == 'DETOUR':
                self._init_detour_watchdog()
            else:
                self._init_action_watchdog()

        # D2 lab hook: allow one deterministic forced D2 request during normal segment following,
        # without waiting for live C/D escalation. This is only active when explicitly enabled.
        if self._mode_state == 'FOLLOWING' and self._follow_kind == 'SEGMENT' and self._goal_handle is not None:
            if self._maybe_force_d2_test_once():
                return

        # For C1/C2 detours, try a lighter-weight 'recompute same target' refresh before the
        # long watchdog timeout would force a heavier recovery.
        self._tick_c_detour_refresh()

        # Detour watchdog tick (abort stuck detours early)
        self._tick_detour_watchdog()
        # Nominal segment watchdog (completion-first anti-stall governor)
        self._tick_action_watchdog()

        # Combined speed-limit overlay:
        #   - Mode B (yield) stays globally active, including during detours.
        #   - Planned 'turn' segments are slowed down a bit to reduce MPPI corner-cutting
        #     near fixed obstacles and make RotationShim engagement smoother.
        speed_ratios: List[float] = []
        if self._decision is not None and str(self._decision.decision.mode) == 'B_YIELD':
            speed_ratios.append(max(0.1, min(1.0, float(self._decision.decision.slowdown_ratio))))
        if bool(self.get_parameter('turn_segment_enable_speed_limit').value):
            if self._follow_kind == 'SEGMENT' and self._mode_state == 'FOLLOWING' and self._active_seg_idx is not None:
                if self._seg_kind(self._active_seg_idx) == 'turn':
                    speed_ratios.append(max(0.1, min(1.0, float(self.get_parameter('turn_segment_speed_limit_ratio').value))))
        if speed_ratios:
            self._publish_speed_limit(min(speed_ratios))
        else:
            self._clear_speed_limit()

        # Waiting logic (Mode A)
        if self._mode_state == 'WAITING':
            now = self.get_clock().now()
            if (now - self._wait_started_t).nanoseconds * 1e-9 > float(self.get_parameter('wait_max_sec').value):
                self.get_logger().warn('[supervisor] wait timeout -> defer')
                self._defer_active('wait_timeout')
                return

            force_resume = self._no_live_obstacle_evidence_for(float(self.get_parameter('stale_shadow_no_live_sec').value))
            if force_resume:
                self._maybe_stale_shadow_cleanup('wait_resume')
            if force_resume or self._decision is None or str(self._decision.decision.mode) in ('NONE', 'B_YIELD'):
                if self._clear_ok_since is None:
                    self._clear_ok_since = now
                hold = float(self.get_parameter('wait_clear_hold_sec').value)
                if (now - self._clear_ok_since).nanoseconds * 1e-9 >= hold:
                    # Resume the currently active path (segment OR detour).
                    # This makes Mode A safe to use even if we entered it during a detour follow.
                    if self._active_path is None:
                        # Fallback: rebuild from active segment if possible
                        if self._active_seg_idx is None:
                            self._mode_state = 'IDLE'
                            return
                        seg = self._segments[int(self._active_seg_idx)]
                        stamp = now.to_msg()
                        path = self._path_from_segment(seg, stamp)
                    else:
                        stamp = now.to_msg()
                        path = self._active_path
                    path = self._trim_path_to_robot(path, stamp)
                    path.header.stamp = stamp
                    self._active_path = path
                    self._follow_kind = str(getattr(self, '_wait_resume_kind', self._follow_kind))

                    # If a moving obstacle interrupted a C1/C2 detour (A_WAIT), do not blindly resume the
                    # old detour polyline; refresh the path to the SAME current rejoin target first.
                    if (self._follow_kind == 'DETOUR'
                            and bool(self.get_parameter('c_wait_resume_recompute').value)
                            and self._is_c_reason(self._detour_reason)
                            and self._detour_cand_i < len(self._detour_candidates)):
                        self.get_logger().info('[supervisor] wait-resume on C detour -> recompute current target')
                        now_mono = time.monotonic()
                        self._compute_future = None
                        self._compute_goal_handle = None
                        self._compute_result_future = None
                        self._detour_active = None
                        self._mode_state = 'DET_COMPUTING'
                        self._detour_stop_hold_until = now_mono + max(0.0, float(self.get_parameter('detour_stop_hold_sec').value))
                        self._detour_delay_until = now_mono + max(0.0, float(self.get_parameter('detour_delay_compute_sec').value))
                        self._clear_ok_since = None
                        return

                    if self._follow_kind == 'SEGMENT' and self._active_seg_idx is not None:
                        self._start_segment_follow(path, int(self._active_seg_idx))
                    else:
                        self._pub_path.publish(path)
                        self._mode_state = 'FOLLOWING'
                        self._send_follow(path)
            else:
                self._clear_ok_since = None
            return


        # Decision handling
        # - A_WAIT is treated as a *global* pause (works during SEGMENT or DETOUR follows).
        # - B_YIELD is handled as a speed-limit overlay earlier.
        # - C/D/E are only meaningful while following a coverage segment (SEGMENT).
        if self._decision is not None and self._mode_state == 'FOLLOWING':
            mode = str(self._decision.decision.mode)
            now_mono = time.monotonic()
            if bool(self.get_parameter('require_live_evidence_for_blocking_modes').value) and mode in ('A_WAIT', 'C_DEFER', 'D_REPLAN'):
                if self._no_live_obstacle_evidence_for(float(self.get_parameter('stale_shadow_no_live_sec').value)):
                    self._maybe_stale_shadow_cleanup(f'ignore_{mode}')
                    self.get_logger().warn(f"[supervisor] ignore {mode}: no live obstacle evidence")
                    mode = 'NONE'

            # A: wait / pause (global, always-preemptive)
            if mode == 'A_WAIT':
                self.get_logger().info('[supervisor] Mode A_WAIT: cancel FollowPath and wait')
                self._wait_resume_kind = self._follow_kind
                self._cancel_follow()
                self._stop_cmd_vel()
                # drop pending follow futures to avoid mixing
                self._goal_handle = None
                self._send_future = None
                self._result_future = None
                self._cancel_future = None
                self._action_wd_started_t = 0.0
                self._action_wd_last_progress_t = 0.0
                self._action_wd_last_xy = None
                self._mode_state = 'WAITING'
                self._wait_started_t = self.get_clock().now()
                self._clear_ok_since = None
                self._last_mode_action_mono = now_mono
                return

            # For non-safety modes: require the decision to be stable + enforce a min interval (avoid thrashing)
            stable_sec = float(self.get_parameter('decision_stable_sec').value)
            min_interval = float(self.get_parameter('mode_min_interval_sec').value)

            if stable_sec > 0.0 and (now_mono - float(getattr(self, '_decision_mode_since_mono', now_mono))) < stable_sec:
                # Too jittery; ignore this tick.
                pass
            elif min_interval > 0.0 and (now_mono - float(getattr(self, '_last_mode_action_mono', 0.0))) < min_interval:
                pass
            else:
                # Segment-only modes
                if self._follow_kind == 'SEGMENT':
                    idx = int(self._active_seg_idx) if self._active_seg_idx is not None else -1

                    # C: slice+rejoin
                    if mode == 'C_DEFER':
                        self.get_logger().warn('[supervisor] Mode C_DEFER -> slice+rejoin')
                        self._start_mode_c()
                        return

                    # D: detour / recovery
                    if mode == 'D_REPLAN':
                        # Policy: when selector asks for D, try C first (keeps plan order and avoids unnecessary cluster jumps)
                        bypass_c_first = bool(self.get_parameter('d_bypass_c_if_region_hard').value) and self._region_prefers_structural_d()
                        if bool(self.get_parameter('d_requires_c_first').value) and (not bypass_c_first):
                            if idx >= 0 and bool(self.get_parameter('c_enable_slice').value):
                                max_c = int(self.get_parameter('c_max_attempts_per_segment').value)
                                if int(self._c_slice_count.get(idx, 0)) < max_c:
                                    self.get_logger().warn(
                                        '[supervisor] Mode D_REPLAN requested; d_requires_c_first -> try C first')
                                    self._start_mode_c()
                                    return
                        elif bypass_c_first:
                            self.get_logger().warn('[supervisor] Mode D_REPLAN requested; region evidence bypasses C-first -> use D')

                        self.get_logger().warn('[supervisor] Mode D_REPLAN -> detour/recovery')
                        # Default: keep plan order (no cluster jump here; forced D from C will enable it)
                        self._start_mode_d(force_cluster_jump=False, allow_cluster_jump=False)
                        return

                    # E: skip
                    if mode == 'E_SKIP':
                        self._last_mode_action_mono = now_mono
                        self._start_skip_active('E_SKIP')
                        return

        # FollowPath result
        if self._result_future is not None and self._result_future.done():
            res = self._result_future.result()
            self._result_future = None
            status = int(res.status) if res is not None else -1
            # Extra diagnostics for ABORTED storms (Nav2 FollowPath exposes error_code / error_msg)
            err_code = 0
            err_msg = ''
            try:
                err_code = int(getattr(getattr(res, 'result', None), 'error_code', 0))
                err_msg = str(getattr(getattr(res, 'result', None), 'error_msg', ''))
            except Exception:
                err_code = 0
                err_msg = ''
            if status != GoalStatus.STATUS_SUCCEEDED and (err_code != 0 or err_msg):
                self.get_logger().warn(
                    f"[supervisor] FollowPath result status={status} error_code={err_code} msg={err_msg}")


            if status == GoalStatus.STATUS_SUCCEEDED:
                if self._follow_kind == 'DETOUR':
                    self.get_logger().info('[supervisor] detour succeeded')
                    self._on_detour_succeeded()
                    return

                # segment done
                idx = int(self._active_seg_idx)
                seg = self._segments[idx]
                self.get_logger().info(f"[supervisor] seg {seg.sid} done")
                self._publish_recovery_event('segment_done', cluster_id=int(seg.cluster_id), seg_id=seg.sid)
                self._active_seg_idx = None
                self._active_path = None
                self._mode_state = 'IDLE'
                self._follow_kind = 'SEGMENT'
                self._action_wd_started_t = 0.0
                self._action_wd_last_progress_t = 0.0
                self._action_wd_last_xy = None
                return

            # non-success
            if self._follow_kind == 'DETOUR':
                self._advance_detour_candidate(f"follow_failed_{status}")
                if self._detour_exhausted():
                    self._handle_detour_exhausted()
                else:
                    self._mode_state = 'DET_COMPUTING'
                return

            self._handle_segment_follow_failure(status)
            return


def main():
    rclpy.init()
    n = CoverageBTSupervisor()
    exec_ = MultiThreadedExecutor(num_threads=4)
    exec_.add_node(n)
    try:
        exec_.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        exec_.shutdown()
        n.destroy_node()
        rclpy.shutdown()
