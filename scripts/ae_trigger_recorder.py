#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ae_trigger_recorder.py

只记录 A_WAIT / B_YIELD / C_DEFER / D_REPLAN / E_SKIP 触发事件（不记录 NONE），并在触发瞬间
抓取关键上下文 topic 的“近似同步快照”，写入 JSONL。

可选：触发时录制一个短 rosbag2 clip（只录关键 topics），便于后续回放定位：
  --bag --bag-seconds 6

用法示例：
  source /opt/ros/humble/setup.bash
  source ~/coverage_ws_farm_gate/install/setup.bash
  python3 ~/coverage_ws_farm_gate/scripts/ae_trigger_recorder.py \
      --out-dir ~/.ros/ae_events --tag "caseA_selectorA" --bag --bag-seconds 6

输出：
  ~/.ros/ae_events/<run_id>/events.jsonl
  ~/.ros/ae_events/<run_id>/bags/<event_id>/  (如果启用 --bag)

注意：
- 本脚本默认使用 use_sim_time（建议用 --ros-args -p use_sim_time:=True 或在节点参数里设置）
- 不会主动“停机器人”，只做记录；你要验证闭环就看事件里 cmd_vel 是否接近 0。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from rclpy.exceptions import ParameterAlreadyDeclaredException

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray

from nav2_msgs.msg import SpeedLimit

from tf2_ros import Buffer, TransformListener, TransformException

from coverage_msgs.msg import StrategyDecisionStamped, ObstaclePredictionArray, ObstacleTrackArray


TRIGGER_MODES = {"A_WAIT", "B_YIELD", "C_DEFER", "D_REPLAN", "E_SKIP"}

MODE_TO_EXEC_ACTION = {
    "A_WAIT": "Supervisor: cancel FollowPath -> WAITING; clear后裁剪当前segment并继续FollowPath",
    "B_YIELD": "Supervisor: 发布 /speed_limit 降速；不cancel FollowPath",
    "C_DEFER": "Supervisor: cancel FollowPath -> defer当前segment到队尾（次数超限则skip）",
    "D_REPLAN": "Supervisor: cancel FollowPath -> 调 /compute_path_to_pose 规划到 rejoin_pose -> FollowPath detour",
    "E_SKIP": "Supervisor: cancel FollowPath -> 直接跳过当前segment",
}


def stamp_to_sec(st) -> float:
    return float(st.sec) + float(st.nanosec) * 1e-9


def now_wall() -> float:
    return time.time()


def yaw_from_quat(q) -> float:
    # planar yaw
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def finite_min(vals: List[float]) -> Optional[float]:
    m = None
    for v in vals:
        if not math.isfinite(v):
            continue
        if m is None or v < m:
            m = v
    return m


@dataclass
class BufItem:
    t_sec: float          # message stamp in ROS time (sec)
    wall_sec: float       # wall receive time
    msg: Any


class MsgBuffer:
    """Ring buffer storing the last N messages (by arrival order)."""

    def __init__(self, maxlen: int = 200):
        self.buf: Deque[BufItem] = deque(maxlen=maxlen)

    def push(self, t_sec: float, msg: Any):
        self.buf.append(BufItem(t_sec=t_sec, wall_sec=now_wall(), msg=msg))

    def nearest(self, t_ref: float) -> Optional[BufItem]:
        if not self.buf:
            return None
        return min(self.buf, key=lambda it: abs(it.t_sec - t_ref))

    def latest(self) -> Optional[BufItem]:
        return self.buf[-1] if self.buf else None


class BagRecorder:
    """Spawn rosbag2 record for a short duration."""

    def __init__(self):
        self._lock = threading.Lock()
        self._busy = False

    def record_clip(self, out_dir: str, topics: List[str], seconds: float) -> bool:
        with self._lock:
            if self._busy:
                return False
            self._busy = True

        def _worker():
            try:
                # IMPORTANT: don't pre-create out_dir; otherwise ros2 bag writes to out_dir_0 and out_dir stays empty.
                if os.path.exists(out_dir):
                    shutil.rmtree(out_dir)
                cmd = ["ros2", "bag", "record", "-o", out_dir] + topics
                proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                t0 = now_wall()
                while now_wall() - t0 < seconds:
                    if proc.poll() is not None:
                        break
                    time.sleep(0.1)

                if proc.poll() is None:
                    # Graceful stop
                    proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        proc.terminate()
                        try:
                            proc.wait(timeout=2.0)
                        except subprocess.TimeoutExpired:
                            proc.kill()
            finally:
                with self._lock:
                    self._busy = False

        threading.Thread(target=_worker, daemon=True).start()
        return True


class AETriggerRecorder(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("ae_trigger_recorder")

        # allow use_sim_time param
        try:
            self.declare_parameter("use_sim_time", True)
        except ParameterAlreadyDeclaredException:
            pass

        self.args = args
        self.run_id = time.strftime("%Y%m%d_%H%M%S")
        self.base_dir = os.path.join(os.path.expanduser(args.out_dir), self.run_id)
        os.makedirs(self.base_dir, exist_ok=True)

        self.jsonl_path = os.path.join(self.base_dir, "events.jsonl")
        self._jsonl_fp = open(self.jsonl_path, "a", encoding="utf-8")

        self.get_logger().info(f"[recorder] run_id={self.run_id}")
        self.get_logger().info(f"[recorder] jsonl={self.jsonl_path}")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # buffers
        self.buf_scan = MsgBuffer(maxlen=args.buf_len)
        self.buf_path = MsgBuffer(maxlen=args.buf_len)
        self.buf_tracks = MsgBuffer(maxlen=args.buf_len)
        self.buf_preds = MsgBuffer(maxlen=args.buf_len)
        self.buf_odom = MsgBuffer(maxlen=args.buf_len)
        self.buf_cmd = MsgBuffer(maxlen=args.buf_len)
        self.buf_speed = MsgBuffer(maxlen=args.buf_len)
        self.buf_follow_status = MsgBuffer(maxlen=args.buf_len)
        self.buf_compute_status = MsgBuffer(maxlen=args.buf_len)

        # mode state
        self.last_mode: str = "NONE"
        self.last_event_wall: float = 0.0

        # bag
        self.bag = BagRecorder()
        # Bag throttling to avoid overlapping rosbag processes
        self._last_bag_wall = 0.0

        # QoS profiles
        qos_default = QoSProfile(depth=50)
        qos_default.reliability = ReliabilityPolicy.RELIABLE

        qos_tf_static = QoSProfile(depth=1)
        qos_tf_static.reliability = ReliabilityPolicy.RELIABLE
        qos_tf_static.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Subscribers
        self.create_subscription(
            StrategyDecisionStamped,
            args.decision_topic,
            self.on_decision,
            qos_default,
        )
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, args.path_topic, self.on_path, qos_default)
        self.create_subscription(ObstacleTrackArray, args.tracks_topic, self.on_tracks, qos_default)
        self.create_subscription(ObstaclePredictionArray, args.preds_topic, self.on_preds, qos_default)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, args.cmd_vel_topic, self.on_cmd, qos_default)
        self.create_subscription(SpeedLimit, args.speed_limit_topic, self.on_speed_limit, qos_default)

        # Action status topics (optional but very helpful)
        self.create_subscription(GoalStatusArray, f"{args.follow_action}/_action/status", self.on_follow_status, qos_default)
        self.create_subscription(GoalStatusArray, f"{args.compute_path_action}/_action/status", self.on_compute_status, qos_default)

        self.get_logger().info("[recorder] subscribed topics:")
        self.get_logger().info(f"  decision: {args.decision_topic}")
        self.get_logger().info(f"  scan:     {args.scan_topic}")
        self.get_logger().info(f"  path:     {args.path_topic}")
        self.get_logger().info(f"  tracks:   {args.tracks_topic}")
        self.get_logger().info(f"  preds:    {args.preds_topic}")
        self.get_logger().info(f"  odom:     {args.odom_topic}")
        self.get_logger().info(f"  cmd_vel:  {args.cmd_vel_topic}")
        self.get_logger().info(f"  speed:    {args.speed_limit_topic}")
        self.get_logger().info(f"  follow status:  {args.follow_action}/_action/status")
        self.get_logger().info(f"  compute status: {args.compute_path_action}/_action/status")

    # ---- callbacks for buffers ----

    def now_ros_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def on_scan(self, msg: LaserScan):
        self.buf_scan.push(stamp_to_sec(msg.header.stamp), msg)

    def on_path(self, msg: Path):
        self.buf_path.push(stamp_to_sec(msg.header.stamp), msg)

    def on_tracks(self, msg: ObstacleTrackArray):
        self.buf_tracks.push(stamp_to_sec(msg.header.stamp), msg)

    def on_preds(self, msg: ObstaclePredictionArray):
        self.buf_preds.push(stamp_to_sec(msg.header.stamp), msg)

    def on_odom(self, msg: Odometry):
        self.buf_odom.push(stamp_to_sec(msg.header.stamp), msg)

    def on_cmd(self, msg: Twist):
        # Twist 没 header，用当前 ROS time 作为“时间戳”
        self.buf_cmd.push(self.now_ros_sec(), msg)

    def on_speed_limit(self, msg: SpeedLimit):
        self.buf_speed.push(stamp_to_sec(msg.header.stamp), msg)

    def on_follow_status(self, msg: GoalStatusArray):
        self.buf_follow_status.push(self.now_ros_sec(), msg)

    def on_compute_status(self, msg: GoalStatusArray):
        self.buf_compute_status.push(self.now_ros_sec(), msg)

    # ---- snapshot utils ----

    def tf_snapshot(self, target: str, source: str, use_latest: bool = True) -> Dict[str, Any]:
        try:
            if use_latest:
                tr = self.tf_buffer.lookup_transform(target, source, Time())
            else:
                tr = self.tf_buffer.lookup_transform(target, source, Time())
            x = float(tr.transform.translation.x)
            y = float(tr.transform.translation.y)
            yaw = yaw_from_quat(tr.transform.rotation)
            return {"ok": True, "target": target, "source": source, "x": x, "y": y, "yaw": yaw}
        except TransformException as e:
            return {"ok": False, "target": target, "source": source, "err": str(e)}
        except Exception as e:
            return {"ok": False, "target": target, "source": source, "err": f"{type(e).__name__}: {e}"}

    def scan_features(self, scan: LaserScan) -> Dict[str, Any]:
        # 前方扇区 [-20deg, +20deg] 的最小距离
        ang0 = 0.0
        half = math.radians(20.0)
        a_min = float(scan.angle_min)
        a_inc = float(scan.angle_increment)
        n = len(scan.ranges)

        def idx_of(a: float) -> int:
            return int(round((a - a_min) / max(1e-9, a_inc)))

        i0 = max(0, min(n - 1, idx_of(ang0 - half)))
        i1 = max(0, min(n - 1, idx_of(ang0 + half)))
        if i0 > i1:
            i0, i1 = i1, i0

        front = [float(r) for r in scan.ranges[i0 : i1 + 1]]
        front_min = finite_min(front)
        all_min = finite_min([float(r) for r in scan.ranges])

        return {
            "front_min_m": front_min,
            "all_min_m": all_min,
            "front_sector_deg": 40.0,
            "n_ranges": n,
        }

    def path_summary(self, path: Path, max_pts: int, decimate: int) -> Dict[str, Any]:
        pts = []
        poses = path.poses
        if not poses:
            return {"poses": 0, "frame_id": path.header.frame_id, "stamp": stamp_to_sec(path.header.stamp), "points_xy": []}

        step = max(1, int(decimate))
        for i in range(0, len(poses), step):
            p = poses[i].pose.position
            pts.append([float(p.x), float(p.y)])
            if len(pts) >= max_pts:
                break

        p0 = poses[0].pose.position
        p1 = poses[-1].pose.position
        return {
            "poses": len(poses),
            "frame_id": path.header.frame_id,
            "stamp": stamp_to_sec(path.header.stamp),
            "first_xy": [float(p0.x), float(p0.y)],
            "last_xy": [float(p1.x), float(p1.y)],
            "points_xy": pts,
        }

    def tracks_summary(self, tr: ObstacleTrackArray, max_items: int) -> Dict[str, Any]:
        out = {
            "stamp": stamp_to_sec(tr.header.stamp),
            "frame_id": tr.header.frame_id,
            "n": len(tr.tracks),
            "tracks": [],
        }
        for t in tr.tracks[:max_items]:
            out["tracks"].append(
                {
                    "id": t.id,
                    "x": float(t.pose.position.x),
                    "y": float(t.pose.position.y),
                    "vx": float(t.velocity.linear.x),
                    "vy": float(t.velocity.linear.y),
                    "radius": float(t.radius),
                    "confidence": float(t.confidence),
                    "cls": int(t.cls),
                }
            )
        return out

    def preds_summary(self, pr: ObstaclePredictionArray, max_items: int, max_pts: int, decimate: int) -> Dict[str, Any]:
        out = {
            "stamp": stamp_to_sec(pr.header.stamp),
            "frame_id": pr.header.frame_id,
            "n": len(pr.predictions),
            "predictions": [],
        }
        step = max(1, int(decimate))
        for p in pr.predictions[:max_items]:
            pts = []
            for i in range(0, len(p.points), step):
                pt = p.points[i]
                pts.append([float(pt.x), float(pt.y)])
                if len(pts) >= max_pts:
                    break
            out["predictions"].append(
                {
                    "id": p.id,
                    "dt": float(p.dt),
                    "confidence": float(p.confidence),
                    "points_xy": pts,
                    "points_total": len(p.points),
                }
            )
        return out

    def odom_summary(self, od: Odometry) -> Dict[str, Any]:
        p = od.pose.pose.position
        q = od.pose.pose.orientation
        t = od.twist.twist
        return {
            "stamp": stamp_to_sec(od.header.stamp),
            "frame_id": od.header.frame_id,
            "child_frame_id": od.child_frame_id,
            "x": float(p.x),
            "y": float(p.y),
            "yaw": float(yaw_from_quat(q)),
            "vx": float(t.linear.x),
            "vy": float(t.linear.y),
            "wz": float(t.angular.z),
        }

    def cmd_summary(self, tw: Twist) -> Dict[str, Any]:
        return {"vx": float(tw.linear.x), "vy": float(tw.linear.y), "wz": float(tw.angular.z)}

    def speed_summary(self, sp: SpeedLimit) -> Dict[str, Any]:
        return {
            "stamp": stamp_to_sec(sp.header.stamp),
            "percentage": bool(sp.percentage),
            "speed_limit": float(sp.speed_limit),
        }

    def status_summary(self, st: GoalStatusArray) -> Dict[str, Any]:
        # 只保留状态码列表，避免 UUID 太长
        return {"n": len(st.status_list), "codes": [int(s.status) for s in st.status_list]}

    # ---- decision handling ----

    def on_decision(self, msg: StrategyDecisionStamped):
        mode = str(msg.decision.mode).strip()
        if mode == "":
            mode = "NONE"

        # 不记录 NONE
        if mode not in TRIGGER_MODES:
            self.last_mode = mode
            return

        # 只在“进入某个触发模式”时记录一次（避免 A_WAIT 持续发布刷屏）
        if mode == self.last_mode:
            return

        # cooldown：防止 mode 抖动导致秒级重复记录
        if self.args.cooldown_sec > 0.0:
            if now_wall() - self.last_event_wall < self.args.cooldown_sec:
                self.last_mode = mode
                return

        self.last_mode = mode
        self.last_event_wall = now_wall()

        t_event = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0 else self.now_ros_sec()
        now_ros = self.now_ros_sec()

        # pick nearest snapshots
        scan_it = self.buf_scan.nearest(t_event)
        path_it = self.buf_path.nearest(t_event)
        trk_it = self.buf_tracks.nearest(t_event)
        prd_it = self.buf_preds.nearest(t_event)
        od_it = self.buf_odom.nearest(t_event)
        cmd_it = self.buf_cmd.nearest(t_event)
        sp_it = self.buf_speed.nearest(t_event)
        fol_it = self.buf_follow_status.latest()
        cmp_it = self.buf_compute_status.latest()

        event_id = f"{time.strftime('%H%M%S')}_{mode}"
        rec: Dict[str, Any] = {
            "event_id": event_id,
            "tag": self.args.tag,
            "wall_time": now_wall(),
            "ros_time": now_ros,
            "decision_stamp": t_event,
            "decision_age_sec": now_ros - t_event,
            "mode": mode,
            "expected_supervisor_action": MODE_TO_EXEC_ACTION.get(mode, ""),
            "decision": {
                "stop_now": bool(msg.decision.stop_now),
                "slowdown_ratio": float(msg.decision.slowdown_ratio),
                "t_clear": float(msg.decision.t_clear),
                "t_block": float(msg.decision.t_block),
                "affected_segment_id": str(msg.decision.affected_segment_id),
                "recommended_cluster_id": str(msg.decision.recommended_cluster_id),
                "safe_stop_pose": {
                    "frame_id": msg.decision.safe_stop_pose.header.frame_id,
                    "x": float(msg.decision.safe_stop_pose.pose.position.x),
                    "y": float(msg.decision.safe_stop_pose.pose.position.y),
                },
                "rejoin_pose": {
                    "frame_id": msg.decision.rejoin_pose.header.frame_id,
                    "x": float(msg.decision.rejoin_pose.pose.position.x),
                    "y": float(msg.decision.rejoin_pose.pose.position.y),
                },
            },
            "tf": {
                "map_base_link": self.tf_snapshot("map", "base_link", use_latest=True),
                "map_odom": self.tf_snapshot("map", "odom", use_latest=True),
            },
            "snapshots": {},
        }

        # attach snapshots (safe even if None)
        if scan_it and isinstance(scan_it.msg, LaserScan):
            sd = {
                "stamp": scan_it.t_sec,
                "frame_id": scan_it.msg.header.frame_id,
                "age_sec": now_ros - scan_it.t_sec,
                "features": self.scan_features(scan_it.msg),
            }
            if self.args.save_scan_ranges:
                # 可能很大，慎用；但事件量通常不多，可以打开
                dec = max(1, int(self.args.scan_decimate))
                sd["ranges_decimated"] = [float(r) for r in scan_it.msg.ranges[::dec]]
                sd["scan_decimate"] = dec
            rec["snapshots"]["scan"] = sd

        if path_it and isinstance(path_it.msg, Path):
            rec["snapshots"]["current_path"] = {
                "age_sec": now_ros - path_it.t_sec,
                **self.path_summary(path_it.msg, self.args.path_max_pts, self.args.path_decimate),
            }

        if trk_it and isinstance(trk_it.msg, ObstacleTrackArray):
            rec["snapshots"]["tracks"] = {
                "age_sec": now_ros - trk_it.t_sec,
                **self.tracks_summary(trk_it.msg, self.args.tracks_max_items),
            }

        if prd_it and isinstance(prd_it.msg, ObstaclePredictionArray):
            rec["snapshots"]["predictions"] = {
                "age_sec": now_ros - prd_it.t_sec,
                **self.preds_summary(prd_it.msg, self.args.preds_max_items, self.args.preds_max_pts, self.args.preds_decimate),
            }

        if od_it and isinstance(od_it.msg, Odometry):
            rec["snapshots"]["odom"] = {
                "age_sec": now_ros - od_it.t_sec,
                **self.odom_summary(od_it.msg),
            }

        if cmd_it and isinstance(cmd_it.msg, Twist):
            rec["snapshots"]["cmd_vel"] = {
                "age_sec": now_ros - cmd_it.t_sec,
                **self.cmd_summary(cmd_it.msg),
            }

        if sp_it and isinstance(sp_it.msg, SpeedLimit):
            rec["snapshots"]["speed_limit"] = {
                "age_sec": now_ros - sp_it.t_sec,
                **self.speed_summary(sp_it.msg),
            }

        if fol_it and isinstance(fol_it.msg, GoalStatusArray):
            rec["snapshots"]["follow_path_status"] = {
                "age_sec": now_ros - fol_it.t_sec,
                **self.status_summary(fol_it.msg),
            }

        if cmp_it and isinstance(cmp_it.msg, GoalStatusArray):
            rec["snapshots"]["compute_path_status"] = {
                "age_sec": now_ros - cmp_it.t_sec,
                **self.status_summary(cmp_it.msg),
            }


        # optional: post snapshot after a short delay (capture supervisor reactions)
        if self.args.post_snapshot_sec and float(self.args.post_snapshot_sec) > 0.0:
            delay = float(self.args.post_snapshot_sec)
            time.sleep(delay)
            now_ros2 = self.now_ros_sec()

            rec["post"] = {
                "ros_time": now_ros2,
                "delay_sec": delay,
                "snapshots": {},
            }

            path2 = self.buf_path.latest()
            cmd2  = self.buf_cmd.latest()
            sp2   = self.buf_speed.latest()
            fol2  = self.buf_follow_status.latest()
            cmp2  = self.buf_compute_status.latest()

            if path2 and isinstance(path2.msg, Path):
                rec["post"]["snapshots"]["current_path"] = {
                    "age_sec": now_ros2 - path2.t_sec,
                    **self.path_summary(path2.msg, self.args.path_max_pts, self.args.path_decimate),
                }
            if cmd2 and isinstance(cmd2.msg, Twist):
                rec["post"]["snapshots"]["cmd_vel"] = {
                    "age_sec": now_ros2 - cmd2.t_sec,
                    **self.cmd_summary(cmd2.msg),
                }
            if sp2 and isinstance(sp2.msg, SpeedLimit):
                rec["post"]["snapshots"]["speed_limit"] = {
                    "age_sec": now_ros2 - sp2.t_sec,
                    **self.speed_summary(sp2.msg),
                }
            if fol2 and isinstance(fol2.msg, GoalStatusArray):
                rec["post"]["snapshots"]["follow_path_status"] = {
                    "age_sec": now_ros2 - fol2.t_sec,
                    **self.status_summary(fol2.msg),
                }
            if cmp2 and isinstance(cmp2.msg, GoalStatusArray):
                rec["post"]["snapshots"]["compute_path_status"] = {
                    "age_sec": now_ros2 - cmp2.t_sec,
                    **self.status_summary(cmp2.msg),
                }
        # write JSONL
        self._jsonl_fp.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._jsonl_fp.flush()

        self.get_logger().info(f"[EVENT] mode={mode} t={t_event:.3f} saved -> {self.jsonl_path}")

        # optional bag clip
        if self.args.bag:
            try:
                noww = now_wall()
                min_iv = float(getattr(self.args, 'bag_min_interval_sec', 0.0))
                if min_iv > 0.0 and (noww - self._last_bag_wall) < min_iv:
                    self.get_logger().warn(
                        f"[BAG] cooldown {noww - self._last_bag_wall:.1f}s<{min_iv:.1f}s, skip this event's bag clip"
                    )
                else:
                    bag_dir = os.path.join(self.base_dir, 'bags', event_id)
                    topics = [
                        self.args.decision_topic,
                        self.args.scan_topic,
                        self.args.path_topic,
                        self.args.tracks_topic,
                        self.args.preds_topic,
                        self.args.odom_topic,
                        self.args.cmd_vel_topic,
                        self.args.speed_limit_topic,
                        f"{self.args.follow_action}/_action/status",
                        f"{self.args.compute_path_action}/_action/status",
                        '/tf',
                        '/tf_static',
                    ]
                    ok = self.bag.record_clip(bag_dir, topics, self.args.bag_seconds)
                    if ok:
                        self._last_bag_wall = noww
                        self.get_logger().info(f"[BAG] recording {self.args.bag_seconds:.1f}s -> {bag_dir}")
                    else:
                        self.get_logger().warn("[BAG] busy, skip this event's bag clip")
            except Exception as e:
                # Never crash the run because of recorder issues
                self.get_logger().error(f"[BAG] recorder error: {type(e).__name__}: {e}")

    def destroy_node(self):
        try:
            self._jsonl_fp.close()
        except Exception:
            pass
        super().destroy_node()


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(add_help=True)

    p.add_argument("--out-dir", default="~/.ros/ae_events", help="输出根目录（每次运行会新建 run_id 子目录）")
    p.add_argument("--tag", default="", help="写入每条事件的 tag（比如 caseA_selectorA）")

    # topics
    p.add_argument("--decision-topic", default="/coverage/strategy_decision")
    p.add_argument("--scan-topic", default="/scan")
    p.add_argument("--path-topic", default="/coverage/current_path")
    p.add_argument("--tracks-topic", default="/coverage/obstacles/tracks")
    p.add_argument("--preds-topic", default="/coverage/obstacles/predictions")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--cmd-vel-topic", default="/cmd_vel")
    p.add_argument("--speed-limit-topic", default="/speed_limit")

    # actions
    p.add_argument("--follow-action", default="/follow_path")
    p.add_argument("--compute-path-action", default="/compute_path_to_pose")

    # buffers & snapshot size
    p.add_argument("--buf-len", type=int, default=250)
    p.add_argument("--cooldown-sec", type=float, default=0.8)

    p.add_argument("--post-snapshot-sec", type=float, default=0.25, help="触发后再延迟抓一次快照（用于捕捉 supervisor cancel/replan 等动作）；设为 0 禁用")

    p.add_argument("--save-scan-ranges", action="store_true", help="保存 decimated scan ranges（事件少时可开）")
    p.add_argument("--scan-decimate", type=int, default=4)

    p.add_argument("--path-max-pts", type=int, default=200)
    p.add_argument("--path-decimate", type=int, default=2)

    p.add_argument("--tracks-max-items", type=int, default=20)

    p.add_argument("--preds-max-items", type=int, default=20)
    p.add_argument("--preds-max-pts", type=int, default=120)
    p.add_argument("--preds-decimate", type=int, default=1)

    # rosbag2
    p.add_argument("--bag", action="store_true", help="触发时录一个短 rosbag2 clip")
    p.add_argument("--bag-seconds", type=float, default=6.0)
    p.add_argument("--bag-min-interval-sec", type=float, default=9.0,
                   help="两次 bag clip 之间的最小墙钟间隔（秒）。用于避免 mode 抖动导致频繁尝试录包 / busy")

    return p


def main():
    # 先让 rclpy 解析 ROS args
    rclpy.init(args=sys.argv)

    non_ros = remove_ros_args(sys.argv)
    parser = build_arg_parser()
    args = parser.parse_args(non_ros[1:])

    args.out_dir = os.path.expanduser(args.out_dir)

    node = AETriggerRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
