import os
import csv
import json
import time
import datetime
from array import array
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

import tf2_ros


def now_iso() -> str:
    return datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc).isoformat()


def default_output_dir() -> str:
    d = os.path.expanduser("~/.ros/coverage_metrics")
    os.makedirs(d, exist_ok=True)
    return d


class CoverageMetrics(Node):
    def __init__(self) -> None:
        super().__init__("coverage_metrics")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("tf_map_frame", "map")
        self.declare_parameter("tf_base_frame", "base_link")
        self.declare_parameter("tool_radius_m", 0.20)

        self.declare_parameter("sample_period_sec", 0.5)
        self.declare_parameter("write_period_sec", 2.0)

        self.declare_parameter("output_dir", "")
        self.declare_parameter("timeseries_filename", "metrics_timeseries.csv")
        self.declare_parameter("summary_filename", "metrics_summary.json")

        self.declare_parameter("events_topic", "/coverage/events")

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Map / coverage grid state
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

        # Events / task stats
        self._stuck_count = 0
        self._seg_total = 0
        self._seg_success = 0
        self._seg_failed = 0
        self._seg_skipped = 0

        # Output
        out_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self._out_dir = out_dir if out_dir else default_output_dir()
        self._ts_path = os.path.join(self._out_dir, self.get_parameter("timeseries_filename").value)
        self._summary_path = os.path.join(self._out_dir, self.get_parameter("summary_filename").value)

        self._ts_file = open(self._ts_path, "w", newline="", encoding="utf-8")
        self._ts_writer = csv.writer(self._ts_file)
        self._ts_writer.writerow([
            "ts_iso",
            "covered_free",
            "overlapped_free",
            "total_free",
            "coverage_ratio",
            "overlap_ratio",
            "stuck_count",
            "seg_total",
            "seg_success",
            "seg_failed",
            "seg_skipped",
        ])
        self._ts_file.flush()

        # Subscriptions
        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, 1)

        events_topic = self.get_parameter("events_topic").get_parameter_value().string_value
        self.create_subscription(String, events_topic, self._events_cb, 10)

        # Timers
        self._sample_timer = self.create_timer(float(self.get_parameter("sample_period_sec").value), self._sample_once)
        self._write_timer = self.create_timer(float(self.get_parameter("write_period_sec").value), self._write_once)

        self.get_logger().info(f"CoverageMetrics writing: {self._ts_path} and {self._summary_path}")

    def destroy_node(self):
        try:
            self._ts_file.flush()
            self._ts_file.close()
        except Exception:
            pass
        super().destroy_node()

    # ----------------------------
    # Map handling
    # ----------------------------
    def _map_cb(self, msg: OccupancyGrid) -> None:
        # Only initialize once (map is static for this experiment)
        if self._map is not None:
            return

        self._map = msg
        self._width = int(msg.info.width)
        self._height = int(msg.info.height)
        self._res = float(msg.info.resolution)
        self._origin_x = float(msg.info.origin.position.x)
        self._origin_y = float(msg.info.origin.position.y)

        n = self._width * self._height
        data = msg.data

        free = bytearray(n)
        total_free = 0
        for i in range(n):
            # Common static map: 0=free, 100=occupied, -1=unknown
            if data[i] == 0:
                free[i] = 1
                total_free += 1
        self._free_mask = free
        self._cover_counts = array("H", [0] * n)
        self._total_free = total_free
        self._covered_free = 0
        self._overlapped_free = 0

        self.get_logger().info(
            f"/map received: {self._width}x{self._height}, res={self._res}, total_free={self._total_free}"
        )

    # ----------------------------
    # Events handling
    # ----------------------------
    def _events_cb(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except Exception:
            return

        et = str(ev.get("type", ""))
        if et == "stuck_watchdog":
            self._stuck_count += 1
            return

        if et == "executor_started":
            try:
                self._seg_total = int(ev.get("total_segments", 0))
            except Exception:
                self._seg_total = 0
            return

        if et == "segment_success":
            self._seg_success += 1
            return

        if et == "segment_failed":
            # This counts attempts; the summary "failed segments" is ambiguous.
            # We treat it as 'segments that ended in failed without later success' at end.
            # For an online indicator, count failed attempts here.
            self._seg_failed += 1
            return

        if et == "segment_skipped":
            self._seg_skipped += 1
            return

        if et == "executor_done":
            self._write_summary(final=True)

    # ----------------------------
    # Coverage projection
    # ----------------------------
    def _lookup_robot_xy(self) -> Optional[tuple]:
        if self._map is None:
            return None
        map_frame = self.get_parameter("tf_map_frame").get_parameter_value().string_value
        base_frame = self.get_parameter("tf_base_frame").get_parameter_value().string_value
        try:
            tf = self._tf_buffer.lookup_transform(map_frame, base_frame, Time())
            x = float(tf.transform.translation.x)
            y = float(tf.transform.translation.y)
            return (x, y)
        except Exception:
            return None

    def _xy_to_cell(self, x: float, y: float) -> Optional[tuple]:
        if self._map is None:
            return None
        mx = int((x - self._origin_x) / self._res)
        my = int((y - self._origin_y) / self._res)
        if mx < 0 or my < 0 or mx >= self._width or my >= self._height:
            return None
        return (mx, my)

    def _sample_once(self) -> None:
        if self._map is None or self._free_mask is None or self._cover_counts is None:
            return

        xy = self._lookup_robot_xy()
        if xy is None:
            return
        cell = self._xy_to_cell(xy[0], xy[1])
        if cell is None:
            return

        radius_m = float(self.get_parameter("tool_radius_m").value)
        r_cells = max(1, int(radius_m / self._res))
        cx, cy = cell

        for dy in range(-r_cells, r_cells + 1):
            y = cy + dy
            if y < 0 or y >= self._height:
                continue
            for dx in range(-r_cells, r_cells + 1):
                x = cx + dx
                if x < 0 or x >= self._width:
                    continue
                # circle test
                if (dx * dx + dy * dy) * (self._res * self._res) > (radius_m * radius_m):
                    continue
                idx = y * self._width + x
                if self._free_mask[idx] != 1:
                    continue

                prev = self._cover_counts[idx]
                self._cover_counts[idx] = prev + 1
                if prev == 0:
                    self._covered_free += 1
                elif prev == 1:
                    self._overlapped_free += 1

    # ----------------------------
    # Outputs
    # ----------------------------
    def _compute_ratios(self) -> Dict[str, Any]:
        total_free = self._total_free
        covered = self._covered_free
        overlapped = self._overlapped_free
        cov_ratio = (covered / total_free) if total_free > 0 else 0.0
        ov_ratio = (overlapped / covered) if covered > 0 else 0.0
        return {
            "covered_free": covered,
            "overlapped_free": overlapped,
            "total_free": total_free,
            "coverage_ratio": cov_ratio,
            "overlap_ratio": ov_ratio,
        }

    def _write_once(self) -> None:
        if self._map is None:
            return
        stats = self._compute_ratios()
        row = [
            now_iso(),
            stats["covered_free"],
            stats["overlapped_free"],
            stats["total_free"],
            f"{stats['coverage_ratio']:.6f}",
            f"{stats['overlap_ratio']:.6f}",
            self._stuck_count,
            self._seg_total,
            self._seg_success,
            self._seg_failed,
            self._seg_skipped,
        ]
        self._ts_writer.writerow(row)
        self._ts_file.flush()

    def _write_summary(self, final: bool = False) -> None:
        stats = self._compute_ratios()
        summary = {
            "ts_iso": now_iso(),
            "final": bool(final),
            **stats,
            "stuck_count": self._stuck_count,
            "segments": {
                "total": self._seg_total,
                "success_events": self._seg_success,
                "failed_events": self._seg_failed,
                "skipped_events": self._seg_skipped,
            },
        }
        os.makedirs(self._out_dir, exist_ok=True)
        with open(self._summary_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f"Wrote summary: {self._summary_path}")


def main() -> None:
    rclpy.init()
    node = CoverageMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
