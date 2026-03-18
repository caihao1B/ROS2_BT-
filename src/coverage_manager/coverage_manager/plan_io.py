import json
import math
import hashlib
from typing import Any, Dict, List, Optional, Tuple

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def load_plan(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def compute_segment_length_m(poses: List[Dict[str, Any]]) -> float:
    if len(poses) < 2:
        return 0.0
    total = 0.0
    prev = (float(poses[0]["x"]), float(poses[0]["y"]))
    for p in poses[1:]:
        cur = (float(p["x"]), float(p["y"]))
        total += _dist(prev, cur)
        prev = cur
    return total


def merge_short_segments(
    segments: List[Dict[str, Any]],
    length_threshold_m: float = 1.0,
    min_poses: int = 3,
) -> List[Dict[str, Any]]:
    """Merge short segments into the previous segment.

    Rule (guard rail):
    - if segment_length < length_threshold_m OR poses < min_poses -> merge into previous
    """
    if not segments:
        return segments

    merged: List[Dict[str, Any]] = []
    for seg in segments:
        poses = seg.get("poses", [])
        seg_len = float(seg.get("meta", {}).get("length_m", compute_segment_length_m(poses)))
        too_short = (seg_len < length_threshold_m) or (len(poses) < min_poses)

        if too_short and merged:
            prev = merged[-1]
            prev_poses = prev.get("poses", [])

            # Avoid duplicating the junction pose if identical
            if prev_poses and poses:
                last = prev_poses[-1]
                first = poses[0]
                if float(last.get("x", 0.0)) == float(first.get("x", 0.0)) and float(last.get("y", 0.0)) == float(first.get("y", 0.0)):
                    poses = poses[1:]

            prev_poses.extend(poses)
            prev["poses"] = prev_poses

            # Update meta (best-effort)
            prev.setdefault("meta", {})
            prev["meta"]["merged"] = True
            prev["meta"].setdefault("merged_from", [])
            prev["meta"]["merged_from"].append(seg.get("id", ""))

            prev_len = float(prev["meta"].get("length_m", compute_segment_length_m(prev_poses)))
            prev["meta"]["length_m"] = prev_len + seg_len
        else:
            merged.append(seg)
    return merged


def _safe_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def make_pose_stamped(p: Dict[str, Any], frame_id: str, stamp) -> PoseStamped:
    """Convert one plan pose dict -> PoseStamped."""
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = stamp

    ps.pose.position.x = _safe_float(p.get("x"), 0.0)
    ps.pose.position.y = _safe_float(p.get("y"), 0.0)
    ps.pose.position.z = _safe_float(p.get("z"), 0.0)

    # Use quaternion if present, otherwise compute from yaw
    if all(k in p for k in ("qx", "qy", "qz", "qw")):
        ps.pose.orientation.x = _safe_float(p.get("qx"), 0.0)
        ps.pose.orientation.y = _safe_float(p.get("qy"), 0.0)
        ps.pose.orientation.z = _safe_float(p.get("qz"), 0.0)
        ps.pose.orientation.w = _safe_float(p.get("qw"), 1.0)
    else:
        yaw = _safe_float(p.get("yaw"), 0.0)
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)

    return ps


def segment_to_path(seg: Dict[str, Any], now_stamp, default_frame: str = "map") -> Path:
    """Convert one segment dict -> nav_msgs/Path.

    Key trick for stability:
    - Use stamp=0 for Path + PoseStamped so TF2 always uses the latest transform.
      This avoids transient TF extrapolation / missing-frame windows at segment switches.
    """
    frame_id = str(seg.get("frame_id") or default_frame) or default_frame

    zero = Time(sec=0, nanosec=0)

    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = zero

    poses = seg.get("poses", []) or []
    out: List[PoseStamped] = []
    for p in poses:
        out.append(make_pose_stamped(p, frame_id, zero))
    path.poses = out
    return path


def plan_bounds(plan: Dict[str, Any]) -> Optional[Tuple[float, float, float, float]]:
    """Return (min_x, min_y, max_x, max_y) over all poses in the plan."""
    segments = plan.get("segments", [])
    xs: List[float] = []
    ys: List[float] = []
    for seg in segments:
        for p in seg.get("poses", []):
            if "x" in p and "y" in p:
                xs.append(float(p["x"]))
                ys.append(float(p["y"]))
    if not xs or not ys:
        return None
    return (min(xs), min(ys), max(xs), max(ys))

