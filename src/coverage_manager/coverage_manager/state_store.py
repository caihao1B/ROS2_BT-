import json
import os
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


def default_state_path(plan_sha: str) -> str:
    base = os.path.expanduser("~/.ros")
    os.makedirs(base, exist_ok=True)
    return os.path.join(base, f"coverage_state_{plan_sha[:10]}.json")


@dataclass
class SegmentState:
    status: str = "pending"  # pending/success/failed/skipped
    attempts: int = 0
    last_error_code: Optional[int] = None
    last_error_msg: str = ""
    last_update_time: str = ""


@dataclass
class ExecutorState:
    plan_sha256: str = ""
    current_index: int = 0
    segments: Dict[str, SegmentState] = field(default_factory=dict)


class StateStore:
    def __init__(self, path: str):
        self.path = path
        self.state = ExecutorState()

    def load(self) -> bool:
        if not self.path or not os.path.exists(self.path):
            return False
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                raw = json.load(f)
            self.state.plan_sha256 = raw.get("plan_sha256", "")
            self.state.current_index = int(raw.get("current_index", 0))
            self.state.segments = {}
            raw_segs = raw.get("segments", {})
            for seg_id, seg_state in raw_segs.items():
                ss = SegmentState(
                    status=str(seg_state.get("status", "pending")),
                    attempts=int(seg_state.get("attempts", 0)),
                    last_error_code=seg_state.get("last_error_code", None),
                    last_error_msg=str(seg_state.get("last_error_msg", "")),
                    last_update_time=str(seg_state.get("last_update_time", "")),
                )
                self.state.segments[str(seg_id)] = ss
            return True
        except Exception:
            return False

    def save(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        raw: Dict[str, Any] = {
            "plan_sha256": self.state.plan_sha256,
            "current_index": self.state.current_index,
            "segments": {
                seg_id: {
                    "status": ss.status,
                    "attempts": ss.attempts,
                    "last_error_code": ss.last_error_code,
                    "last_error_msg": ss.last_error_msg,
                    "last_update_time": ss.last_update_time,
                }
                for seg_id, ss in self.state.segments.items()
            },
        }
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(raw, f, indent=2, ensure_ascii=False)
        os.replace(tmp, self.path)
