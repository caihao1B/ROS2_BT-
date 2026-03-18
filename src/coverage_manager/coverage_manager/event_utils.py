import json
import datetime
from typing import Any, Dict, Optional


def now_iso() -> str:
    return datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc).isoformat()


def make_event(event_type: str, **kwargs) -> Dict[str, Any]:
    ev: Dict[str, Any] = {"type": event_type, "ts": now_iso()}
    ev.update(kwargs)
    return ev


def dumps_event(ev: Dict[str, Any]) -> str:
    return json.dumps(ev, ensure_ascii=False)


def safe_int(x: Any, default: Optional[int] = None) -> Optional[int]:
    try:
        return int(x)
    except Exception:
        return default
