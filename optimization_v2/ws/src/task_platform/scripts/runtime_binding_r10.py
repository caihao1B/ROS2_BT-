"""Accept only the exact normal runtime or the deterministic observer-only derivation."""
from common import *
import hashlib

def spray_runtime_valid(spec):
 original=PKG/'scripts/runtime_spray_r10.py'
 if spec.get('runtime_sha256')==sha(original):return True
 from prepare_r12 import recording_source
 from prepare_spray_video_r12 import derive as video_source
 candidates=[('runtime_spray.py',recording_source(original.read_text(),'scene_spray_r10','R10')),('runtime_spray_video.py',video_source(original.read_text()))]
 bindings=spec.get('implementation_bindings',{})
 for name,expected in candidates:
  derived=edition('R12')/name;expected_hash=hashlib.sha256(expected.encode()).hexdigest()
  if derived.is_file() and spec.get('runtime_sha256')==expected_hash==sha(derived) and bindings.get(str(original.relative_to(ROOT)))==sha(original) and bindings.get(str(derived.relative_to(ROOT)))==expected_hash:return True
 return False
