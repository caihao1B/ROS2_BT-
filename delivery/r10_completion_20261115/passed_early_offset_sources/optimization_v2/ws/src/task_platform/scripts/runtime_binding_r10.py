"""Accept only the exact normal runtime or the deterministic observer-only derivation."""
from common import *
import hashlib

def spray_runtime_valid(spec):
 original=PKG/'scripts/runtime_spray_r10.py'
 if spec.get('runtime_sha256')==sha(original):return True
 from prepare_r12 import recording_source
 derived=edition('R12')/'runtime_spray.py';expected=recording_source(original.read_text(),'scene_spray_r10','R10').encode();expected_hash=hashlib.sha256(expected).hexdigest()
 bindings=spec.get('implementation_bindings',{})
 return bool(derived.is_file() and spec.get('runtime_sha256')==expected_hash==sha(derived) and bindings.get(str(original.relative_to(ROOT)))==sha(original) and bindings.get(str(derived.relative_to(ROOT)))==expected_hash)
