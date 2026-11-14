"""Dino initialization needs a longer observed correction stability window."""
from startup_r08 import stable_corrections as base_stability

def stable_corrections(history):
 ok,metrics=base_stability(history)
 metrics.update(required_span_s=3.5,required_samples=18)
 return ok and metrics['span_s']>=3.5 and metrics['samples']>=18,metrics
