"""Dino initialization needs a longer observed correction stability window."""
from startup_r08 import stable_corrections as base_stability
import numpy as np

def stable_corrections(history):
 ok,metrics=base_stability(history)
 gap=float(np.diff([x[0] for x in history]).max()) if len(history)>1 else 0.
 metrics.update(required_span_s=3.5,required_samples=18,max_observation_gap_s=gap)
 return ok and metrics['span_s']>=3.5 and metrics['samples']>=18 and gap<=.600001,metrics
