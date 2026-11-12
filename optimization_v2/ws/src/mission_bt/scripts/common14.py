"""S14 adapters use frozen upstream inputs without changing upstream receipts."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[5]
BASE = ROOT / 'optimization_v2'
PKG = Path(__file__).resolve().parents[1]
NAV = BASE / 'ws/src/navigation_adapter'
sys.path.insert(0, str(NAV / 'scripts'))
from common13 import read, dump, sha, rows, plan as base_plan, stamp, yaw
import json, math, time, os, yaml, numpy as np

def plan():
    from binding14 import active_plan
    return active_plan()

def cluster(kind):
    return plan()['zones']['A' if kind == 'diff' else 'B']['plans'][0]

def replace_once(source, before, after):
    if source.count(before) != 1:
        raise ValueError('UPSTREAM_ADAPTER_MISMATCH: ' + before[:100])
    return source.replace(before, after)
