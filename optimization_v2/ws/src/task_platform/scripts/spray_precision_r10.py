"""Stationary evidence and short, low-speed capture for A1 point service.

Both return proposals. Only the existing global dock and physical tool action
can admit spraying. All input comes from the online supervisor and gate.
"""
import math
from collections import deque

from docking_r10 import stable_dock


class StoppedWindow:
    def __init__(self, target, settle_s):
        self.target = target
        self.settle_s = settle_s
        self.history = []
        self.epoch = None
        self.stopped_since = None

    def update(self, now, loc, gate):
        stamp = loc.get('odom_t')
        qualified = (loc.get('valid') and loc.get('pose') and stamp is not None
                     and -.03 <= now - stamp < .15 and -.03 <= now - loc.get('t', -100) < .15
                     and -.03 <= now - gate.get('t', -100) < .15
                     and gate.get('stopped') and gate.get('tool_stopped'))
        epoch = loc.get('localization_epoch')
        if not qualified or epoch != self.epoch:
            self.history.clear()
            self.stopped_since = None
            self.epoch = epoch
        if not qualified:
            return False, {}
        if self.history and (stamp < self.history[-1][0] or stamp - self.history[-1][0] > .15):
            self.history.clear()
            self.stopped_since = None
        if self.stopped_since is None:
            self.stopped_since = now
        # Global messages can arrive after the gate with an older acquisition stamp.
        if stamp < self.stopped_since:
            return False, {}
        if not self.history or stamp > self.history[-1][0]:
            self.history.append((stamp, loc['pose'][:2]))
            self.history[:] = [r for r in self.history if stamp - r[0] <= 2.2]
        stable, metrics = stable_dock(self.history, self.target)
        metrics['continuous_stopped_s'] = now - self.stopped_since
        metrics['localization_epoch'] = epoch
        return bool(stable and now - self.stopped_since >= self.settle_s), metrics


class LowSpeedCapture:
    """Request a controlled stop only after multiple in-tolerance samples."""
    def __init__(self, goal_m):
        self.radius = goal_m
        self.samples = deque()
        self.epoch = None

    def update(self, now, target, loc, gate, qualified):
        stamp = loc.get('odom_t')
        epoch = loc.get('localization_epoch')
        if not (qualified and loc.get('pose') and stamp is not None
                and -.03 <= now - stamp < .15 and -.03 <= now - gate.get('t', -100) < .15
                and math.isfinite(loc.get('v', float('nan'))) and abs(loc['v']) <= .10):
            self.samples.clear()
            return None
        distance = math.dist(loc['pose'][:2], target)
        if (not math.isfinite(distance) or distance > self.radius or epoch != self.epoch
                or (self.samples and not 0 <= stamp-self.samples[-1][0] <= .15)):
            self.samples.clear()
        self.epoch = epoch
        if not math.isfinite(distance) or distance > self.radius:
            return None
        if self.samples and stamp == self.samples[-1][0]:
            return None
        self.samples.append((stamp, distance))
        while self.samples and stamp - self.samples[0][0] > .35:
            self.samples.popleft()
        if len(self.samples) >= 4 and stamp - self.samples[0][0] >= .12:
            return dict(reason='FRESH_LOW_SPEED_INSIDE_NAV_RADIUS', distance_m=distance,
                        samples=len(self.samples), span_s=stamp-self.samples[0][0],
                        measured_v_mps=loc['v'], navigation_success=False, work_admitted=False)
        return None
