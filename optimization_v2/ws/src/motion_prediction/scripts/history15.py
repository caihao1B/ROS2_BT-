"""Causal feature builder; no future labels or world script input."""
from collections import defaultdict, deque
import math
import numpy as np


def observation(frame, obj):
    t = float(obj['observation_t'])
    values = [t, *obj['position'], *obj['velocity'], *obj['size'], *obj['covariance']]
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Nonfinite object observation')
    if frame.get('frame') != 'odom' or obj.get('frame') != 'odom':
        raise ValueError('Prediction input frame must be odom')
    if t > frame['t']+.05 or frame['t']-t > .6:
        return None
    if not obj.get('confirmed') or obj.get('state') != 'OBSERVED':
        return None
    covariance = np.array(obj['covariance'], dtype=float).reshape(4, 4)[:2, :2]
    if np.linalg.eigvalsh((covariance+covariance.T)/2).min() < -1e-6:
        raise ValueError('Invalid observation covariance')
    return dict(t=t, id=int(obj['id']), position=list(obj['position'][:2]),
                velocity=list(obj['velocity']), covariance=covariance.tolist(),
                size=list(obj['size']), classification=obj.get('classification'),
                confidence=float(obj.get('confidence', 0.)))


def prepare(records, cutoff, dt=.4, length=8, max_gap=.8):
    """Interpolation may use any observed sample <= cutoff, never a future label.

    Derivatives are backward differences. A duplicated initial derivative is a
    startup convention, not a centered derivative peeking past the current time.
    """
    available = [r for r in records if r['t'] <= cutoff]
    if not available:
        raise ValueError('No causal observations')
    times = np.array([r['t'] for r in available])
    if np.any(np.diff(times) <= 0) or cutoff-times[-1] > .6:
        raise ValueError('Stale or unordered track')
    end = float(times[-1])
    grid = end-np.arange(length-1, -1, -1)*dt
    if grid[0] < times[0]-1e-8:
        raise ValueError('History warmup')
    coordinates = np.array([r['position'] for r in available], dtype=float)
    xy = []
    for t in grid:
        hi = int(np.searchsorted(times, t, side='left'))
        if hi < len(times) and abs(times[hi]-t) < 1e-8:
            xy.append(coordinates[hi])
            continue
        if not 0 < hi < len(times) or times[hi]-times[hi-1] > max_gap:
            raise ValueError('History contains an unobserved interval')
        fraction = (t-times[hi-1])/(times[hi]-times[hi-1])
        xy.append(coordinates[hi-1]*(1-fraction)+coordinates[hi]*fraction)
    xy = np.array(xy)
    velocity = np.zeros_like(xy)
    velocity[1:] = np.diff(xy, axis=0)/dt
    velocity[0] = velocity[1]
    acceleration = np.zeros_like(xy)
    acceleration[1:] = np.diff(velocity, axis=0)/dt
    state = np.concatenate([xy, velocity, acceleration], axis=1)
    return dict(t=end, id=available[-1]['id'], state=state.tolist(),
                covariance=available[-1]['covariance'], size=available[-1]['size'],
                classification=available[-1]['classification'],
                confidence=available[-1]['confidence'], history_times=grid.tolist())


class History:
    def __init__(self):
        self.tracks = defaultdict(lambda: deque(maxlen=120))
        self.last = None

    def add(self, frame):
        t = float(frame['t'])
        if not math.isfinite(t) or (self.last is not None and t <= self.last):
            raise ValueError('Prediction input clock did not advance')
        self.last = t
        accepted = []
        for obj in frame['objects']:
            record = observation(frame, obj)
            if record is None:
                continue
            history = self.tracks[record['id']]
            if history and record['t'] <= history[-1]['t']:
                continue
            if history and record['t']-history[-1]['t'] > 1.2:
                history.clear()
            history.append(record)
            accepted.append(record)
        for identity in list(self.tracks):
            if not self.tracks[identity] or t-self.tracks[identity][-1]['t'] > 1.2:
                del self.tracks[identity]
        return accepted

    def features(self, cutoff):
        features = []
        for records in self.tracks.values():
            try:
                features.append(prepare(records, cutoff))
            except ValueError:
                pass
        return features
