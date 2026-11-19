"""Full B1 entrance candidate with measured progress between path vertices."""
import bisect
import math
from pathlib import Path

import qualify_b1_full as full
from qualify_b1_steering import RetryProgress


class ContinuousRetryProgress(RetryProgress):
    """Track forward arc on the current primitive. Only the retry counter consumes it."""

    def __init__(self, segment, begin, max_speed):
        super().__init__(segment, begin, max_speed)
        self.previous_sample = None
        self.gear = segment['direction']
        self.cursor = 0.
        self.furthest_seen = 0.
        self.last_position_time = begin

    def observe(self, loc, gate, clock, navigating=True):
        if not loc or not loc.get('valid'):
            self.previous_sample = None
            return
        now = loc.get('odom_t')
        pose, odom = loc.get('pose'), loc.get('odom')
        published = loc.get('t')
        if (now is None or published is None or not math.isfinite(published) or not math.isfinite(clock)
                or not 0 <= clock-published <= .15 or not 0 <= clock-now <= .15
                or not 0 <= published-now <= .15
                or not pose or len(pose) < 3 or not odom or len(odom) < 3
                or not all(math.isfinite(x) for x in (now, *pose[:3], *odom[:3]))):
            self.previous_sample = None
            return

        previous = self.previous_sample
        epoch = loc.get('localization_epoch')
        if self.last is not None and now <= self.last:
            return
        self.last = now
        gate_time = (gate or {}).get('t')
        if (gate_time is None or not math.isfinite(gate_time) or not 0 <= clock-gate_time <= .15
                or epoch is None):
            self.previous_sample = None
            return
        driving = bool(gate.get('allowed') and gate.get('mode') == 'DRIVE'
                       and gate.get('armed_direction') == self.gear)
        self.previous_sample = (now, odom[:3], epoch, driving)
        if not driving:
            return
        forward = 0.
        if previous is not None and previous[2] == epoch and now - previous[0] <= .5:
            ox, oy, heading = previous[1]
            dx, dy = odom[0]-ox, odom[1]-oy
            if math.hypot(dx,dy) > self.max_speed * (now-previous[0]) + .035:
                return
            if previous[3]:
                forward = self.gear * (math.cos(heading)*dx + math.sin(heading)*dy)
        # A trustworthy pose can reacquire the nearby part of this segment after
        # a hold. Reacquisition itself contributes zero to the retry counter.
        lower = max(0., self.cursor - .20)
        dt = max(0., min(.5, now - self.last_position_time))
        upper = min(self.arc[-1], self.cursor + self.max_speed*dt + .035)
        start = max(0, bisect.bisect_right(self.arc, lower) - 1)
        stop = min(len(self.points) - 1, bisect.bisect_right(self.arc, upper))
        closest = None
        for idx in range(start, stop):
            a, b = self.points[idx:idx+2]
            length = self.arc[idx+1] - self.arc[idx]
            if length < 1e-9:
                continue
            dx, dy = b[0]-a[0], b[1]-a[1]
            fraction = max(0., min(1., ((pose[0]-a[0])*dx + (pose[1]-a[1])*dy) / (length*length)))
            arc = self.arc[idx] + fraction*length
            if not lower <= arc <= upper:
                continue
            heading = a[2] + fraction * math.remainder(b[2]-a[2], math.tau)
            offset = math.hypot(pose[0]-a[0]-fraction*dx, pose[1]-a[1]-fraction*dy)
            if offset <= .15 and abs(math.remainder(pose[2]-heading, math.tau)) <= .18:
                candidate = (offset, arc)
                if closest is None or candidate < closest:
                    closest = candidate
        if closest is not None:
            self.cursor = closest[1]
            new_arc = max(0., self.cursor - self.furthest_seen)
            self.furthest_seen = max(self.furthest_seen, self.cursor)
            self.last_position_time = now
            if navigating and forward > 0.:
                self.best += min(new_arc, forward)


def main():
    # The entrance runtime launches the qualifying module itself as the worker and
    # hashes that file and its generated code in the session's implementation bindings.
    from b1_terrain_timing import for_full_candidate

    original_worker = full.context.worker
    entrance_configure = full.configure

    def configure(wall_seconds):
        with for_full_candidate():
            return entrance_configure(wall_seconds)

    def worker():
        code = full.replace_once(original_worker(),
            "   if retry_progress is not None and not st.get('endpoint_recovery'):retry_progress.observe(st.get('loc'),st.get('gate'))",
            "  if retry_progress is not None:\n"
            "   retry_progress.observe(None if st.get('endpoint_recovery') else st.get('loc'),"
            "st.get('gate'), st['t'], navigating=st['phase']=='NAVIGATING' and not st['nav_idle'])")
        return 'from qualify_b1_progress import ContinuousRetryProgress as RetryProgress\n' + code

    full.configure = configure
    full.context.worker = worker
    full.__file__ = str(Path(__file__).resolve())
    full.RESULTS = full.APP / 'results/experiments/b1_full_46_progress'
    full.main()


if __name__ == '__main__':
    main()
