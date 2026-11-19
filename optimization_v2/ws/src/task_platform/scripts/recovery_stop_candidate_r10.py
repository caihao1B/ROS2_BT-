"""Candidate recovery worker. Require newly observed motor and body stops before changing gear."""
from collections import deque
import json
import math
from pathlib import Path


class RecoveryStopEvidence:
    def __init__(self, session, revision, max_gap):
        self.session = session
        self.revision = revision
        self.max_gap = max_gap
        self.samples = deque(maxlen=64)

    def observe(self, row):
        stamp = row.get('t')
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return
        if self.samples and stamp <= self.samples[-1][0]:
            return
        rpm = row.get('rpm_rad_s')
        stopped = (row.get('sensor_epoch') == self.session and row.get('tool_revision') == self.revision
                   and row.get('healthy') is True and row.get('request_on') is False
                   and row.get('flow') == 0 and isinstance(rpm, (int, float)) and abs(rpm) < .1)
        self.samples.append((stamp, stopped))

    def confirmed(self, barrier, now, gate):
        if not gate or not gate.get('stopped') or not gate.get('tool_stopped'):
            return False
        t = gate.get('t')
        if not isinstance(t, (float, int)) or not barrier < t <= now or now-t > self.max_gap:
            return False
        times = []
        for stamp, stopped in reversed(self.samples):
            if stamp <= barrier or not stopped or stamp > now:
                break
            if times and times[-1]-stamp > self.max_gap:
                break
            times.append(stamp)
        return bool(times and now-times[0] <= self.max_gap and times[0]-times[-1] >= self.max_gap)


def source_for_candidate(source):
    # Specialize the archived production worker at two explicit extension sites.
    # Keep the qualified worker bytes available for reproducing its original run.
    marker = " def publish():"
    subscription = """ stop_evidence=RecoveryStopEvidence(spec['session'],toolcfg['tool_revision'],toolcfg['feedback_gap_s'])
 n.create_subscription(String,'/s10/tool/feedback',lambda m:stop_evidence.observe(json.loads(m.data)),30)
"""
    if source.count(marker) != 1:
        raise ValueError('Mowing worker subscription point changed')
    source = source.replace(marker, subscription + marker)
    marker = "st['nav_idle']=True;tool_finish_result();wait("
    if source.count(marker) != 1:
        raise ValueError('Mowing worker stopped-confirmation point changed')
    source = source.replace(marker, """st['nav_idle']=True;tool_finish_result()
  if st.get('endpoint_recovery'):
   barrier=st['t']
   wait(lambda:stop_evidence.confirmed(barrier,st['t'],st['gate']),5,True)
   log('RECOVERY_STOP_FEEDBACK_CONFIRMED',requested_at=barrier,minimum_span_s=stop_evidence.max_gap)
  wait(""")
    return source


if __name__ == '__main__':
    worker = Path(__file__).with_name('mission_mowing_r10.py')
    exec(compile(source_for_candidate(worker.read_text()), str(worker), 'exec'))
