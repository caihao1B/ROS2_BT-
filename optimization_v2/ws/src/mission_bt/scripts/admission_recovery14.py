"""Bounded recovery of an interrupted observation admission, not path planning.

The safety gate still stops independently. Only current qualified observations,
stopped tools/base, and current costmaps can clear this latch. A navigation or
tool failure must not enter it. The 12-resume ceiling is the existing S13 action
retry ceiling, applied more conservatively to the whole S14 mission.
"""
class AdmissionRecovery:
    def __init__(self, clear_hold_s, max_block_s, max_resumes=12):
        self.clear_hold_s = clear_hold_s
        self.max_block_s = max_block_s
        self.max_resumes = max_resumes
        self.since = None
        self.clear_since = None
        self.resumes = 0
        self.exhausted = False

    def begin(self, t):
        if self.since is None:
            self.since = t
            self.clear_since = None

    def update(self, t, qualified):
        if self.since is None:
            return 'IDLE'
        if (t < self.since or t - self.since >= self.max_block_s
                or self.resumes >= self.max_resumes):
            self.exhausted = True
            return 'EXHAUSTED'
        if not qualified:
            self.clear_since = None
            return 'WAITING'
        if self.clear_since is None:
            self.clear_since = t
        if t - self.clear_since < self.clear_hold_s:
            return 'CLEAR_HOLD'
        self.since = self.clear_since = None
        self.resumes += 1
        return 'RESTORED'


def observation_interruption(reason, platform_ok, perception):
    """A fresh protective geometry stop stays in the obstruction policy."""
    physical_obstruction = (perception.get('geometry_fresh', False)
                            and perception.get('boundary_valid', False)
                            and perception.get('reason') == 'GEOMETRIC_PROTECTIVE_STOP')
    return (reason in ['SHORT_OBSTRUCTION_WAIT', 'CLEAR_HYSTERESIS_WAIT',
                       'OBSERVATION_RECOVERY_WAIT']
            and not platform_ok and not physical_obstruction)
