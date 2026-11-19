"""Estimate residual yaw gyro bias from stationary, capture-aligned wheel observations."""
from collections import deque
import math


class StationaryYawBias:
    """A missing observation freezes the last estimate. It never asserts zero motion."""

    def __init__(self):
        self.wheels = deque(maxlen=32)
        self.gates = deque(maxlen=32)
        self.last_imu = None
        self.quiet_since = None
        self.offset = 0.0
        self.updates = 0

    def observe_wheel(self, stamp, speed, yaw_rate):
        if all(math.isfinite(v) for v in (stamp, speed, yaw_rate)):
            if not self.wheels or stamp > self.wheels[-1][0]:
                self.wheels.append((stamp, speed, yaw_rate))

    def observe_gate(self, gate):
        stamp = gate.get("t")
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return
        if self.gates and stamp <= self.gates[-1][0]:
            return
        zero_request = (gate.get("mode") == "STOP" and gate.get("v") == 0
                        and gate.get("w") == 0 and gate.get("nav_idle") is True
                        and gate.get("stopped") is True)
        self.gates.append((stamp, zero_request))

    @staticmethod
    def previous(samples, stamp, age):
        for sample in reversed(samples):
            if sample[0] <= stamp:
                return sample if stamp - sample[0] <= age else None
        return None

    @staticmethod
    def wheel_disputed(stamp, evidence):
        age = stamp - evidence.get("t", -100.)
        independent = evidence.get("independent_motion", {})
        return (0. <= age < 0.5 and independent.get("available") is True
                and "wheel_nis" in independent and independent.get("wheel_valid") is False)

    def update(self, stamp, startup_corrected_gyro, slip_suspected=False):
        """Return bias relative to the original startup calibration, in rad/s."""
        if not math.isfinite(stamp) or (self.last_imu is not None and stamp <= self.last_imu):
            return self.offset
        dt = 0.0 if self.last_imu is None else stamp - self.last_imu
        self.last_imu = stamp
        wheel = self.previous(self.wheels, stamp, 0.08)
        gate = self.previous(self.gates, stamp, 0.15)
        if (not math.isfinite(startup_corrected_gyro) or dt > 0.10 or not wheel
                or slip_suspected or not gate or not gate[1] or abs(wheel[1]) > 0.012
                or abs(wheel[2]) > 0.010):
            self.quiet_since = None
            return self.offset
        # Subtract the measured encoder rate, retaining even slow physical rotation.
        observed_bias = startup_corrected_gyro - wheel[2]
        if abs(observed_bias) > 0.020 or abs(observed_bias - self.offset) > 0.020:
            self.quiet_since = None
            return self.offset
        if self.quiet_since is None:
            self.quiet_since = stamp
        if stamp - self.quiet_since >= 0.6:
            self.offset += -math.expm1(-dt / 4.0) * (observed_bias - self.offset)
            self.updates += 1
        return self.offset
