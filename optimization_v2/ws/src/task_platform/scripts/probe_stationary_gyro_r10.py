"""Exercise the production spray adapter in a separate ROS domain, without Gazebo."""
import argparse
import json
import hashlib
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import String


SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parents[4]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(overlay=None):
    from overlay_r08 import overlay as build_overlay
    with tempfile.TemporaryDirectory(prefix="spray_bias_overlay_") as temp:
        generated = build_overlay("spray", Path(temp) / "overlay")
        actual = overlay or generated
        if digest(actual / "input_adapter.py") != digest(generated / "input_adapter.py"):
            raise ValueError("Selected input adapter differs from the production derivation")
        files = [SCRIPTS / name for name in ("probe_stationary_gyro_r10.py", "spray_inputs_r10.py",
                 "stationary_gyro_r10.py", "rig_r05.py", "overlay_r07.py", "overlay_r08.py")]
        files += [ROOT / "optimization_v2/ws/src/localization_guard/scripts" / name
                  for name in ("input_adapter.py", "support.py")]
        files += [ROOT / "optimization_v2/config/localization_policy.yaml"]
        before = {str(f.relative_to(ROOT)): digest(f) for f in files}
        generated_hash = digest(actual / "input_adapter.py")
        report = exercise(actual)
        if any(digest(ROOT / name) != value for name, value in before.items()) or digest(actual / "input_adapter.py") != generated_hash:
            raise RuntimeError("Probe inputs changed while running")
        report.update(bindings=before, generated_input_adapter_sha256=generated_hash)
        return report


def exercise(overlay):
    domain = 173
    rclpy.init(domain_id=domain)
    probe = rclpy.create_node("spray_yaw_bias_ros_probe")
    clock = probe.create_publisher(Clock, "/clock", qos_profile_sensor_data)
    wheel = probe.create_publisher(TwistWithCovarianceStamped, "/s04/localization/wheel", qos_profile_sensor_data)
    gyro = probe.create_publisher(Imu, "/s04/localization/imu", qos_profile_sensor_data)
    gate = probe.create_publisher(String, "/r07/gate", 20)
    samples = []
    def observation(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1.e-9
        samples.append((t, m.angular_velocity.z))
    probe.create_subscription(Imu, "/s05/inputs/imu", observation, 40)
    try:
        with tempfile.TemporaryDirectory(prefix="spray_gyro_probe_") as tmp:
            out = Path(tmp) / "inputs"
            env = dict(os.environ, ROS_DOMAIN_ID=str(domain), R05_ADAPTER=str(overlay))
            proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name("spray_inputs_r10.py")),
                                     "--out", str(out)], env=env)
            try:
                end = time.monotonic() + 10.
                while not (all(p.get_subscription_count() for p in (clock, wheel, gyro, gate))
                           and probe.count_publishers("/s05/inputs/imu") == 1):
                    if proc.poll() is not None or time.monotonic() > end:
                        raise RuntimeError("ROS input/output discovery failed")
                    rclpy.spin_once(probe, timeout_sec=.02)
                for tick in range(1451):
                    t = tick / 50.
                    c = Clock()
                    c.clock.sec, c.clock.nanosec = divmod(tick * 20_000_000, 1_000_000_000)
                    clock.publish(c)
                    turning = 19. <= t < 23.
                    if t < 23.:
                        gate.publish(String(data=json.dumps(dict(t=t, mode="DRIVE" if turning else "STOP",
                            v=0., w=.11 if turning else 0., nav_idle=not turning, stopped=not turning))))
                    w = TwistWithCovarianceStamped()
                    w.header.stamp = c.clock
                    w.header.frame_id = "base_link"
                    w.twist.twist.angular.z = .11 if turning else 0.
                    wheel.publish(w)
                    m = Imu()
                    m.header.stamp = c.clock
                    m.header.frame_id = "imu_link"
                    m.linear_acceleration.z = 9.80665
                    m.angular_velocity.z = (.002 if t < 4. else -.004 if t < 23. else .012) + w.twist.twist.angular.z
                    gyro.publish(m)
                    rclpy.spin_once(probe, timeout_sec=.002)
                    time.sleep(.002)
                for _ in range(80):
                    rclpy.spin_once(probe, timeout_sec=.003)
            finally:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
            result = json.loads((out / "result.json").read_text())
            windows = [(3., 3.8), (4.1, 4.5), (18., 18.9), (20., 22.), (26., 28.)]
            means = []
            for a, b in windows:
                rates = [rate for t, rate in samples if a <= t <= b]
                if len(rates) < 10:
                    raise RuntimeError(f"Insufficient actual ROS output samples for {a}–{b}")
                means.append(statistics.mean(rates))
            checks = dict(clean_exit=proc.returncode == 0, epoch_preserved=result["epoch"] == 0,
                          initialized=abs(means[0]) < .0001, injected_drift_visible=means[1] < -.004,
                          recovered_at_stop=abs(means[2]) < .001, real_turn_retained=abs(means[3]-.11) < .001,
                          missing_gate_does_not_hide_new_drift=means[4] > .014,
                          exercised_estimator=result["stationary_updates"] > 100)
            return dict(passed=all(checks.values()), scope="Actual production ROS inputs/IMU output in private domain with synthetic sensors; component only.",
                        checks=checks, sample_windows=windows, output_mean_yaw_rad_s=means,
                        processed_output_samples=len(samples), adapter=result)
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("overlay", nargs="?", type=Path, help="Optional previous session overlay to verify against a fresh derivation")
    args.add_argument("--output", type=Path)
    parsed = args.parse_args()
    report = run(parsed.overlay.resolve() if parsed.overlay else None)
    message = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if parsed.output:
        parsed.output.write_text(message)
    print(message)
    raise SystemExit(0 if report["passed"] else 2)
