"""Use the qualified input adapter with spray-only, measured standstill gyro calibration."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal

import rclpy
from std_msgs.msg import String
from stationary_gyro_r10 import StationaryYawBias


overlay = Path(os.environ["R05_ADAPTER"])
spec = importlib.util.spec_from_file_location("r10_spray_session_inputs", overlay / "input_adapter.py")
upstream = importlib.util.module_from_spec(spec)
spec.loader.exec_module(upstream)


class SprayInputs(upstream.Inputs):
    def __init__(self, out):
        super().__init__(out)
        self.n.create_subscription(String, "/r07/gate", self.gate, 20)

    def reset_state(self):
        super().reset_state()
        self.stationary_bias = StationaryYawBias()
        self.startup_yaw_bias = None

    def gate(self, message):
        try:
            row = json.loads(message.data)
        except ValueError:
            return
        if isinstance(row, dict):
            self.stationary_bias.observe_gate(row)

    def wheel(self, message):
        t = upstream.stamp(message)
        if t <= self.t + 0.25:
            self.stationary_bias.observe_wheel(t, message.twist.twist.linear.x, message.twist.twist.angular.z)
        super().wheel(message)

    def imu(self, message):
        t = upstream.stamp(message)
        if 4 <= t - self.epoch_start and t <= self.t + 0.25 and self.bias_n > 0:
            if self.startup_yaw_bias is None:
                self.startup_yaw_bias = float(self.bias[2])
            corrected = message.angular_velocity.z - self.startup_yaw_bias
            disputed = self.slipping or self.stationary_bias.wheel_disputed(t, self.icp_data)
            self.bias[2] = self.startup_yaw_bias + self.stationary_bias.update(t, corrected, disputed)
        super().imu(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    inputs = SprayInputs(args.out)
    running = True

    def stop(signum, frame):
        nonlocal running
        running = False

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    try:
        while running:
            rclpy.spin_once(inputs.n, timeout_sec=0.01)
    finally:
        inputs.f.close()
        upstream.dump(args.out / "result.json", dict(
            epoch=inputs.epoch, stationary_bias_samples=inputs.bias_n,
            gyro_bias_estimate=inputs.bias.tolist(), startup_yaw_bias=inputs.startup_yaw_bias,
            stationary_updates=inputs.stationary_bias.updates, incremental_yaw_bias=inputs.stationary_bias.offset,
        ))
        inputs.n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
