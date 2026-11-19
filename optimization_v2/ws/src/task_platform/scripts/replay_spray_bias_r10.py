"""Replay observed A1 gate, wheel and IMU logs. No truth, simulator or action credit."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics
from stationary_gyro_r10 import StationaryYawBias


def replay(run):
    windows = [(1338., 1344.), (1391., 1402.)]
    measurements = [[] for _ in windows]
    with ((run / "control/gate.jsonl").open() as gate_file, (run / "inputs/inputs.jsonl").open() as inputs,
          (run / "icp_gate/quality.jsonl").open() as icp_file):
        gates = map(json.loads, gate_file)
        next_gate = next(gates, None)
        icp_samples = map(json.loads, icp_file)
        next_icp = next(icp_samples, None)
        quality = {}
        slipping = False
        tracker = StationaryYawBias()
        updates = Counter()
        for sample in map(json.loads, inputs):
            t = sample.get("t", 0.)
            while next_gate and next_gate["t"] <= t:
                tracker.observe_gate(next_gate)
                next_gate = next(gates, None)
            while next_icp and next_icp["t"] <= t:
                quality = next_icp
                next_icp = next(icp_samples, None)
            if sample.get("key") == "wheel":
                slipping = sample.get("slip_suspected", False)
                tracker.observe_wheel(t, sample["v"], sample["w"])
                continue
            if sample.get("key") != "imu" or t < 4.:
                continue
            initial = sample["gyro"][2]
            before = tracker.updates
            offset = tracker.update(t, initial, slipping or tracker.wheel_disputed(t, quality))
            if tracker.updates > before:
                gate = tracker.previous(tracker.gates, t, .15)
                updates["stationary" if gate and gate[1] else "unsupervised"] += 1
            wheel = tracker.previous(tracker.wheels, t, .08)
            if wheel:
                for store, (start, end) in zip(measurements, windows):
                    if start <= t <= end:
                        store.append((initial - wheel[2], initial - offset - wheel[2]))
    results = []
    for measurements_at_stop, interval in zip(measurements, windows):
        raw, corrected = zip(*measurements_at_stop)
        results.append(dict(interval_sim_s=interval, samples=len(raw),
                            old_mean_gyro_minus_wheel_rad_s=statistics.mean(raw),
                            new_mean_gyro_minus_wheel_rad_s=statistics.mean(corrected)))
    return dict(run=run.name, scope="Offline replay of decimated IMU samples and real recorded gates and encoder. "
                "No global heading ground truth, navigation action or completion credit.",
                updates=dict(updates), measured_stop_windows=results,
                passed=updates["stationary"] > 0 and updates["unsupervised"] == 0 and
                    all(abs(row["new_mean_gyro_minus_wheel_rad_s"]) <
                        abs(row["old_mean_gyro_minus_wheel_rad_s"])/2 for row in results))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = replay(args.run)
    text = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text, end="")
    raise SystemExit(0 if report["passed"] else 2)
