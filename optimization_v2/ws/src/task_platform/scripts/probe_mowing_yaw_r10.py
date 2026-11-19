"""Ackermann variant of the isolated live ROS IMU/odometer/gate component experiment."""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile

from overlay_r08 import overlay
from mowing_yaw_candidate_r10 import CANDIDATE_SOURCES, ROOT, SCRIPTS, replace_once


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(archive):
    # Reuse the qualified ROS harness (real node topology, QoS, measurement
    # windows and shutdown) while the mower follows a feasible moving arc.
    qualified = SCRIPTS / 'probe_stationary_gyro_r10.py'
    code = qualified.read_text()
    code = replace_once(code, 'domain = 173', 'domain = 174')
    code = replace_once(code, 'spray_yaw_bias_ros_probe', 'mowing_yaw_bias_ros_probe')
    code = replace_once(code, 'v=0., w=.11 if turning', 'v=.35 if turning else 0., w=.11 if turning')
    code = replace_once(code, 'w.twist.twist.angular.z = .11 if turning else 0.',
                        'w.twist.twist.linear.x = .35 if turning else 0.\n                    w.twist.twist.angular.z = .11 if turning else 0.')
    code = replace_once(code, 'else -.004 if t < 23.', 'else -.003 if t < 23.')
    namespace = dict(__name__='mowing_component_harness', __file__=str(qualified))
    exec(compile(code, str(qualified), 'exec'), namespace)
    with tempfile.TemporaryDirectory(prefix='mower_probe_overlay_') as temp:
        generated = overlay('mowing', Path(temp) / 'overlay')
        actual = archive if archive else generated
        if digest(generated / 'input_adapter.py') != digest(actual / 'input_adapter.py'):
            raise ValueError('Archived mower input adapter differs from the current production derivation')
        result = namespace['exercise'](actual)
        result.update(scope='Isolated real ROS component, synthetic measured inputs. '
                            'Mower overlay, .005 rad/s warm yaw bias, .35 m/s forward and .11 rad/s Ackermann turn, stale STOP gate.',
                      domain=174, input_adapter_sha256=digest(actual / 'input_adapter.py'),
                      bound_sha256={str(f.relative_to(ROOT)):digest(f) for f in
                                    [qualified, Path(__file__).resolve(), *[SCRIPTS / name for name in CANDIDATE_SOURCES]]})
        return result


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--overlay',type=Path)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    report = run(args.overlay.resolve() if args.overlay else None)
    text = json.dumps(report,indent=2)+'\n'
    if args.output:
        args.output.write_text(text)
    print(text,end='')
    raise SystemExit(0 if report['passed'] else 2)
