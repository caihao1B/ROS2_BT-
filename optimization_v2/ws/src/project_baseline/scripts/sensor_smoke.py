#!/usr/bin/env python3
"""Bounded stationary native sensor check. This evaluator publishes no commands."""
import argparse, collections, json, math, os, pathlib, signal, subprocess, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu, JointState, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

p = argparse.ArgumentParser()
p.add_argument('--out', type=pathlib.Path, required=True)
p.add_argument('--seconds', type=float, default=35)
p.add_argument('--world', default='')
p.add_argument('--sim-seconds', type=float, default=None)
p.add_argument('--gui', action='store_true')
a = p.parse_args()
assert 15 <= a.seconds <= 600
assert a.sim_seconds is None or 0 < a.sim_seconds <= 1800
a.out.mkdir(parents=True, exist_ok=False)
os.environ.setdefault('ROS_DOMAIN_ID', '118')
os.environ['GZ_PARTITION'] = 'native_stationary_' + str(os.getpid())
cmd = ['ros2', 'launch', 'dino_sensors', 'sensors.launch.py',
       'dual_lidar:=true', 'validation_world:=true', 'enable_ekf:=true']
if a.world:
    cmd += ['world_file:=' + str(pathlib.Path(a.world).resolve())]
log = (a.out / 'launch.log').open('w')
proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
gui = None
if a.gui:
    project=pathlib.Path(os.environ['DINO_PROJECT_ROOT'])
    config=project/'native_20261108/ros2_ws/install/dino_description/share/dino_description/config/gui.config'
    gui_config=a.out/'gui.config'
    gui_config.write_text(config.read_text().replace('5 -5 3.4 0 0.37 2.35619449','28 -38 34 0 0.62 2.206'))
    gui=subprocess.Popen(['gz','sim','--force-version','8','-g','--render-engine-gui','ogre','--gui-config',str(gui_config)],stdout=log,stderr=subprocess.STDOUT)
result = {'command': cmd, 'domain': int(os.environ['ROS_DOMAIN_ID']), 'partition': os.environ['GZ_PARTITION'],
          'software_rendering': os.environ.get('DINO_SOFTWARE_RENDERING','0'),
          'command_publications_by_evaluator': 0, 'started_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())}
rclpy.init()
n = Node('native_stationary_evaluator')
counts = collections.Counter()
sample_times={}; scan_points={}
scans = {}; clocks = []; odom = []; commands = []; tf_sources = set(); frames = set()
subscriptions = []
def subscribe(kind, topic, callback):
    def receive(message):
        counts[topic] += 1
        sample_times.setdefault(topic,[]).append(time.monotonic())
        callback(message)
    subscriptions.append(n.create_subscription(kind, topic, receive, qos_profile_sensor_data))
def scan(topic, m):
    finite = [float(x) for x in m.ranges if math.isfinite(x)]
    scans[topic] = {'samples':len(m.ranges),'finite':len(finite),'frame':m.header.frame_id,
                   'min':min(finite) if finite else None, 'max':max(finite) if finite else None}
    scan_points[topic]=[[i*m.angle_increment+m.angle_min,float(x)] for i,x in enumerate(m.ranges) if math.isfinite(x)]
for topic in ['/dino/lidar/front/scan','/dino/lidar/rear/scan',
              '/dino/lidar/front/scan_filtered','/dino/lidar/rear/scan_filtered']:
    subscribe(LaserScan, topic, lambda m,t=topic:scan(t,m))
subscribe(Clock, '/clock', lambda m:clocks.append((time.monotonic(),m.clock.sec+m.clock.nanosec*1e-9)))
subscribe(Imu, '/dino/imu', lambda m:frames.add(m.header.frame_id))
subscribe(JointState, '/joint_states', lambda m:None)
subscribe(NavSatFix, '/dino/gnss/fix', lambda m:None)
subscribe(Odometry, '/dino/odometry/filtered', lambda m:odom.append((m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x)))
subscribe(Twist, '/dino/drive/request', lambda m:commands.append((m.linear.x,m.angular.z)))
def tf(m, info):
    result['tf_message_info_fields']=list(info) if isinstance(info,dict) else []
    for t in m.transforms:
        if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
            counts['odom_to_base_tf'] += 1
subscriptions.append(n.create_subscription(TFMessage,'/tf',tf,qos_profile_sensor_data))
def interrupted(sig,frame): raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupted)
signal.signal(signal.SIGINT,interrupted)
def capture_window():
    # Capture only the Gazebo window rectangle, never the whole desktop.
    import re
    from PIL import ImageGrab
    listing=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
    for line in listing.splitlines():
        if '"gz-sim-gui"' not in line: continue
        wid=line.strip().split()[0]
        info=subprocess.check_output(['xwininfo','-id',wid],text=True)
        def num(key):return int(re.search(key+r':\s*(-?\d+)',info).group(1))
        x,y,w,h=num('Absolute upper-left X'),num('Absolute upper-left Y'),num('Width'),num('Height')
        if w>100 and h>100:
            # Limit to the 3D viewport, excluding desktop notifications/tool panels.
            ImageGrab.grab(bbox=(x+4,y+160,x+int(w*.65)-4,y+h-45)).save(a.out/'gazebo_window.png')
            return True
    return False
try:
    started=time.monotonic(); captured=False
    deadline = time.monotonic()+a.seconds
    while time.monotonic()<deadline and proc.poll() is None:
        rclpy.spin_once(n, timeout_sec=.05)
        if a.gui and not captured and time.monotonic()-started>12:
            try: captured=capture_window()
            except Exception as exc: result['screenshot_error']=str(exc)
        if a.sim_seconds and clocks and clocks[-1][1]>=a.sim_seconds: break
    result['publishers'] = {t:n.count_publishers(t) for t in ['/clock','/dino/odometry/filtered','/dino/drive/request']}
    result['counts'] = dict(counts)
    result['received_wall_hz']={k:(len(t)-1)/(t[-1]-t[0]) for k,t in sample_times.items() if len(t)>1}
    result['observed_sim_seconds']=clocks[-1][1] if clocks else None
    result['observed_wall_seconds']=time.monotonic()-started
    result['gui_enabled']=a.gui
    result['screenshot_captured']=captured
    result['clock_samples']=clocks[::max(1,len(clocks)//100)]
    (a.out/'last_scan_points.json').write_text(json.dumps(scan_points))
    result['last_scans'] = scans
    result['tf_publisher_nodes'] = sorted(i.node_name for i in n.get_publishers_info_by_topic('/tf'))
    result['odometry_publisher_nodes'] = sorted(i.node_name for i in n.get_publishers_info_by_topic('/dino/odometry/filtered'))
    result['tf_authority_check_method']='Observed TF edge + publisher topology + audited source/URDF contract; this rclpy does not expose per-message publisher GID.'
    result['odom_span_m'] = math.dist(odom[0][:2],odom[-1][:2]) if len(odom)>1 else None
    result['max_abs_forward_velocity_mps'] = max((abs(x[2]) for x in odom),default=None)
    result['clock_rtf'] = (clocks[-1][1]-clocks[0][1])/(clocks[-1][0]-clocks[0][0]) if len(clocks)>1 else None
    result['checks'] = {
        'clock_advances':len(clocks)>100 and clocks[-1][1]>clocks[0][1] and all(b[1]>=aa[1] for aa,b in zip(clocks,clocks[1:])),
        'dual_raw_scan':all(counts[t]>=10 for t in ['/dino/lidar/front/scan','/dino/lidar/rear/scan']),
        'dual_filtered_scan':all(counts[t]>=5 for t in ['/dino/lidar/front/scan_filtered','/dino/lidar/rear/scan_filtered']),
        'scan_data_finite':len(scans)==4 and all(v['samples']==1440 and v['finite']>20 for v in scans.values()),
        'imu_received':counts['/dino/imu']>=100,
        'gnss_received':counts['/dino/gnss/fix']>=100,
        'joints_received':counts['/joint_states']>=100,
        'local_odometry_received':len(odom)>=100,
        'odom_tf_topology_matches_single_estimator':result['tf_publisher_nodes']==['dino_local_estimator_io','robot_state_publisher'] and result['odometry_publisher_nodes']==['dino_local_estimator_io'] and counts['odom_to_base_tf']>=100,
        'unique_authorities':all(v==1 for v in result['publishers'].values()),
        'stationary':len(odom)>1 and result['odom_span_m']<.03 and result['max_abs_forward_velocity_mps']<.03,
        'only_zero_actuator_requests':bool(commands) and all(abs(v)+abs(w)<1e-9 for v,w in commands),
        'launch_survived':proc.poll() is None,
    }
    if a.gui:result['checks']['gui_survived']=gui is not None and gui.poll() is None
except KeyboardInterrupt:
    result['interrupted']=True
except Exception as error:
    result['probe_error']=repr(error)
    raise
finally:
    if gui and gui.poll() is None:
        gui.send_signal(signal.SIGINT)
        try:gui.wait(timeout=8)
        except subprocess.TimeoutExpired:gui.terminate();gui.wait(timeout=5)
    if proc.poll() is None:
        # ROS launch propagates this once to its children; group SIGINT duplicates it.
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            result['forced_cleanup']=True
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill();proc.wait(timeout=5)
    result['launch_exit_code']=proc.returncode
    log.close();n.destroy_node();rclpy.try_shutdown()
    result.setdefault('counts',dict(counts))
    result['resource_error_lines']=[line for line in (a.out/'launch.log').read_text(errors='replace').splitlines()
                                  if any(marker in line for marker in ['Unable to find file','Could not resolve file','Unable to load mesh','Failed to load system plugin'])]
    if result.get('checks'):
        result['checks']['no_missing_runtime_resources']=not result['resource_error_lines']
    result['status']='PASS' if result.get('checks') and all(result['checks'].values()) and not result.get('forced_cleanup') and proc.returncode==0 else 'FAIL'
    (a.out/'result.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['status']=='PASS' else 1)
