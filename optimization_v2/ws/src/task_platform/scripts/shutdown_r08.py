"""Owned ROS teardown: stop input producers before estimator callback threads.
Physical cancellation/stop confirmation is completed by the worker beforehand.
This changes process shutdown only, not vehicle or tool stopping thresholds.
"""
import signal,time,json
from pathlib import Path

def stop_ros(records,children,forced,out):
 estimators={'icp','fused','wheel_imu'};trace=[]
 for phase,roles,grace in [('producers_and_consumers',lambda r:r not in estimators and r!='server',12.),('estimators_after_inputs_closed',lambda r:r in estimators,20.)]:
  group=[(rec,p) for rec,p in zip(records,children) if roles(rec['role']) and p.poll() is None];start=time.monotonic()
  for rec,p in group:p.send_signal(signal.SIGINT)
  pending=list(group)
  while pending and time.monotonic()-start<grace:
   for rec,p in pending[:]:
    if p.poll() is not None:trace.append(dict(phase=phase,role=rec['role'],exit=p.returncode,elapsed_wall_s=time.monotonic()-start,forced=False));pending.remove((rec,p))
   if pending:time.sleep(.05)
  for rec,p in pending:
   info={}
   for name in ['status','wchan']:
    try:info[name]=(Path('/proc')/str(p.pid)/name).read_text()
    except OSError:pass
   trace.append(dict(phase=phase,role=rec['role'],elapsed_wall_s=time.monotonic()-start,forced=True,process_state=info));forced.append(rec['role']);p.kill();p.wait(timeout=3)
 (out/'ros_shutdown.json').write_text(json.dumps(dict(physical_stop_gate='worker retains original thresholds',phases=trace),indent=2))
