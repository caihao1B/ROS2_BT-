"""Bounded passive capture around the ramp: no TF, commands, actions or truth."""
import argparse,json,signal,time,hashlib
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock

ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);args=ap.parse_args()
run=args.run.resolve();out=run/'late_grid_diagnostic';out.mkdir(exist_ok=False)
spec=json.loads((run/'spec.json').read_text())
if spec['task']!='mowing' or (run/'mission_result.json').exists():raise SystemExit('Only the selected active mowing run is eligible')
rclpy.init();node=rclpy.create_node('r10_late_grid_diagnostic');state=dict(t=0.,loc={},risk=None,nav=None,stop=False,last=-100.,count=0);begin=time.monotonic()
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def info(m):
    q=m.info.origin.orientation;p=m.info.origin.position
    return dict(t=stamp(m),frame=m.header.frame_id,width=m.info.width,height=m.info.height,resolution=m.info.resolution,origin=[p.x,p.y,p.z,q.x,q.y,q.z,q.w])
def capture():
    if state['count']>=160 or state['t']-state['last']<3.:return
    try:live=json.loads((run/'live.json').read_text())
    except (OSError,ValueError):return
    if not 7<=live.get('index',-1)<=13:return
    if state['risk'] is None or state['nav'] is None:return
    state['last']=state['t'];index=state['count'];state['count']+=1
    risk=state['risk'];nav=state['nav']
    np.savez(out/f'grid_{index:03}.npz',risk=np.array(risk.data,dtype=np.int8).reshape(risk.info.height,risk.info.width),nav=np.array(nav.data,dtype=np.int8).reshape(nav.info.height,nav.info.width))
    row=dict(t=state['t'],mission_index=live['index'],phase=live.get('phase'),risk=info(risk),nav=info(nav),localization=state['loc'],scope='Passive existing-topic capture; freshness must be checked from stored stamps; no control or task credit')
    (out/f'grid_{index:03}.json').write_text(json.dumps(row)+'\n')
def clock(m):state['t']=m.clock.sec+m.clock.nanosec*1e-9
node.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
node.create_subscription(String,'/r07/localization',lambda m:state.update(loc=json.loads(m.data)),10)
node.create_subscription(OccupancyGrid,'/r07/risk_local',lambda m:state.update(risk=m),2)
node.create_subscription(OccupancyGrid,'/local_costmap/costmap',lambda m:state.update(nav=m),2)
signal.signal(signal.SIGINT,lambda *_:state.update(stop=True));signal.signal(signal.SIGTERM,lambda *_:state.update(stop=True))
metadata=dict(source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),run=str(run),subscriptions=['/clock','/r07/localization','/r07/risk_local','/local_costmap/costmap'],control_publishers=[],truth_input=False,maximum_frames=160,minimum_interval_sim_s=3.,selected_segment_indices=[7,13])
(out/'observer.json').write_text(json.dumps(metadata,indent=2)+'\n')
try:
    while rclpy.ok() and not state['stop'] and time.monotonic()-begin<spec['wall_budget']:
        if (run/'mission_result.json').exists() or (run/'runtime.json').exists():break
        rclpy.spin_once(node,timeout_sec=.1);capture()
finally:
    (out/'result.json').write_text(json.dumps(dict(frames=state['count'],last_sim_t=state['t'],wall_seconds=time.monotonic()-begin,controlled_stop=state['stop'],scope='Diagnostic capture only'))+'\n')
    node.destroy_node();rclpy.shutdown()
