"""Declared test-only delayed sensor delivery and read-only terrain recorder."""
from common17 import *
import argparse,signal
from collections import deque

def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--delay',action='store_true');a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    if os.environ.get('ROS_DOMAIN_ID')!='119':raise ValueError('OBSERVED_INPUT_DOMAIN_REQUIRED')
    rclpy.init();n=rclpy.create_node('s17_fault_input_recorder');state=dict(t=0.,start=None,moving_since=None,perception={},stop=False);pending=deque();log=(a.out/'delivery.jsonl').open('w');terrain=(a.out/'terrain.jsonl').open('w')
    pub=n.create_publisher(PointCloud2,'/s17/delayed/perception',qos_profile_sensor_data) if a.delay else None
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def status(m):
        d=json.loads(m.data)
        if not a.delay:return
        if state['start'] is not None:return
        perception=state['perception'];ready=d.get('navigation_allowed') and abs(d.get('v',0))>.03 and perception.get('geometry_fresh') and perception.get('boundary_valid') and perception.get('max_speed',0)>0 and 0<=d['t']-perception.get('t',-100)<.45
        if not ready:state['moving_since']=None
        elif state['moving_since'] is None:state['moving_since']=d['t']
        elif d['t']-state['moving_since']>=1.5:
            state['start']=d['t'];dump(a.out/'injection17.json',dict(t=d['t'],end=d['t']+20.,observed_v=d['v'],stable_motion_since=state['moving_since'],perception_t=perception['t'],delay_s=1.2,scope='acquisition stamp preserved; motion verified at injection'))
    def receive(m):
        delay=1.2 if state['start'] is not None and state['start']<=state['t']<state['start']+20 else 0.
        pending.append((state['t']+delay,m,delay))
        # Delivery order and original acquisition headers are preserved.
        if len(pending)>40:raise RuntimeError('DELAY_QUEUE_BOUND')
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<state['t']:pending.clear();state['start']=None
        state['t']=t
        while pending and pending[0][0]<=t:
            due,msg,delay=pending.popleft();pub.publish(msg)
            log.write(json.dumps(dict(acquired=stamp(msg),queued_at=due-delay,delivered=t,requested_delay=delay,original_header_preserved=True))+'\n');log.flush()
    def cells(m):
        d=json.loads(m.data);terrain.write(json.dumps(d,separators=(',',':'))+'\n');terrain.flush()
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(String,'/s08/status',status,20);n.create_subscription(String,'/s09/safety',lambda m:state.update(perception=json.loads(m.data)),20);n.create_subscription(String,'/s09/terrain/cells',cells,2)
    if a.delay:n.create_subscription(PointCloud2,'/s04/perception/points',receive,qos_profile_sensor_data)
    def stop(s,f):state['stop']=True
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,stop)
    (a.out/'ready').write_text('Test input boundary; no control or TF output')
    try:
        while not state['stop']:rclpy.spin_once(n,timeout_sec=.01)
    finally:log.close();terrain.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
