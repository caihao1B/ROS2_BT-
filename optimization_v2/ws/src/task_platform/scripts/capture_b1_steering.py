"""Read-only steering trace for a known current Gazebo run. Never publishes commands."""
import argparse
import json
import signal
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('session',type=Path)
    args=ap.parse_args()
    out=args.session.resolve()
    if (out/'runtime.json').exists() or not (out/'mission.json').is_file():
        raise ValueError('A known active connector session is required')
    rclpy.init()
    node=rclpy.create_node('b1_read_only_front_steering_recorder')
    state={'gate':{},'segment':None,'last':-1.,'stop':False}
    def stop(*_):
        state['stop']=True
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    file=out/'measured_front_steering.jsonl'
    with file.open('x') as stream:
        def gate(msg):
            state['gate']=json.loads(msg.data)
        def segment(msg):
            state['segment']=json.loads(msg.data).get('segment_id')
        def encoder(msg):
            e=json.loads(msg.data)
            if e['t']-state['last']<.04 or len(e.get('q',[]))<4:
                return
            state['last']=e['t']
            g=state['gate']
            row=dict(t=e['t'],segment=state['segment'],front_rad=e['q'][2:4],
                     limited_v=e.get('limited_v'),limited_w=e.get('limited_w'),request_v=g.get('v'),request_w=g.get('w'),
                     request_t=g.get('t'),actuator_reason=e.get('reason'),armed=e.get('armed'))
            stream.write(json.dumps(row,separators=(',',':'))+'\n')
            stream.flush()
        node.create_subscription(String,'/s03/encoders',encoder,qos_profile_sensor_data)
        node.create_subscription(String,'/r07/gate',gate,10)
        node.create_subscription(String,'/r07/mission',segment,10)
        deadline=time.monotonic()+1500.
        while not state['stop'] and not (out/'runtime.json').exists() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.2)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__=='__main__':
    main()
