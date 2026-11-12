"""Stationary sensor survey: no active lease, no navigation or tool goal."""
from common14 import *
import argparse,signal
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    for name in ['vehicle','controller','case']:ap.add_argument('--'+name,required=True)
    ap.add_argument('--seconds',type=float,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);n=rclpy.create_node('s13_action_client');pub=n.create_publisher(String,'/s13/lease',1)
    st=dict(t=0.,stop=False,tool={},odom=None);revision=plan()['plan_revision']
    n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    n.create_subscription(Odometry,'/s05/odometry',lambda m:st.update(odom=m),20)
    n.create_subscription(String,'/s10/action/state',lambda m:st.update(tool=json.loads(m.data)),20)
    def stop(s,f):st['stop']=True
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,stop)
    def publish():pub.publish(String(data=json.dumps(dict(t=st['t'],active=False,operation='STOP',action_id='STATIONARY_SURVEY',plan_revision=revision))))
    n.create_timer(.03,publish);(a.out/'ready').write_text('stationary independent sensor survey')
    try:
        end=76 if a.case.startswith('regional_probe_fault') else 22
        while not st['stop'] and st['t']<min(end,a.seconds-8):rclpy.spin_once(n,timeout_sec=.02)
    finally:
        publish();o=st['odom'];stopped=bool(o and abs(o.twist.twist.linear.x)<.01 and abs(o.twist.twist.angular.z)<.02 and st['tool'].get('stopped_confirmed'))
        dump(a.out/'shutdown.json',dict(state='STOPPED' if stopped else 'UNCONFIRMED',tool_on_requested=False))
        dump(a.out/'result.json',dict(t=st['t'],completed=stopped and not st['stop'],scope='STATIONARY_SENSOR_SURVEY_NO_WORK'))
        n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
