"""Integrate estimated body twist into a continuous local frame; no truth or pose snapping.

robot_localization estimates twist/tilt. Its unanchored position has cross-covariance
corrections even for velocity-only inputs. The public odom uses kinematic integration;
raw EKF pose remains available for audit. Both WI and fused comparisons use this output.
"""
import argparse,copy,signal
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from support import *
class Integrator:
    def __init__(self):self.reset()
    def reset(self):self.t=None;self.xyz=np.zeros(3);self.yaw=0.;self.cov=np.eye(6)*1e-6
    def update(self,m):
        t=stamp(m)
        if self.t is not None and t<self.t:self.reset()
        dt=0. if self.t is None else t-self.t
        if dt<0 or dt>.5:dt=0.  # do not bridge an unobserved interval
        self.t=t;roll,pitch,_=euler(quat(m.pose.pose.orientation));w=m.twist.twist.angular
        yaw_rate=(math.sin(roll)*w.y+math.cos(roll)*w.z)/max(.1,math.cos(pitch));dyaw=yaw_rate*dt
        rot=Rotation.from_euler('xyz',[roll,pitch,self.yaw+dyaw/2]);v=m.twist.twist.linear
        self.xyz+=rot.apply([v.x,v.y,v.z])*dt;self.yaw+=dyaw
        tc=np.array(m.twist.covariance).reshape(6,6);r=rot.as_matrix()
        self.cov[:3,:3]+=r@tc[:3,:3]@r.T*dt*dt+np.eye(3)*1e-4*dt
        self.cov[5,5]+=max(tc[5,5],0)*dt*dt+1e-5*dt
        self.cov[3,3]=max(m.pose.covariance[21],1e-6);self.cov[4,4]=max(m.pose.covariance[28],1e-6)
        out=copy.deepcopy(m);out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=self.xyz.tolist();q=Rotation.from_euler('xyz',[roll,pitch,self.yaw]).as_quat();out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=q.tolist();out.pose.covariance=self.cov.reshape(-1).tolist();return out

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('s05_local_odometry');tf=TransformBroadcaster(n);states={k:Integrator() for k in ['fused','wheel_imu']};pubs={k:n.create_publisher(Odometry,t,20) for k,t in [('fused','/s05/odometry'),('wheel_imu','/s05/wheel_imu/odometry')]};now=[0.];epochs=[]
    def reset(reason):
        for s in states.values():s.reset()
        epochs.append(dict(t=now[0],reason=reason))
    def epoch(m):reset(json.loads(m.data))
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<now[0]:reset('CLOCK_REWIND')
        now[0]=t
    def receive(m,key):
        if stamp(m)>now[0]+.25:return
        out=states[key].update(m);pubs[key].publish(out)
        if key=='fused':
            tr=TransformStamped();tr.header=out.header;tr.child_frame_id='base_link';tr.transform.translation.x=out.pose.pose.position.x;tr.transform.translation.y=out.pose.pose.position.y;tr.transform.translation.z=out.pose.pose.position.z;tr.transform.rotation=out.pose.pose.orientation;tf.sendTransform(tr)
    for k in states:n.create_subscription(Odometry,'/s05/raw/'+k,lambda m,key=k:receive(m,key),20)
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(String,'/s05/epoch',epoch,10);stop=[False]
    def end(sig,fr):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('sole public odom TF; integrated estimated twist')
    try:
        while not stop[0]:rclpy.spin_once(n,timeout_sec=.01)
    finally:dump(a.out/'result.json',dict(resets=epochs));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
