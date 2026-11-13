"""Stereo VO contributes bounded increments only; never absolute poses or TF."""
from common17 import *
import argparse,signal,numpy as np,copy
from collections import OrderedDict,deque
from scipy.spatial.transform import Rotation

class Window:
    def __init__(self):self.rows=deque()
    def update(self,t,p,q,cov,valid):
        cov=np.asarray(cov,float);cov=np.diag(cov) if cov.shape==(6,) else cov
        if not valid or cov.shape!=(6,6) or not np.isfinite(np.r_[t,p,q,cov.ravel()]).all() or not .5<np.linalg.norm(q)<1.5:
            self.rows.clear();return None
        if not np.allclose(cov,cov.T,atol=1e-9) or np.linalg.eigvalsh(cov).min()<-1e-9:
            self.rows.clear();return None
        if self.rows and (t<=self.rows[-1][0] or t-self.rows[-1][0]>.6):self.rows.clear()
        self.rows.append((t,np.array(p),np.array(q),np.array(cov)))
        while len(self.rows)>2 and self.rows[1][0]<t-.8:self.rows.popleft()
        a,position,orientation,variance=self.rows[0];span=t-a
        if span<.5:return None
        v=Rotation.from_quat(q).inv().apply(np.array(p)-position)/span
        w=(Rotation.from_quat(orientation).inv()*Rotation.from_quat(q)).as_rotvec()/span
        if np.linalg.norm(v)>=1.2 or np.linalg.norm(w)>=.8:
            self.rows.clear();return None
        # Two endpoint estimates are correlated by VO. A factor of two bounds
        # the unknown cross-correlation contribution; do not assume independent
        # poses and thereby claim an overconfident velocity measurement.
        R=Rotation.from_quat(q).inv().as_matrix();J=np.zeros((6,6));J[:3,:3]=R;J[3:,3:]=R
        covariance=2*J@(cov+variance)@J.T/span**2+np.diag([.0025]*3+[.0004]*3)
        return v,w,covariance

def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from rtabmap_msgs.msg import OdomInfo
    from rosgraph_msgs.msg import Clock
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('s17_visual_increment_gate');pub=n.create_publisher(TwistWithCovarianceStamped,'/s17/inputs/visual_increment',10);pending=OrderedDict();infos=OrderedDict();window=Window();state=dict(t=0.,stop=False)
    log=(a.out/'quality.jsonl').open('w');info_log=(a.out/'frames.jsonl').open('w')
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def drain():
        for t in list(pending):
            if t not in infos:continue
            o=pending.pop(t);i=infos.pop(t);p=o.pose.pose;xyz=[p.position.x,p.position.y,p.position.z];q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w];C=np.array(o.pose.covariance).reshape(6,6)
            valid=not i.lost and i.inliers>=20 and i.matches>=20 and i.inliers/max(1,i.matches)>=.2 and np.isfinite(C).all() and np.diag(C).min()>=0 and np.diag(C).max()<100 and -.05<=state['t']-t<=.5
            inc=window.update(t,xyz,q,C,valid)
            if inc is not None:
                v,w,cov=inc;m=TwistWithCovarianceStamped();m.header=copy.deepcopy(o.header);m.header.frame_id='base_link';m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z=v.tolist();m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z=w.tolist()
                m.twist.covariance=cov.ravel().tolist()
                pub.publish(m)
            log.write(json.dumps(dict(t=t,pose_valid=bool(valid),increment_valid=inc is not None,lost=i.lost,inliers=i.inliers,matches=i.matches,compute_s=i.time_estimation))+'\n');log.flush()
        for queue in [pending,infos]:
            while len(queue)>8:queue.popitem(last=False)
    def odom(m):pending[round(stamp(m),6)]=m;drain()
    def info(m):
        t=stamp(m);info_log.write(json.dumps(dict(t=t,lost=m.lost,inliers=m.inliers,matches=m.matches,compute_s=m.time_estimation))+'\n');info_log.flush();infos[round(t,6)]=m;drain()
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<state['t']:pending.clear();infos.clear();window.rows.clear()
        state['t']=t
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(Odometry,'/s17/visual/raw',odom,qos_profile_sensor_data);n.create_subscription(OdomInfo,'/s17/visual/info',info,qos_profile_sensor_data)
    def stop(s,f):state['stop']=True
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,stop)
    (a.out/'ready').write_text('No absolute pose feedback and no TF publisher')
    try:
        while not state['stop']:rclpy.spin_once(n,timeout_sec=.02)
    finally:log.close();info_log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
