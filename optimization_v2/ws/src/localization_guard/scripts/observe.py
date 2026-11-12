"""Online-only trajectories, TF graph, quality and a separate planning projection."""
import argparse,copy,signal,time
from collections import Counter
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from support import *
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('s05_observation_audit');f=(a.out/'observations.jsonl').open('w');s=dict(t=0.,epoch=0,stop=False,last={},errors=[],edges={},owners={},counts=Counter(),health={},paths={k:[] for k in ['wheel','wheel_imu','fused']},last_path={},last_graph=-1.)
    pubs={k:n.create_publisher(RosPath,'/s05/path/'+k,10) for k in s['paths']};plan=n.create_publisher(PoseStamped,'/s05/planning_pose',10)
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<s['t']:
            s['last'].clear();s['last_path'].clear();s['last_graph']=-1.
            f.write(json.dumps(dict(key='clock_rewind',previous=s['t'],t=t))+'\n')
        s['t']=t
    def epoch(m):d=json.loads(m.data);s['epoch']=d['epoch'];s['last'].clear();s['last_path'].clear();[v.clear() for v in s['paths'].values()];f.write(json.dumps(dict(key='epoch',**d))+'\n')
    def odom(m,key):
        t=stamp(m)
        if t>s['t']+.25:return  # discard queued packets from the previous clock epoch
        xyz,q=pose(m);angles=euler(q);pc=np.array(m.pose.covariance).reshape(6,6);tc=np.array(m.twist.covariance).reshape(6,6)
        if t<s['last'].get(key,-1):s['errors'].append(dict(reason='TIME_REWIND_WITHOUT_EPOCH',key=key,t=t))
        s['last'][key]=t;s['counts'][key]+=1
        row=dict(key=key,t=t,epoch=s['epoch'],wall=time.monotonic(),xyz=xyz.tolist(),q=q.tolist(),rpy=angles.tolist(),v=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z],w=[m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z],pose_cov=list(m.pose.covariance),twist_cov=list(m.twist.covariance),cov_min=float(min(np.linalg.eigvalsh((pc+pc.T)/2).min(),np.linalg.eigvalsh((tc+tc.T)/2).min())),frame=m.header.frame_id,child=m.child_frame_id)
        f.write(json.dumps(row,separators=(',',':'))+'\n')
        if key in pubs and t-s['last_path'].get(key,-100)>=.1:
            ps=PoseStamped();ps.header=m.header;ps.pose=m.pose.pose;s['paths'][key].append(ps);s['last_path'][key]=t
            if len(s['paths'][key])>2500:s['paths'][key]=s['paths'][key][-2500:]
            path=RosPath();path.header=m.header;path.poses=s['paths'][key];pubs[key].publish(path)
        if key=='fused':
            p=PoseStamped();p.header=m.header;p.pose=copy.deepcopy(m.pose.pose);p.pose.position.z=0.;p.pose.orientation.x=p.pose.orientation.y=0.;p.pose.orientation.z=math.sin(angles[2]/2);p.pose.orientation.w=math.cos(angles[2]/2);plan.publish(p)
            if round(t*30)%15==0:f.write(json.dumps(dict(key='planning_projection',t=t,epoch=s['epoch'],xyz=[p.pose.position.x,p.pose.position.y,p.pose.position.z],rpy=euler(quat(p.pose.orientation)).tolist(),source_rpy=angles.tolist()))+'\n')
    def tf(m,static=False):
        for tr in m.transforms:
            edge=tr.header.frame_id+'->'+tr.child_frame_id;s['edges'][edge]=dict(parent=tr.header.frame_id,child=tr.child_frame_id,static=static)
            if not static:
                t=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
                if t>s['t']+.25:continue
                k='tf_'+edge
                if t<s['last'].get(k,-1):s['errors'].append(dict(reason='TF_REWIND',t=t,edge=edge))
                s['last'][k]=t;s['counts'][k]+=1
    def health(m):s['health']=json.loads(m.data);f.write(json.dumps(dict(key='health',**s['health']))+'\n')
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(String,'/s05/epoch',epoch,10);n.create_subscription(String,'/s05/safety_limit',health,10)
    for key,topic in [('wheel','/s05/wheel_only'),('wheel_imu','/s05/wheel_imu/odometry'),('fused','/s05/odometry'),('icp','/s05/inputs/icp'),('raw_fused','/s05/raw/fused')]:n.create_subscription(Odometry,topic,lambda m,k=key:odom(m,k),20)
    n.create_subscription(TFMessage,'/tf',tf,100);q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);n.create_subscription(TFMessage,'/tf_static',lambda m:tf(m,True),q)
    audits=[]
    def end(sig,fr):s['stop']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('no scoring truth input')
    try:
        while not s['stop'] and s['t']<a.seconds:
            rclpy.spin_once(n,timeout_sec=.01)
            if s['t']-s['last_graph']>1:
                s['last_graph']=s['t'];owners=[i.node_name for i in n.get_publishers_info_by_topic('/tf')];audits.append(dict(t=s['t'],publishers=owners));dump(a.out/'live.json',dict(t=s['t'],health=s['health'],counts=dict(s['counts']),paths={k:[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in v[::3]] for k,v in s['paths'].items()}))
    finally:
        graph={t:types for t,types in n.get_topic_names_and_types()};f.close();dump(a.out/'result.json',dict(completed=not s['stop'],epoch=s['epoch'],errors=s['errors'],edges=s['edges'],counts=dict(s['counts']),tf_audits=audits,graph=graph,subscriptions={t:[i.node_name for i in n.get_subscriptions_info_by_topic(t)] for t in graph},publishers={t:[i.node_name for i in n.get_publishers_info_by_topic(t)] for t in graph}));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
