"""Record actual global/local estimates, quality, TF ownership and admission."""
import argparse,signal
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path as ROSPath
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from collections import Counter
from std_msgs.msg import String
from rtabmap_msgs.msg import Info
import common08
from s07_common import *
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seconds',type=float,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('s08_observer');log=(a.out/'observations.jsonl').open('w');st=dict(t=0.,stop=False,status=None,pose=None,odom=None,last_audit=-1.);audits=[];tf_edges=Counter();gid_nodes={};trace=ROSPath();trace.header.frame_id='map';trace_pub=n.create_publisher(ROSPath,'/s07/trace',2);marker_pub=n.create_publisher(Marker,'/s07/state_marker',2)
    def record(row):log.write(json.dumps(row,separators=(',',':'))+'\n')
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<st['t']:st['stop']=True
        st['t']=t
    def string(m,key):
        d=json.loads(m.data);record(dict(key=key,**d))
        if key=='status':st['status']=d
    def pose_msg(m,key):
        xyz,q=pose(m);d=dict(key=key,t=stamp(m),pose=[*xyz.tolist(),*q.tolist()]);record(d);st[key]=d
        if key=='pose' and (not trace.poses or stamp(m)-stamp(trace.poses[-1])>.25):
            item=PoseStamped();item.header=m.header;item.pose=m.pose.pose;trace.header=m.header;trace.poses.append(item);trace_pub.publish(trace)
            marker=Marker();marker.header=m.header;marker.ns='S07_state';marker.id=0;marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD;marker.pose=m.pose.pose;marker.pose.position.z+=2.;marker.scale.z=.55;marker.color.a=1.;status=st['status'] or {};marker.color.g=1. if status.get('navigation_allowed') else .45;marker.color.r=.2 if status.get('navigation_allowed') else 1.;marker.text=status.get('state','INITIALIZING')+' | admitted='+str(status.get('navigation_allowed',False));marker_pub.publish(marker)
    def tf_message(m):
        for tr in m.transforms:tf_edges[(tr.header.frame_id,tr.child_frame_id)]+=1
    n.create_subscription(TFMessage,'/tf',tf_message,100)
    def info(m):record(dict(key='info',t=stamp(m),loop=m.loop_closure_id,proximity=m.proximity_detection_id,stats=dict(zip(m.stats_keys,m.stats_values))))
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(Info,'/s07/rtabmap/info',info,20)
    for topic,key in [('/s08/status','status'),('/s07/localization/evidence','evidence'),('/s05/safety_limit','health'),('/s08/gnss','gnss'),('/s08/map','map')]:n.create_subscription(String,topic,lambda m,k=key:string(m,k),20)
    n.create_subscription(PoseWithCovarianceStamped,'/s08/pose',lambda m:pose_msg(m,'pose'),20);n.create_subscription(Odometry,'/s05/odometry',lambda m:pose_msg(m,'odom'),30)
    def end(s,f):st['stop']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('online observer; no truth subscriptions')
    try:
        while not st['stop'] and st['t']<a.seconds:
            rclpy.spin_once(n,timeout_sec=.01)
            if st['t']-st['last_audit']>=1:
                st['last_audit']=st['t'];gid_nodes.update({bytes(x.endpoint_gid).hex():x.node_name for x in n.get_publishers_info_by_topic('/tf')});audits.append(dict(t=st['t'],tf=[x.node_name for x in n.get_publishers_info_by_topic('/tf')],actuator=[x.node_name for x in n.get_publishers_info_by_topic('/s03/actuator/request')]));log.flush();dump(a.out/'live.json',dict(t=st['t'],status=st['status'],pose=st['pose'],odom=st['odom']))
    finally:
        topics=n.get_topic_names_and_types();log.close();dump(a.out/'result.json',dict(completed=not st['stop'],t=st['t'],audits=audits,tf_edges=[dict(parent=k[0],child=k[1],count=v) for k,v in tf_edges.items()],subscriptions={name:[x.node_name for x in n.get_subscriptions_info_by_topic(name)] for name,typ in topics}));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
