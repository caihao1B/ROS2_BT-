"""Bounded graph/statistics observer. All observations are online, never truth."""
import argparse,signal,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from rtabmap_msgs.msg import Info,MapGraph
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from s06_common import *
def graph_data(m):
    def p(x):return [x.position.x,x.position.y,x.position.z,x.orientation.x,x.orientation.y,x.orientation.z,x.orientation.w]
    def tr(x):return [x.translation.x,x.translation.y,x.translation.z,x.rotation.x,x.rotation.y,x.rotation.z,x.rotation.w]
    return dict(t=stamp(m),ids=list(m.poses_id),poses=[p(v) for v in m.poses],links=[dict(source=x.from_id,target=x.to_id,type=x.type,transform=tr(x.transform),information=list(x.information)) for x in m.links],map_to_odom=tr(m.map_to_odom))
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seconds',type=float,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('s06_mapping_observer');st=dict(t=0.,stop=False,ref=0,loops=[],graph_count=0,last_save=-10.,last_audit=-10.,saved_loop_count=0)
    log=(a.out/'observations.jsonl').open('w');graphs=(a.out/'graphs.jsonl').open('w');audits=[]
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<st['t']:st['stop']=True;st['clock_rewind']=True
        st['t']=t
    def info(m):
        row=dict(key='info',t=stamp(m),received_at_clock=st['t'],ref=m.ref_id,loop=m.loop_closure_id,proximity=m.proximity_detection_id,stats=dict(zip(m.stats_keys,m.stats_values)));log.write(json.dumps(row)+'\n');log.flush();st['ref']=m.ref_id
        if m.loop_closure_id or m.proximity_detection_id:st['loops'].append(dict(t=row['t'],ref=row['ref'],loop=row['loop'],proximity=row['proximity']))
    def graph(m):
        d=graph_data(m);dump(a.out/'graph.json',d);st['graph_count']+=1
        loops=sum(x['type'] in [1,2] for x in d['links'])
        if st['t']-st['last_save']>5 or loops!=st['saved_loop_count']:
            graphs.write(json.dumps(d,separators=(',',':'))+'\n');graphs.flush();st['last_save']=st['t'];st['saved_loop_count']=loops
    def odom(m):
        p,q=pose(m);log.write(json.dumps(dict(key='odom',t=stamp(m),xyz=p.tolist(),q=q.tolist(),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z))+'\n')
    def status(m,key):log.write(json.dumps(dict(key=key,**json.loads(m.data)))+'\n')
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(Info,'/s06/rtabmap/info',info,20);n.create_subscription(MapGraph,'/s06/rtabmap/mapGraph',graph,10);n.create_subscription(Odometry,'/s05/odometry',odom,30)
    for topic,key in [('/s05/safety_limit','health'),('/s06/manager/status','lifecycle')]:n.create_subscription(String,topic,lambda m,k=key:status(m,k),10)
    def end(s,f):st['stop']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('mapping observation')
    try:
        while not st['stop'] and st['t']<a.seconds:
            rclpy.spin_once(n,timeout_sec=.01)
            if st['t']-st['last_audit']>1:
                st['last_audit']=st['t'];audit=dict(t=st['t'],tf=[x.node_name for x in n.get_publishers_info_by_topic('/tf')],actuator=[x.node_name for x in n.get_publishers_info_by_topic('/s03/actuator/request')]);audits.append(audit);dump(a.out/'live.json',dict(t=st['t'],ref=st['ref'],loops=st['loops'],audit=audit))
    finally:
        topics=n.get_topic_names_and_types();log.close();graphs.close();dump(a.out/'result.json',dict(completed=not st['stop'],**st,audits=audits,subscriptions={k:[x.node_name for x in n.get_subscriptions_info_by_topic(k)] for k,v in topics}));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
