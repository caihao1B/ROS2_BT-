"""Strict adapter around S15 bounded worker transport; no second publisher."""
from common16 import *
from common15 import add14
once=add14()
source=(P15/'scripts/prediction_node15.py').read_text()
source=once(source,"choices=['no_prediction','imm','learned']","choices=['full','event','cache']")
source=once(source,"str(PKG/'scripts/model_worker15.py')","str(BASE/'ws/src/experience_memory/scripts/worker16.py')")
source=once(source,"cfg=yaml.safe_load((BASE/'config/prediction.yaml').read_text())","cfg=yaml.safe_load((BASE/'config/prediction.yaml').read_text());cfg['inference_period_sim_s']=0.")
source=once(source,"    n.create_subscription(String,'/s09/objects',lambda m:queue.append(json.loads(m.data)),30)",'''    from features16 import CausalContext
    context_state=CausalContext()
    def status(m):
        j=json.loads(m.data);context_state.add_status(j)
    def odometry(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        context_state.add_robot(dict(t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z))
    def objects(m):
        j=json.loads(m.data);j['context']=context_state.at(j['t']);queue.append(j)
    from nav_msgs.msg import Odometry
    n.create_subscription(String,'/s08/status',status,20)
    n.create_subscription(Odometry,'/s05/odometry',odometry,30)
    n.create_subscription(String,'/s09/objects',objects,30)''')
# Display-only: MarkerArray does not defer future-stamped TF lookups like PointCloud.
# Packet t / model inputs / all safety deadlines stay unchanged. Zero = latest TF.
source=once(source,'from geometry_msgs.msg import Point','from geometry_msgs.msg import Point\nfrom builtin_interfaces.msg import Time as MarkerTime')
marker_stamp='m.header.stamp=n.get_clock().now().to_msg()'
if source.count(marker_stamp)!=2:raise ValueError('Unexpected S15 marker layout')
source=source.replace(marker_stamp,'m.header.stamp=MarkerTime()')
exec(compile(source,str(P15/'scripts/prediction_node15.py'),'exec'),globals())
