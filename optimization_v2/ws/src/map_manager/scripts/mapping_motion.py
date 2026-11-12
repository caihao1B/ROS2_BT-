"""Operator survey route client; obeys observed health and the frozen S03 arbiter."""
import argparse,signal,time
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from rtabmap_msgs.msg import MapGraph
from rclpy.qos import qos_profile_sensor_data
from s06_common import *
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,required=True);p.add_argument('--case',required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);f=(a.out/'commands.jsonl').open('w');rclpy.init();n=rclpy.create_node('s03_test_driver');pub=n.create_publisher(String,'/s03/request',1)
    st=dict(t=0.,epoch=0,odom=None,limit=None,drive=None,stop=False,manager=None,angle=0.,prev_angle=0.,goal=False,range_t=-100.,front_range=0.,waypoint=0,correction=[0.,0.,0.]);seq=0;last=0.;last_save=-1.
    cfg=yaml.safe_load((BASE/'config/routes/mapping_routes.yaml').read_text())
    points=[];last_live=-1.;circle=cfg['mapping_loop'];radius=circle['radius'];center=np.array(circle['center']);laps=circle['laps'];speed=cfg['speed_mps']
    if a.case=='mapping_full':
        points=[(center+radius*np.array([math.sin(t),-math.cos(t)])).tolist() for t in np.linspace(0,2*math.pi*laps,76)[1:]]
        start=np.array(cfg['spawn_survey'][:2]);outbound=np.array(cfg['mapping_full']['outbound'])-start
        points+=outbound.tolist()
        b=cfg['mapping_full']['zone_b_circle'];bc=np.array(b['center']);br=b['radius']
        points+=[(bc+br*np.array([math.cos(t),math.sin(t)])-start).tolist() for t in np.linspace(math.pi,3*math.pi,65)[1:]]
        if cfg['mapping_full']['return_to_start']:points+=outbound[-2::-1].tolist()+[[0.,0.]]
    def graph(m):
        c=m.map_to_odom;q=c.rotation;st['correction']=[c.translation.x,c.translation.y,float(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2])]
    n.create_subscription(MapGraph,'/s06/rtabmap/mapGraph',graph,10)
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<st['t']:
            st.update(stop=True,goal=False)
        st['t']=t
    def odom(m):xyz,q=pose(m);st['odom']=[*xyz,*euler(q)]
    def enc(m):st['epoch']=json.loads(m.data)['epoch']
    def limit(m):st['limit']=json.loads(m.data)
    def manager(m):st['manager']=json.loads(m.data)
    n.create_subscription(String,'/s06/manager/status',manager,10)
    def drive(m):st['drive']=json.loads(m.data)
    def scan(m):
        rr=np.array(m.ranges);aa=m.angle_min+np.arange(len(rr))*m.angle_increment;xx=rr*np.cos(aa);yy=rr*np.sin(aa);mask=np.isfinite(rr)&(xx>0)&(np.abs(yy)<.68);st['front_range']=float(np.min(xx[mask])) if mask.any() else 8.;st['range_t']=stamp(m)
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(Odometry,'/s05/odometry',odom,10);n.create_subscription(String,'/s03/encoders',enc,qos_profile_sensor_data);n.create_subscription(String,'/s05/safety_limit',limit,10);n.create_subscription(String,'/s03/drive/status',drive,10)
    n.create_subscription(LaserScan,'/s03/front_scan',scan,qos_profile_sensor_data)
    def end(s,f):st['stop']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('fused odom + safety limit; no world pose')
    def send(mode,v,w):
        nonlocal seq
        seq+=1;m=String();m.data=json.dumps(dict(sequence=seq,epoch=st['epoch'],stamp=st['t'],mode=mode,v=v,w=w));pub.publish(m)
    try:
        while not st['stop'] and st['t']<a.seconds:
            rclpy.spin_once(n,timeout_sec=.005);now=time.monotonic()
            if now-last<.025:continue
            last=now;t=st['t']
            if t<last_save:last_save=-1.
            mode='STOP';v=w=0.;reason='NO_HEALTH_PERMISSION';health=st['limit']
            if health and st['manager'] and st['manager']['motion_allowed'] and t-st['manager']['t']<.3 and st['odom'] and t>=9 and t<a.seconds-4 and 0<=t-health['t']<.3 and health['max_speed']>0 and not st['goal']:
                x,y,z,roll,pitch,yaw=st['odom'];cap=health['max_speed'];reason=health['reason']
                if not st['drive'] or not st['drive']['armed']:mode='ARM'
                else:
                    mode='DRIVE'
                    if a.case=='mapping_full':
                        cx,cy,ca=st['correction'];px=cx+math.cos(ca)*x-math.sin(ca)*y;py=cy+math.sin(ca)*x+math.cos(ca)*y;heading=wrap(yaw+ca)
                        while st['waypoint']<len(points) and np.linalg.norm(np.array(points[st['waypoint']])-[px,py])<cfg['arrival_m']:st['waypoint']+=1
                        if st['waypoint']>=len(points):st['goal']=True;mode='STOP'
                        else:
                            delta=np.array(points[st['waypoint']])-[px,py];err=wrap(math.atan2(delta[1],delta[0])-heading)
                            v=min(speed,cap)*max(0.,math.cos(err)) if abs(err)<.7 else 0.
                            w=max(-.35,min(.35,1.1*err))
                    elif a.case=='slope':
                        if x>11.5:st['goal']=True;mode='STOP'
                        else:v=min(.25,cap);w=max(-.15,min(.15,-.6*y-.7*yaw))
                    else:
                        phi=math.atan2(x-center[0],center[1]-y);d=wrap(phi-st['prev_angle']);st['angle']+=d;st['prev_angle']=phi
                        if a.case in ['mapping_loop','weak_texture'] and st['angle']>=2*math.pi*laps:st['goal']=True;mode='STOP'
                        else:
                            target=center+radius*np.array([math.sin(phi+.3),-math.cos(phi+.3)]);delta=target-[x,y];ly=-math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1];curvature=2*ly/max(.1,float(delta@delta));v=min(speed,cap);w=v*max(-.45,min(.45,curvature))
            if mode=='DRIVE' and (t-st['range_t']>.3 or st['front_range']<.35*abs(v)+v*v/(2*.8)+.65):mode='STOP';reason='FRONT_RANGE_PROTECTIVE_STOP'
            if mode!='DRIVE':v=w=0.
            send(mode,v,w)
            if t-last_save>=.1:f.write(json.dumps(dict(t=t,mode=mode,v=v,w=w,reason=reason,health=health,odom=st['odom'],angle=st['angle'],waypoint=st['waypoint'],goal=st['goal']))+'\n');f.flush();last_save=t
            if t-last_live>=1:dump(a.out/'live.json',dict(t=t,goal=st['goal'],odom=st['odom'],reason=reason,waypoint=st['waypoint']));last_live=t
    finally:
        for _ in range(8):send('STOP',0.,0.);rclpy.spin_once(n,timeout_sec=.02)
        f.close();dump(a.out/'result.json',dict(completed=not st['stop'],t=st['t'],goal=st['goal'],loop_angle=st['angle'],waypoint=st['waypoint'],waypoint_count=len(points)));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
