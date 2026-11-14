"""Measured terrain + authorized prior boundary. Unknown ground remains unknown."""
import argparse,json,math,signal,copy
from pathlib import Path
import numpy as np
from shapely.geometry import shape
from shapely import contains_xy
from shapely.affinity import rotate,translate
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid,Odometry
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from reference_r07 import stamp,rot
from footprint_r09 import body_polygon
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--config',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);cfg=json.loads(a.config.read_text());allowed=shape(cfg['allowed_prior']).buffer(-cfg['footprint_margin_m']);rclpy.init();n=rclpy.create_node('r09_observed_risk',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);pub=n.create_publisher(OccupancyGrid,'/r07/risk_local',2);health=n.create_publisher(String,'/r07/safety',10);st=dict(t=0.,stop=False,loc=None,terrain=None,odom=None,worker=None);log=(a.out/'risk.jsonl').open('w')
    n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    n.create_subscription(String,'/r07/localization',lambda m:st.update(loc=json.loads(m.data)),10)
    n.create_subscription(String,'/s09/terrain/health',lambda m:st.update(terrain=json.loads(m.data)),10)
    n.create_subscription(Odometry,'/s05/odometry',lambda m:st.update(odom=m),20)
    n.create_subscription(String,'/r07/mission',lambda m:st.update(worker=json.loads(m.data)),10)
    def grid(m):
        t=stamp(m);loc=st['loc'];h=st['terrain'];v=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width).copy()
        valid=bool(loc and loc['valid'] and -.03<=st['t']-loc['t']<.2 and h and h['valid'] and abs(t-h['t'])<.45)
        boundary=False;protective=True;direction=1
        if valid:
            x=m.info.origin.position.x+(np.arange(m.info.width)+.5)*m.info.resolution;y=m.info.origin.position.y+(np.arange(m.info.height)+.5)*m.info.resolution;xx,yy=np.meshgrid(x,y);tr=loc['transform'];co,si=math.cos(tr[2]),math.sin(tr[2]);wx=co*xx-si*yy+tr[0]+28;wy=si*xx+co*yy+tr[1]+14
            v[~contains_xy(allowed,wx,wy)]=100
            from shapely.geometry import Polygon
            body=body_polygon(cfg['vehicle']['footprint'],loc['pose'][0]+28,loc['pose'][1]+14,loc['pose'][2]);boundary=allowed.covers(body)
            # Standard current-footprint clearing cannot erase a measured lethal.
            current=contains_xy(body,wx,wy);v[current&(v<0)]=0
            # Own Ackermann directional stopping envelope, evaluated on the
            # same measured risk grid; native terrain remains the risk source.
            worker=st['worker'] or {};direction=worker.get('direction',1) if -.03<=st['t']-worker.get('t',-100)<.2 else 0
            p=loc['odom'];cs,sn=math.cos(p[2]),math.sin(p[2]);bx=cs*(xx-p[0])+sn*(yy-p[1]);by=-sn*(xx-p[0])+cs*(yy-p[1]);xmin=min(x for x,y in cfg['vehicle']['footprint']);xmax=max(x for x,y in cfg['vehicle']['footprint']);half=max(abs(y) for x,y in cfg['vehicle']['footprint'])+.1
            backward=.6 if direction<=0 else .1;forward=.6 if direction>=0 else .1;envelope=(bx>=xmin-backward)&(bx<=xmax+forward)&(abs(by)<=half);protective=bool(np.any(v[envelope]>=100))

        else:v[:]=-1
        out=copy.deepcopy(m);out.data=v.astype('int8').ravel().tolist();pub.publish(out)
        d=dict(t=t,valid=valid and boundary,boundary_valid=boundary,max_speed=min(cfg['speed_cap_mps'],cfg['reverse_cap_mps'] if direction<0 else cfg['speed_cap_mps']) if valid and boundary and not protective else 0.,reason=('DIRECTIONAL_PROTECTIVE_STOP' if protective else 'OBSERVED_DIRECTIONAL_GEOMETRY') if valid and boundary else 'OBSERVATION_OR_BOUNDARY_STOP',unknown_cells=int((v<0).sum()),lethal_cells=int((v==100).sum()),ground_is_current_measurement=True,terrain_health=h)
        health.publish(String(data=json.dumps(d)));log.write(json.dumps(d)+'\n');log.flush();(a.out/'live.json').write_text(json.dumps(d))
    n.create_subscription(OccupancyGrid,'/s09/terrain/ackermann',grid,2)
    def stop(s,f):st['stop']=True
    for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
    (a.out/'ready').write_text('observed terrain; no hidden world obstacles')
    try:
        while not st['stop']:rclpy.spin_once(n,timeout_sec=.01)
    finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
