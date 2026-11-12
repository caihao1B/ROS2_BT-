"""Regional frozen-cloud localization from current lidar and relative odometry.
No GNSS/truth subscriber; deployment assets enter provenance hashes only,
never the pose matcher or its initial estimate.
"""
from common14 import *
from binding14 import region,ATLAS
from regional_math14 import SurfaceMap,rotation
import argparse,signal
from collections import deque
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
sys.path.insert(0,str(BASE/'ws/src/map_manager/scripts'))
from mapping_analysis import ply

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--request',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    atlas,reg=region();request=read(a.request);base=read(BASE/'maps/frozen/manifest.json')
    if any(request.get(k)!=base[k] for k in ['map_id','revision','frame_epoch','calibration_revision']):raise ValueError('SOURCE_MAP_REQUEST_MISMATCH')
    if reg['cloud_frame']!='site_enu':raise ValueError('REGIONAL_CLOUD_FRAME')
    reference=SurfaceMap(ply(ROOT/reg['cloud_path']));reference.prepare_normals();lo,hi=np.array(reg['site_extent_xy']);center=np.array(reg['registered_center_xy']);param=np.array(reg['parameter_covariance'])
    signatures={str(p):(p.stat().st_ino,p.stat().st_size,p.stat().st_mtime_ns,p.stat().st_ctime_ns) for p in [ATLAS,*[ROOT/k for k in atlas['bindings']]]}
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);n=rclpy.create_node('s14_regional_localizer');pub=n.create_publisher(String,'/s07/localization/evidence',10);contract=n.create_publisher(String,'/s07/map_contract',10)
    clouds=deque(maxlen=4);odoms=deque(maxlen=100);state=dict(t=0.,stop=False,hint=None,previous=None,contract_valid=True);f=(a.out/'evidence.jsonl').open('w');heart=0.;stat=0.
    n.create_subscription(Clock,'/clock',lambda m:state.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    n.create_subscription(PointCloud2,'/s07/inputs/cloud',lambda m:clouds.append((time.monotonic(),m)),5)
    n.create_subscription(Odometry,'/s07/inputs/odom',lambda m:odoms.append(m),30)
    def hint(m):
        if m.header.frame_id!='map':return
        p=m.pose.pose;state['hint']=[p.position.x,p.position.y,yaw(p.orientation)];state['previous']=None
    n.create_subscription(PoseWithCovarianceStamped,'/s07/rtabmap/initialpose',hint,10)
    def end(s,f):state['stop']=True
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,end)
    (a.out/'ready').write_text('registered frozen-cloud matching; no GNSS/truth subscriber')
    try:
        while not state['stop']:
            rclpy.spin_once(n,timeout_sec=.005);wall=time.monotonic()
            if wall-stat>.5:
                stat=wall
                for name,signature in signatures.items():
                    try:s=Path(name).stat();same=(s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)==signature
                    except OSError:same=False
                    if not same:state['contract_valid']=False
            if wall-heart>.15:
                heart=wall;contract.publish(String(data=json.dumps(dict(t=state['t'],valid=state['contract_valid'],reason='REGIONAL_ATLAS_VERIFIED' if state['contract_valid'] else 'REGIONAL_INPUT_CHANGED',map_id=atlas['map_id'],frame_epoch=atlas['frame_epoch'],revision=atlas['revision']))))
            if not clouds or not odoms or state['hint'] is None:continue
            received,m=clouds[0];t=stamp(m);o=min(odoms,key=lambda x:abs(stamp(x)-t))
            if abs(stamp(o)-t)>.03 and wall-received<.3:continue
            clouds.popleft()
            if state['t']-t>1.0 or abs(stamp(o)-t)>.03:continue
            q=o.pose.pose;local=np.array([q.position.x,q.position.y,yaw(q.orientation)]);previous=state['previous'];coarse=previous is None
            if previous is None:seed=state['hint']
            else:
                pp,oo=previous;h=pp[2]-oo[2];seed=[*(np.array(pp[:2])+rotation(h)@(local[:2]-oo[:2])),local[2]+h]
            raw=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=True).reshape(-1,3);start=time.monotonic()
            matches=[reference.align(raw,[*seed[:2],seed[2]+d],coarse=coarse) for d in ([-.15,0,.15] if coarse else [0.])]
            good=[x for x in matches if x['valid']];valid=bool(good) and state['contract_valid'];reason='HELD_OUT_SURFACE_MATCH' if valid else 'MAP_MATCH_REJECTED'
            if good:
                spread=max(math.dist(good[0]['pose'][:2],x['pose'][:2]) for x in good)
                yaw_spread=max(abs(math.atan2(math.sin(x['pose'][2]-good[0]['pose'][2]),math.cos(x['pose'][2]-good[0]['pose'][2]))) for x in good)
                if spread>.08 or yaw_spread>.015:valid=False;reason='AMBIGUOUS_INITIAL_MATCH'
                best=min(good,key=lambda x:x['p95_m']);rawpose=best['pose'];xy=np.array(rawpose[:2]);heading=rawpose[2]
                if not coarse and (best['translation_from_seed_m']>.15 or best['yaw_from_seed_rad']>.035):valid=False;reason='TRACKING_CORRECTION_BOUND'
                if not (np.all(xy>=lo)&np.all(xy<=hi)):valid=False;reason='OUTSIDE_REGISTERED_REGION'
                delta=xy-center;J=np.array([[1,0,-delta[1]],[0,1,delta[0]],[0,0,1.]])
                covariance=J@param@J.T+np.diag([.05**2,.05**2,.004**2]);C=np.eye(6)*.01;C[np.ix_([0,1,5],[0,1,5])]=covariance
                p=[*xy.tolist(),0.,0.,0.,math.sin(heading/2),math.cos(heading/2)]
                d=dict(t=t,received_t=state['t'],map_id=atlas['map_id'],frame_epoch=atlas['frame_epoch'],atlas_revision=atlas['revision'],valid=bool(valid),reason=reason,pose=p,covariance=C.reshape(-1).tolist(),raw_frozen_map_pose=rawpose,surface_overlap=best['overlap'],surface_p95_m=best['p95_m'],surface_points=best['heldout_points'],initialization=coarse,multistart_spread_m=spread,processing_wall_s=time.monotonic()-start,quality_source='current lidar; relative odometry guess; independent held-out surface check; separate GNSS calibration stations')
                if valid:state['previous']=(rawpose,local)
            else:d=dict(t=t,map_id=atlas['map_id'],valid=False,reason=reason,matches=matches)
            pub.publish(String(data=json.dumps(d)));f.write(json.dumps(d)+'\n');f.flush();dump(a.out/'live.json',d)
    finally:f.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
