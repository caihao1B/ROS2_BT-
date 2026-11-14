"""R07 map matching: coarse prealignment every frame; unchanged holdout, tracking and GNSS gates."""
import argparse,os,sys,json,math,time,signal,importlib.util
from pathlib import Path
from collections import deque
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,NavSatFix
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from contract_r05 import verify_runtime_contract,read,sha,ROOT,BASE
spec=importlib.util.spec_from_file_location('regional_math_r05',Path(os.environ['R05_ADAPTER'])/'regional_math.py');math14=importlib.util.module_from_spec(spec);spec.loader.exec_module(math14)
spec=importlib.util.spec_from_file_location('geodesy_r05',BASE/'ws/src/sensor_sim_adapter/scripts/common.py');geo=importlib.util.module_from_spec(spec);spec.loader.exec_module(geo)
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def dump(p,d):q=p.with_suffix('.tmp');q.write_text(json.dumps(d));q.replace(p)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--request',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);manifest=verify_runtime_contract(read(a.request));binding=read(Path(os.environ['R05_BINDING']));rig=read(ROOT/binding['rig_path']);atlas=read(ROOT/binding['atlas_path']);cloudpath=ROOT/binding['reference_cloud_path'];points=np.loadtxt(cloudpath,skiprows=7);reference=math14.SurfaceMap(points);reference.prepare_normals();signatures={str(p):(p.stat().st_ino,p.stat().st_size,p.stat().st_mtime_ns,p.stat().st_ctime_ns) for p in [Path(os.environ['R05_BINDING']),ROOT/binding['rig_path'],ROOT/binding['vehicle_path'],ROOT/binding['atlas_path'],cloudpath]}
 rclpy.init();n=rclpy.create_node('r05_regional_localizer',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);pub=n.create_publisher(String,'/s07/localization/evidence',10);clouds=deque(maxlen=4);odoms=deque(maxlen=100);fixes=deque(maxlen=40);state=dict(t=0.,stop=False,hint=None,previous=None,contract_valid=True);log=(a.out/'evidence.jsonl').open('w');laststat=0.;corrections=deque(maxlen=7)
 n.create_subscription(Clock,'/clock',lambda m:state.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data);n.create_subscription(PointCloud2,'/s07/inputs/cloud',lambda m:clouds.append((time.monotonic(),m)),5);n.create_subscription(Odometry,'/s07/inputs/odom',lambda m:odoms.append(m),30);n.create_subscription(NavSatFix,'/s04/localization/gnss',lambda m:fixes.append(m),qos_profile_sensor_data)
 def hint(m):
  if m.header.frame_id=='map':state['hint']=[m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)];state['previous']=None;corrections.clear()
 n.create_subscription(PoseWithCovarianceStamped,'/s07/rtabmap/initialpose',hint,10)
 def stop(s,f):state['stop']=True
 for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
 (a.out/'ready').write_text('current observed map match and independent GNSS gate; no truth');processed=0
 try:
  while not state['stop']:
   rclpy.spin_once(n,timeout_sec=.005);wall=time.monotonic()
   if wall-laststat>.5:
    laststat=wall
    for name,old in signatures.items():
     try:s=Path(name).stat();sig=(s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)
     except OSError:sig=None
     if sig!=old:state['contract_valid']=False
   if not clouds or not odoms or state['hint'] is None:continue
   received,m=clouds[0];t=stamp(m);o=min(odoms,key=lambda x:abs(stamp(x)-t))
   if abs(stamp(o)-t)>.03 and wall-received<.3:continue
   clouds.popleft()
   if state['t']-t>1.0 or abs(stamp(o)-t)>.03:continue
   q=o.pose.pose;local=np.array([q.position.x,q.position.y,yaw(q.orientation)]);previous=state['previous'];coarse=previous is None
   if coarse:seed=state['hint']
   else:
    pp,oo=previous;heading=pp[2]-oo[2];seed=[*(np.array(pp[:2])+math14.rotation(heading)@(local[:2]-oo[:2])),local[2]+heading]
   raw=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=True).reshape(-1,3);start=time.monotonic();matches=[reference.align(raw,[*seed[:2],seed[2]+d],coarse=True) for d in ([-.15,0,.15] if coarse else [0.])];good=[x for x in matches if x['valid']];valid=bool(good) and state['contract_valid'];reason='HELD_OUT_SURFACE_MATCH' if valid else 'MAP_MATCH_REJECTED'
   if good:
    spread=max(math.dist(good[0]['pose'][:2],x['pose'][:2]) for x in good);angle=max(abs(math.atan2(math.sin(x['pose'][2]-good[0]['pose'][2]),math.cos(x['pose'][2]-good[0]['pose'][2]))) for x in good)
    if spread>.08 or angle>.015:valid=False;reason='AMBIGUOUS_INITIAL_MATCH'
    best=min(good,key=lambda x:x['p95_m']);p=best['pose'];h=p[2]
    if not coarse and (best['translation_from_seed_m']>.15 or best['yaw_from_seed_rad']>.035):valid=False;reason='TRACKING_CORRECTION_BOUND'
    residual=None;nis=None;fix=min(fixes,key=lambda f:abs(stamp(f)-t)) if fixes else None;cov=np.eye(6)*.01;cov[0,0]=cov[1,1]=.05**2+atlas['datum_translation_sigma_m']**2;cov[5,5]=.004**2
    if fix is not None and abs(stamp(fix)-t)<.6 and fix.status.status==2:
     enu=geo.to_enu([fix.latitude,fix.longitude,fix.altitude],atlas['datum_llh'])-np.array(atlas['site_origin']);antenna=np.array(p[:2])+math14.rotation(h)@np.array(rig['gnss']['xyz'][:2]);delta=enu[:2]-antenna;C=np.array(fix.position_covariance).reshape(3,3)[:2,:2]+cov[:2,:2];residual=float(np.linalg.norm(delta));nis=float(delta@np.linalg.solve(C,delta));gnss_ok=residual<=.15 and nis<=9.21 and np.linalg.eigvalsh(C).min()>0
    else:gnss_ok=False
    if not gnss_ok:valid=False;reason='GNSS_MAP_INNOVATION_OR_FRESHNESS'
    evidence=dict(t=t,received_t=state['t'],valid=bool(valid),reason=reason,pose=[*p[:2],0.,0.,0.,math.sin(h/2),math.cos(h/2)],covariance=cov.reshape(-1).tolist(),surface_points=best['heldout_points'],surface_overlap=best['overlap'],surface_p95_m=best['p95_m'],multistart_spread_m=spread,gnss_innovation_m=residual,gnss_nis=nis,initialization=coarse,processing_wall_s=time.monotonic()-start)
    if valid:
     state['previous']=(p,local)
     cyaw=math.atan2(math.sin(h-local[2]),math.cos(h-local[2]));shift=np.array(p[:2])-math14.rotation(cyaw)@local[:2];corrections.append((t,shift,cyaw))
     while corrections and t-corrections[0][0]>1.5:corrections.popleft()
     shifts=np.array([v[1] for v in corrections]);angles=np.unwrap([v[2] for v in corrections]);mh=float(np.median(angles));ms=np.median(shifts,axis=0);smooth=ms+math14.rotation(mh)@local[:2];syaw=local[2]+mh
     evidence['raw_pose']=evidence['pose'];evidence['pose']=[*smooth,0.,0.,0.,math.sin(syaw/2),math.cos(syaw/2)];scatter=np.median(np.abs(shifts-ms),axis=0)*1.4826;cov[0,0]+=scatter[0]**2;cov[1,1]+=scatter[1]**2;evidence['covariance']=cov.reshape(-1).tolist();evidence['temporal_filter']=dict(method='median map-to-odom correction',samples=len(corrections),span_s=t-corrections[0][0],robust_scatter_m=scatter.tolist(),original_capture_stamp=t)

   else:evidence=dict(t=t,valid=False,reason=reason,matches=matches,processing_wall_s=time.monotonic()-start)
   evidence.update(map_id=atlas['map_id'],map_revision=atlas['revision'],rig_revision=rig['revision'],quality_source='S14/S17 sensor-derived frozen cloud; RTAB local ICP + wheel/IMU; held-out map matching; independent GNSS supervisor',backend='regional_surface_map',truth_used=False)
   pub.publish(String(data=json.dumps(evidence)));log.write(json.dumps(evidence)+'\n');log.flush();dump(a.out/'live.json',evidence);processed+=1
 finally:log.close();dump(a.out/'result.json',dict(processed=processed,contract_valid=state['contract_valid']));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
