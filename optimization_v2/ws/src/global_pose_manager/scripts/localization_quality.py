"""Verify RTAB pose against observed elevated surfaces of the frozen map.
No simulation poses, spawn fixture, future trajectories or private labels are read.
"""
import argparse,signal,time
from collections import deque
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from rtabmap_msgs.msg import Info
from scipy.spatial import cKDTree
from s07_common import *
from mapping_analysis import ply
from map_refinement import refine
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--request',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    frozen=BASE/'maps/frozen';manifest=verify_bundle(frozen,read(a.request));quality=yaml.safe_load((BASE/'config/map_contract.yaml').read_text())['quality'];points=ply(frozen/'map_cloud.ply');points=points[(points[:,2]>.2)&np.isfinite(points).all(axis=1)];tree=cKDTree(points)
    paths=[frozen/x['path'] for x in manifest['files']]+[frozen/'manifest.json',BASE/'config/sensors.yaml',BASE/'results/S06/latest.json',a.request]
    signatures={str(p):(p.stat().st_ino,p.stat().st_size,p.stat().st_mtime_ns,p.stat().st_ctime_ns) for p in paths}
    rclpy.init();n=rclpy.create_node('s07_localization_quality');pub=n.create_publisher(String,'/s07/localization/evidence',10);contract=n.create_publisher(String,'/s07/map_contract',10);clouds=deque(maxlen=30);infos=deque(maxlen=30);poses=deque(maxlen=20);state=dict(t=0.,stop=False,valid_contract=True,reason='VERIFIED_FROZEN_BUNDLE');log=(a.out/'evidence.jsonl').open('w');last_heartbeat=0.;last_stat=0.;processed=0
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<state['t']:state['valid_contract']=False;state['reason']='CLOCK_REWIND'
        state['t']=t
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(PointCloud2,'/s07/inputs/cloud',lambda m:clouds.append(m),10);n.create_subscription(Info,'/s07/rtabmap/info',lambda m:infos.append(m),20);n.create_subscription(PoseWithCovarianceStamped,'/s07/rtabmap/localization_pose',lambda m:poses.append((time.monotonic(),m)),20)
    def end(s,f):state['stop']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('frozen map + current observations only')
    try:
        while not state['stop']:
            rclpy.spin_once(n,timeout_sec=.01);wall=time.monotonic()
            if wall-last_stat>.5:
                last_stat=wall
                for p in paths:
                    try:s=p.stat();sig=(s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)
                    except OSError:sig=None
                    if sig!=signatures[str(p)]:state['valid_contract']=False;state['reason']='BOUND_FILE_CHANGED: '+str(p);break
            if wall-last_heartbeat>.15:
                last_heartbeat=wall;m=String();m.data=json.dumps(dict(t=state['t'],valid=state['valid_contract'],reason=state['reason'],map_id=manifest['map_id']));contract.publish(m)
            if not poses or not clouds or not infos:continue
            pose_received_wall,p=poses[0];t=stamp(p)
            if state['t']-t<.15:continue
            cloud=min(clouds,key=lambda c:abs(stamp(c)-t));info=min(infos,key=lambda i:abs(stamp(i)-t))
            # Pose and Info are separate DDS callbacks. Wait for the matching
            # Info instead of consuming the pose with the previous frame's Info.
            if (abs(stamp(info)-t)>1e-5 or abs(stamp(cloud)-t)>.05) and wall-pose_received_wall<.25:continue
            poses.popleft();stats=dict(zip(info.stats_keys,info.stats_values));xyz,q=pose(p);cov=list(p.pose.covariance)
            sync=abs(stamp(cloud)-t)<=.05 and abs(stamp(info)-t)<1e-5;match=info.loop_closure_id!=0 or info.proximity_detection_id!=0
            raw=point_cloud2.read_points_numpy(cloud,field_names=['x','y','z'],skip_nans=True).reshape(-1,3);raw=raw[(raw[:,2]>.2)&(np.linalg.norm(raw,axis=1)<30)]
            backend_pose=[*xyz.tolist(),*q.tolist()];backend_covariance=cov.copy();refined,refinement=refine(raw,xyz,q,tree) if sync and match else (None,dict(reason='NO_SYNCHRONIZED_BACKEND_MATCH'))
            if refined is not None:
                xyz,q=refined['position'],refined['quaternion'];distance=refined['distance'];jac=np.zeros((6,6));jac[:3,:3]=jac[3:,3:]=refined['rotation'];matrix=jac@np.array(cov).reshape(6,6)@jac.T;matrix+=np.diag([refined['variance_floor']]*3+[refined['angular_variance_floor']]*3);cov=matrix.reshape(-1).tolist()
            else:distance=np.array([])
            inlier=distance[distance<quality['correspondence_gate_m']];ratio=len(inlier)/max(1,len(distance));p95=float(np.quantile(inlier,.95)) if len(inlier) else None;all95=float(np.quantile(distance,.95)) if len(distance) else None
            finite=np.isfinite([*xyz,*q,*cov]).all();covariance_ok=finite and all(0<=cov[k]<=quality['max_xy_variance'] for k in [0,7]) and 0<=cov[35]<=quality['max_yaw_variance']
            valid=refined is not None and state['valid_contract'] and sync and match and covariance_ok and len(raw)>=quality['min_surface_points'] and ratio>=quality['min_overlap_fraction'] and p95 is not None and p95<=quality['max_surface_p95_m']
            reason='OBSERVED_MAP_MATCH' if valid else ('SYNC_REJECTED' if not sync else 'NO_REGISTERED_MATCH' if not match else 'COVARIANCE_REJECTED' if not covariance_ok else refinement['reason'] if refined is None else 'ELEVATED_SURFACE_MISMATCH')
            row=dict(t=t,received_t=state['t'],map_id=manifest['map_id'],valid=bool(valid),reason=reason,pose=[*xyz.tolist(),*q.tolist()],covariance=cov,loop=int(info.loop_closure_id),proximity=int(info.proximity_detection_id),surface_points=len(distance),backend_pose=backend_pose,backend_covariance=backend_covariance,refinement=refinement,surface_overlap=ratio,surface_p95_m=p95,all_surface_p95_m=all95,sync=sync,registration_linear_variance=stats.get('Loop/Linear_variance/'),registration_angular_variance=stats.get('Loop/Angular_variance/'),backend_visual_inliers=stats.get('Loop/Visual_inliers/'),quality_source='RTAB candidate plus bounded ICP on alternating elevated points; held-out geometry validation; same sensors, not independent fusion')
            m=String();m.data=json.dumps(row);pub.publish(m);log.write(m.data+'\n');log.flush();processed+=1;dump(a.out/'live.json',row)
    finally:log.close();dump(a.out/'result.json',dict(processed=processed,contract_valid=state['valid_contract'],reason=state['reason']));n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
