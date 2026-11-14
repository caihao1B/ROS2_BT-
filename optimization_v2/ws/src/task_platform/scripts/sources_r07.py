"""Calibrated observations for the unchanged native S08 localization supervisor."""
from common import ROOT,BASE,read,sha
import argparse,json,signal,os,importlib.util,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from contract_r05 import verify_runtime_contract
spec=importlib.util.spec_from_file_location('r07_geodesy',BASE/'ws/src/sensor_sim_adapter/scripts/common.py');geo=importlib.util.module_from_spec(spec);spec.loader.exec_module(geo)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--config',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);cfg=read(a.config);verify_runtime_contract(read(a.out.parent/'request.json'));binding=read(Path(os.environ['R05_BINDING']));paths=[a.config,Path(os.environ['R05_BINDING']),ROOT/binding['rig_path'],ROOT/binding['atlas_path'],ROOT/binding['reference_cloud_path']];signatures={str(p):(p.stat().st_ino,p.stat().st_size,p.stat().st_mtime_ns,p.stat().st_ctime_ns) for p in paths};rclpy.init();n=rclpy.create_node('r07_global_sources',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);pubs={k:n.create_publisher(String,'/s08/'+k,20) for k in ['gnss','map','contract']};st=dict(t=0.,stop=False,valid=True);fixes={};quality={};log=(a.out/'sources.jsonl').open('w')
    n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    def publish(key,d):pubs[key].publish(String(data=json.dumps(d,allow_nan=False)));log.write(json.dumps(dict(kind=key,**d))+'\n');log.flush()
    def match():
        for key in list(fixes):
            if key not in quality:continue
            m=fixes.pop(key);q=quality.pop(key);d=dict(t=key,mode=q['reported_mode'],valid=False,antenna_base_m=cfg['rig']['gnss']['xyz'])
            try:
                cov=np.array(m.position_covariance).reshape(3,3);llh=[m.latitude,m.longitude,m.altitude]
                if m.header.frame_id!='gnss_link' or m.status.status<0 or m.position_covariance_type==0 or not np.isfinite(cov).all() or not np.allclose(cov,cov.T) or np.linalg.eigvalsh(cov).min()<=0:raise ValueError('GNSS_FRAME_OR_COVARIANCE')
                if not np.isfinite(llh).all() or not -90<llh[0]<90 or not -180<=llh[1]<=180:raise ValueError('GNSS_COORDINATES')
                xyz=geo.to_enu(llh,cfg['datum_llh'])-np.array(cfg['site_origin']);R=geo.enu_rotation(cfg['datum_llh'])@geo.enu_rotation(llh).T;C=R@cov@R.T+np.eye(3)*cfg['datum_translation_sigma_m']**2;d.update(valid=st['valid'],antenna=xyz.tolist(),covariance=C.reshape(-1).tolist(),reason='CALIBRATED_ENU')
            except (ValueError,TypeError) as ex:d['reason']=str(ex)
            publish('gnss',d)
        for cache in [fixes,quality]:
            while len(cache)>20:cache.pop(next(iter(cache)))
    def fix(m):fixes[round(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,6)]=m;match()
    def quality_cb(m):d=json.loads(m.data);quality[round(d['stamp'],6)]=d;match()
    n.create_subscription(NavSatFix,'/s04/gnss/fix',fix,qos_profile_sensor_data);n.create_subscription(String,'/s04/gnss/quality',quality_cb,qos_profile_sensor_data)
    def map_cb(m):
        d=json.loads(m.data);d['valid']=bool(d.get('valid') and st['valid'] and d.get('rig_revision')==cfg['rig']['revision'] and d.get('map_revision')==cfg['map_revision']);publish('map',d)
    n.create_subscription(String,'/s07/localization/evidence',map_cb,20)
    def contract():
        for path,expected in signatures.items():
            try:s=Path(path).stat();actual=(s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)
            except OSError:actual=None
            if actual!=expected:st['valid']=False
        publish('contract',dict(t=st['t'],valid=st['valid'],map_id=cfg['map_id'],reason='R05_RIG_MAP_BINDING' if st['valid'] else 'FILE_CHANGED_RESTART_REQUIRED'))
    n.create_timer(.1,contract)
    def stop(s,f):st['stop']=True
    for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
    (a.out/'ready').write_text('actual GNSS + quality pairing; sensor-derived map; immutable bindings')
    try:
        while not st['stop']:rclpy.spin_once(n,timeout_sec=.01)
    finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
