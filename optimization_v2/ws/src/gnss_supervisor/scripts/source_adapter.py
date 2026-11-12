"""Convert noisy GNSS; explicit observation fault fixture, never /truth or raw121.
No TF, no global observation fed back into S05 or RTAB.
"""
import argparse,signal,copy
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from common08 import *
from alignment import convert,geo

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--case',required=True);a=p.parse_args();alignment=validate();faults=yaml.safe_load((BASE/'config/fault_cases/localization.yaml').read_text())[a.case];a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('s08_source_adapter');f=(a.out/'events.jsonl').open('w');fingerprint=digest(BASE/'config/gnss_alignment.yaml');state=dict(stop=False,quality={},fixes={},alignment_valid=True);gps=n.create_publisher(String,'/s08/gnss',20);mp=n.create_publisher(String,'/s08/map',20)
    def record(d):f.write(json.dumps(d,separators=(',',':'))+'\n');f.flush()
    def pub(p,d):m=String();m.data=json.dumps(d,allow_nan=False);p.publish(m);record(d)
    def fault(t):return next((x for x in faults if x['start']<=t<x['end']),{})
    def match():
        for key in list(state['fixes']):
            if key not in state['quality']:continue
            m=state['fixes'].pop(key);q=state['quality'].pop(key);t=stamp(m);ph=fault(t);mode=ph.get('gnss',q['reported_mode']);d=dict(t=t,kind='gnss',mode=mode,valid=False,reason='UNAVAILABLE',frame='map',antenna_base_m=alignment['antenna_base_m'])
            if mode=='dropout':d['mode']='no_fix';pub(gps,d);continue
            llh=np.array([m.latitude,m.longitude,m.altitude]);cov=np.array(m.position_covariance).reshape(3,3)
            if mode=='false_fixed':llh=geo.from_enu(geo.to_enu(llh,alignment['datum_llh'])+ph['bias_enu'],alignment['datum_llh']);d['mode']='fixed'
            if mode=='float':cov=np.diag([.09,.09,.36])
            try:
                if m.header.frame_id!='gnss_link' or m.position_covariance_type==0 or m.status.status<0:raise ValueError('frame/covariance/status invalid')
                xyz,C=convert(llh,cov,alignment);d.update(valid=True,reason='CONVERTED',antenna=xyz.tolist(),covariance=C.reshape(-1).tolist(),llh=llh.tolist())
            except Exception as e:d['reason']=str(e)
            pub(gps,d)
        for k in ['fixes','quality']:
            while len(state[k])>20:state[k].pop(next(iter(state[k])))
    contract_pub=n.create_publisher(String,'/s08/contract',20)
    def contract(m):
        d=json.loads(m.data)
        state['alignment_valid']=state['alignment_valid'] and digest(BASE/'config/gnss_alignment.yaml')==fingerprint
        d['valid']=d['valid'] and state['alignment_valid']
        if not state['alignment_valid']:d['reason']='GNSS_ALIGNMENT_CHANGED_RESTART_REQUIRED'
        message=String();message.data=json.dumps(d);contract_pub.publish(message)
    n.create_subscription(String,'/s07/map_contract',contract,20)
    def gnss(m):state['fixes'][round(stamp(m),6)]=m;match()
    def quality(m):q=json.loads(m.data);state['quality'][round(q['stamp'],6)]=q;match()
    def map_obs(m):
        d=json.loads(m.data);t=d['t'];ph=fault(t);d['kind']='map';d['original_pose']=d.get('pose');d['fixture_fault']=ph.get('map','none')
        if ph.get('map')=='dropout':d.update(valid=False,reason='TEST_MAP_OBSERVATION_UNAVAILABLE')
        if ph.get('map')=='mismatch' and d.get('pose'):d['pose']=list(d['pose']);d['pose'][0]+=1.;d['pose'][1]-=.5
        pub(mp,d)
    n.create_subscription(NavSatFix,'/s04/gnss/fix',gnss,qos_profile_sensor_data);n.create_subscription(String,'/s04/gnss/quality',quality,qos_profile_sensor_data);n.create_subscription(String,'/s07/localization/evidence',map_obs,20)
    def end(s,f):state['stop']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('ENU observations only; declared test faults are injected at observation boundary')
    try:
        while not state['stop']:rclpy.spin_once(n,timeout_sec=.02)
    finally:f.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
