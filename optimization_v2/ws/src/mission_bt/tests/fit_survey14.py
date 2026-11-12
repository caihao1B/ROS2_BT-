"""Analyze captured independent observations; never reads deployment.json/SDF."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
from regional_math14 import SurfaceMap,rotation,rigid_fit
sys.path.insert(0,str(BASE/'ws/src/map_manager/scripts'))
from mapping_analysis import ply
sys.path.insert(0,str(BASE/'ws/src/gnss_supervisor/scripts'))
from alignment import geo

def observation(path,reference):
    raw=path/'sensor_diagnostic';quality={round(q['stamp'],4):q['reported_mode'] for q in rows(raw/'quality.jsonl')}
    samples=[q for q in rows(raw/'gnss.jsonl') if q['t']>=9 and q['t']<=21 and quality.get(round(q['t'],4))=='fixed' and q['status']>=0]
    if len(samples)<25:raise ValueError('INSUFFICIENT_INDEPENDENT_GNSS '+str(path))
    alignment=yaml.safe_load((BASE/'config/gnss_alignment.yaml').read_text());enu=np.array([geo.to_enu(q['llh'],alignment['datum_llh']) for q in samples]);antenna=np.median(enu,axis=0)-[28,14,0]
    # Only the recorded, deliberately coarse operator yaw is used as an initial guess.
    hints=read(path/'runtime.json')['events'];hint=next(q['pose'] for q in hints if q['event']=='initial_hint');heading=2*math.atan2(hint[5],hint[6]);matches=[];rejected=[]
    for file in sorted(raw.glob('cloud_*.npz')):
        cloud=np.load(file)['points'];alternatives=[]
        for dyaw in [-.15,0,.15]:
            h=heading+dyaw;guess=antenna[:2]-rotation(h)@np.array(alignment['antenna_base_m'][:2]);m=reference.align(cloud,[*guess,h]);alternatives.append(m)
        good=[m for m in alternatives if m['valid']]
        if not good:
            rejected.append(dict(file=str(file.relative_to(ROOT)),sha256=sha(file),reason='NO_SURFACE_MATCH',alternatives=alternatives));continue
        poses=np.array([m['pose'] for m in good]);spread=float(np.linalg.norm(poses[:,:2]-poses[0,:2],axis=1).max());ys=float(np.abs(np.arctan2(np.sin(poses[:,2]-poses[0,2]),np.cos(poses[:,2]-poses[0,2]))).max())
        if spread>.08 or ys>.015:
            rejected.append(dict(file=str(file.relative_to(ROOT)),sha256=sha(file),reason='AMBIGUOUS_MULTI_START',alternatives=alternatives));continue
        best=min(good,key=lambda m:m['p95_m']);matches.append(dict(file=str(file.relative_to(ROOT)),sha256=sha(file),multistart_spread_m=spread,**best))
    if len(matches)<3:
        dump(path/'registration_rejection.json',dict(reason='FEWER_THAN_THREE_QUALITY_FRAMES',accepted=matches,rejected=rejected))
        raise ValueError('INSUFFICIENT_TEMPORAL_CONSENSUS '+str(path))
    poses=np.array([m['pose'] for m in matches]);p=np.median(poses,axis=0);h=np.median(np.unwrap(poses[:,2]));p[2]=math.atan2(math.sin(h),math.cos(h));mapped_antenna=p[:2]+rotation(p[2])@np.array(alignment['antenna_base_m'][:2]);scatter=np.linalg.norm(enu[:,:2]-np.median(enu[:,:2],axis=0),axis=1)
    if np.linalg.norm(poses[:,:2]-p[:2],axis=1).max()>.05:raise ValueError('UNSTABLE_STATION_ALIGNMENT '+str(path))
    result=dict(source=str(path.relative_to(ROOT)),source_hashes={f.name:sha(f) for f in [raw/'gnss.jsonl',raw/'quality.jsonl',path/'runtime.json']},map_pose=p.tolist(),raw_map_antenna_xy=mapped_antenna.tolist(),site_antenna_xy=antenna[:2].tolist(),gnss_samples=len(samples),gnss_scatter_p95_m=float(np.quantile(scatter,.95)),map_scatter_m=float(np.linalg.norm(poses[:,:2]-p[:2],axis=1).max()),matches=matches,provenance='noisy sensor observations; operator heading only initializes ICP')
    result['rejected_frames']=rejected;result['quality_policy']='At least three independently quality-gated frames, <=5 cm temporal scatter; no gate relaxation'
    dump(path/'registration_observation.json',result);return result

def main():
    p=BASE/'results/S14/survey_20261112';reference=SurfaceMap(ply(BASE/'maps/frozen/map_cloud.ply'));reference.prepare_normals()
    for name in sys.argv[1:]:
        result=observation(p/name,reference);print(name,json.dumps({k:v for k,v in result.items() if k not in ['matches','source_hashes']},ensure_ascii=False),flush=True)
if __name__=='__main__':main()
