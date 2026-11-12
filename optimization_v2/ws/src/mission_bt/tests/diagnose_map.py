"""Offline initial-pose repair experiment from measured scans, never truth.
These candidates are diagnostics only and cannot authorize motion.
"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
sys.path.insert(0,str(BASE/'ws/src/map_manager/scripts'))
from mapping_analysis import ply
from scipy.spatial import cKDTree

def run(directory):
    p=Path(directory);reference=ply(BASE/'maps/frozen/map_cloud.ply');reference=reference[(reference[:,2]>.2)&np.isfinite(reference).all(axis=1)];tree=cKDTree(reference)
    raw=np.load(sorted((p/'sensor_diagnostic').glob('cloud_*.npz'))[0])['points'];raw=raw[(raw[:,2]>.2)&(np.linalg.norm(raw,axis=1)<30.)]
    gps=[x['antenna'] for x in rows(p/'sources/events.jsonl') if x.get('kind')=='gnss' and x.get('valid')];center=np.median(gps,axis=0);trials=[]
    for heading in [-.15,0.,.15]:
        co,si=math.cos(heading),math.sin(heading);r=np.array([[co,-si],[si,co]]);pos=center.copy();pos[2]=0.;pos[:2]-=r@[-.1,0]
        fit=raw[::2].copy();fit[:,:2]=fit[:,:2]@r.T+pos[:2];fit[:,2]+=pos[2];total_yaw=heading
        for iteration in range(35):
            dist,index=tree.query(fit);mask=dist<1.5
            if mask.sum()<100:break
            a=fit[mask,:2];b=reference[index[mask],:2];ca=a.mean(axis=0);cb=b.mean(axis=0);u,sv,vh=np.linalg.svd((a-ca).T@(b-cb));update=vh.T@u.T
            if np.linalg.det(update)<0:vh[-1]*=-1;update=vh.T@u.T
            shift=cb-update@ca;fit[:,:2]=fit[:,:2]@update.T+shift;pos[:2]=update@pos[:2]+shift;r=update@r;total_yaw+=math.atan2(update[1,0],update[0,0])
        held=raw[1::2].copy();held[:,:2]=held[:,:2]@r.T+pos[:2];d=tree.query(held)[0];inside=d[d<.35]
        trials.append(dict(seed_yaw=heading,estimated_pose=[*pos[:2],total_yaw],surface_count=len(held),overlap=float(len(inside)/max(1,len(d))),p95_m=float(np.quantile(inside,.95)) if len(inside) else None,gnss_innovation_m=float(np.linalg.norm(pos[:2]-(center[:2]-r@[-.1,0]))),no_online_admission=True))
    result=dict(experiment='bounded wider-basin planar ICP, alternating fit/heldout points',raw_scan=str(p/'sensor_diagnostic'),reference_sha=sha(BASE/'maps/frozen/map_cloud.ply'),trials=trials)
    dump(p/'map_repair_diagnostic.json',result);print(json.dumps(result,ensure_ascii=False))
if __name__=='__main__':run(sys.argv[1])
