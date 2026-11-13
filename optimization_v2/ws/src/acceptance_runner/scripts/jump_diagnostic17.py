"""Offline geometry diagnostic on recorded observations; not controller replay."""
from common17 import *
import argparse,numpy as np
from scipy.spatial.transform import Rotation

def matrix(p):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(p[3:]).as_matrix();t[:3,3]=p[:3];return t

def diagnose(folder):
    folder=Path(folder).resolve();all_obs=rows(folder/'online/observations.jsonl');odom=[r for r in all_obs if r['record_type']=='odom'];states=[r for r in all_obs if r['record_type']=='status' and r['has_transform']];times=np.array([r['t'] for r in odom]);status_times=np.array([r['t'] for r in states]);cases=[];tested=0
    for r in rows(folder/'quality/evidence.jsonl'):
        if not r.get('valid') or not len(times) or not len(status_times):continue
        k=int(np.searchsorted(times,r['t']));candidates=[i for i in [k-1,k] if 0<=i<len(times)];k=min(candidates,key=lambda i:abs(times[i]-r['t']))
        j=int(np.searchsorted(status_times,r.get('received_t',r['t']),side='right'))-1
        if j<0 or abs(times[k]-r['t'])>.05:continue
        local=matrix(odom[k]['pose']);current=matrix(states[j]['correction']);candidate=matrix(r['pose'])@np.linalg.inv(local);tested+=1
        origin=float(np.linalg.norm(candidate[:3,3]-current[:3,3]));base=float(np.linalg.norm((candidate@local)[:3,3]-(current@local)[:3,3]));angle=float(Rotation.from_matrix(current[:3,:3].T@candidate[:3,:3]).magnitude())
        if origin>.15 and base<=.15 and angle<=.035:cases.append(dict(t=r['t'],odom_distance_m=float(np.linalg.norm(local[:2,3])),origin_translation_m=origin,robot_translation_m=base,rotation_rad=angle))
    result=dict(scope='pointwise geometry on recorded map/odom/status observations; not an exact timestamp replay of all ROS callbacks',observations_tested=tested,origin_sensitive_candidates=len(cases),examples=cases[:12],source=str(folder.relative_to(ROOT)),input_hashes={str((folder/n).relative_to(ROOT)):sha(folder/n) for n in ['online/observations.jsonl','quality/evidence.jsonl']})
    dump(RESULTS/'component_contracts'/('jump_diagnostic_'+folder.name+'.json'),result);return result
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('case',type=Path);args=ap.parse_args();print(json.dumps(diagnose(args.case),ensure_ascii=False,indent=2))
