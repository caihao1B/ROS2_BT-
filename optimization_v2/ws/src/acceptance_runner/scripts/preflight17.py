"""Read original obligations and expose qualified-region gaps before driving."""
from common17 import *
import numpy as np
from shapely.geometry import shape,box

def inspect(out):
    p=read(BASE/'plans/validated/mission_plan.json');atlas=read(BASE/'maps/s14_regions/manifest.json');result={};stations={}
    for z,zone in p['zones'].items():
        lo,hi=np.array(atlas['regions'][z]['site_extent_xy']);allpts=[]
        for i,c in enumerate(zone['plans']):
            xy=np.array([a[:2] for s in c['segments'] for a in s['poses']])-[28,14]
            allpts.append(xy);outside=~np.all((xy>=lo)&(xy<=hi),axis=1)
            result[z+str(i+1)]=dict(required_points=len(c.get('visited',[])),required_area_m2=c.get('coverage_check',{}).get('domain_area_m2'),nominal_s=c['timing']['total_s'],path_samples=len(xy),outside_registered_samples=int(outside.sum()),bounds=[xy.min(0).tolist(),xy.max(0).tolist()],eligible_in_S14=z+str(i+1) in ['A1','B1'])
        # Heldout station candidates on existing paths; deployment enters simulator
        # only. Farthest point sampling gives spatial spread, never fit from truth.
        pool=np.concatenate(allpts);chosen=[]
        for direction in [[1,0],[-1,0],[0,1],[0,-1]]:
            chosen.append(pool[np.argmax(pool@direction)])
        for _ in range(4):
            distances=np.linalg.norm(pool[:,None,:]-np.array(chosen)[None,:,:],axis=2).min(1)
            chosen.append(pool[np.argmax(distances)])
        for i,xy in enumerate(chosen):
            stations[z+str(i)]=dict(region=z,role='heldout',vehicle='diff' if z=='A' else 'ackermann',seed=17001+(0 if z=='A' else 8)+i,prior_pose=[*(xy+[28,14]).tolist(),0.],source='S12 existing route sample; no truth observation enters matcher')
    report=dict(schema='S17-PREFLIGHT/1',clusters=result,old_scope_sufficient=all(v['outside_registered_samples']==0 for v in result.values()),base_plan_revision=p['plan_revision'],atlas_revision=atlas['revision'],original_obligations_unchanged=True,scope='Geometry and old admission boundary only; no actual six-cluster completion claim')
    dump(out/'preflight.json',report);dump(out/'stations.json',dict(stations=stations));return report
if __name__=='__main__':inspect(RESULTS/'qualification')
