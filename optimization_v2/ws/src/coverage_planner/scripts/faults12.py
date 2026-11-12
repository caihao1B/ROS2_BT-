"""Runnable refusal cases, isolated from the authoritative input files."""
import copy
from common12 import *
from contracts12 import original_infeasible,check_binding
from motion12 import segment
from spray12 import visibility

def run_faults(mission,c,out):
    out.mkdir(parents=True,exist_ok=True);d={'original_budget':original_infeasible(c),'stale':[]}
    for name,needle in [('map','maps/prior/workspaces.geojson'),('robot','config/vehicles/ackermann.yaml'),('tool','config/tool_contract.yaml'),('planning','config/planning.yaml')]:
        versions=dict(mission['versions']);key=next(k for k in versions if k.endswith(needle));versions[key]='0'*64
        try:check_binding(mission,versions);error=None
        except ValueError as exc:error=str(exc)
        d['stale'].append(dict(kind=name,changed_key=key,error=error,passed=error=='STALE_INPUT_VERSION',scope='in-memory tampered version; authoritative files unchanged'))
    bad=copy.deepcopy(mission);bad['normal_budget_s']+=1
    try:check_binding(bad);error=None
    except ValueError as exc:error=str(exc)
    d['stale'].append(dict(kind='plan_payload',error=error,passed=error=='PLAN_HASH_MISMATCH'))
    narrow=box(-2,-2,2,5);r=backend(dict(mode='turns',radius=2.11,ds=.04,queries=[dict(id=0,start=[0,0,0],goal=[0,3,math.pi])]),out,'narrow')
    d['narrow']=dict(motion=mapping(narrow),candidates=[])
    for raw in r['turns']:
        s=segment(raw['poses'],'TURN','ackermann',narrow,c,stop_curvature=not raw['continuous_curvature']);d['narrow']['candidates'].append(dict(poses=s['poses'],validation=s['validation'],continuous_curvature=raw['continuous_curvature']))
    d['narrow']['passed']=all(not s['validation']['passed'] for s in d['narrow']['candidates'])
    try:visibility([[137,69],[128,58]],shape(mission['zones']['B']['motion_domain']),c);error=None
    except ValueError as exc:error=str(exc)
    d['unreachable']=dict(point=[137,69],error=error,passed=bool(error and error.startswith('UNREACHABLE')))
    d['passed']=all(s['passed'] for s in d['stale']) and d['narrow']['passed'] and d['unreachable']['passed'] and d['original_budget']['status']=='INFEASIBLE'
    dump(out/'faults.json',d);return d
