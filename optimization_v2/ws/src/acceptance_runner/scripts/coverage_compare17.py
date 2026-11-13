"""Same six point-service obligations: offline order versus online nearest task.

The prior and target set are known in both arms. Online means the service order
is selected at dispatch from the current estimated pose; it is not exploration
of an unknown map, nor online mowing. Both arms use live Nav2 repair.
"""
from common17 import *
from overlay17 import build,replace
import argparse,math
from collections import deque

def select_jobs(m,jobs):
    points=[j for j in jobs if j['kind']=='POINT_SERVICE']
    start=[m.p['start'][0]-28,m.p['start'][1]-14]
    chosen={j['id'] for j in sorted(points,key=lambda j:(math.dist(start,j['poses'][-1][:2]),j['id']))[:6]}
    selected=[j for j in points if j['id'] in chosen]
    if len(selected)!=6:raise ValueError('SIX_ORIGINAL_TARGETS_REQUIRED')
    # The original task IDs/targets/tool quantity and return destination remain.
    result=selected+[j for j in jobs if j['kind']=='TRANSIT']
    dump(m.a.out/'coverage_contract17.json',dict(original_plan=m.c['plan_revision'],scope='SIX_POINT_SERVICE_ABLATION_NOT_FULL_CLUSTER',targets=sorted(selected,key=lambda j:j['id']),return_jobs=[j for j in result if j['kind']=='TRANSIT'],selection='six closest original service poses to the common deployment, Euclidean distance then task ID',required_points=6,required_quantity_each=2.))
    return result

def choose(m):
    if os.environ['S17_COVERAGE_METHOD']!='pure_online' or m.busy or m.blocked or m.admission_recovery.since is not None or not m.initialized or not m.pending:return
    candidates=[j for j in m.pending if j['kind']=='POINT_SERVICE']
    if not candidates:return
    xy=m.api['xy']();j=min(candidates,key=lambda j:(math.dist(xy,j['poses'][-1][:2]),j['id']))
    if j['id']==m.pending[0]['id']:return
    old=m.pending[0]['id'];m.pending=deque([j]+[x for x in m.pending if x['id']!=j['id']])
    # A directive for the former front item cannot dispatch the new one: the
    # existing BT obligation/token checks must produce a fresh decision first.
    m.log('online_target_selected',id=j['id'],previous=old,estimated_xy=list(map(float,xy)),remaining=len(candidates),policy='NEAREST_UNSERVED_POINT_THEN_NAV2')

def run(a):
    if not a.out.resolve().is_relative_to(RESULTS) or not (a.seed in read(EVAL/'seeds.json')['paired'] or (a.seed==17310 and a.out.name.startswith('study_probe_'))):raise ValueError('EXPERIMENT_SCOPE')
    os.environ.update(S17_CLUSTER='A1',S17_CONTROLLER='DWB',S17_OPERATION='cluster',S17_BENCHMARK='',S17_COVERAGE_METHOD=a.method,S17_PREDICTION='disabled')
    os.environ.pop('S17_RESUME_MANIFEST',None);os.environ.pop('S17_SURVEY_STATIC',None)
    from inputs17 import snapshot
    snapshot(a.out);scripts=build(a.out/'overlay',RESULTS/'qualification/stations.json',a.rtf,True)
    p=scripts/'mission14.py';s=p.read_text()
    s=replace(s,"self.scope='INSPECTION' if inspecting() else ('FULL_CLUSTER' if a.case.startswith('nominal_') else 'SMOKE')","self.scope='PAIRED_POINT_SERVICE'")
    s=replace(s,'        return jobs\n\n    def transit_job', '        from coverage_compare17 import select_jobs\n        return select_jobs(self,jobs)\n\n    def transit_job')
    s=replace(s,"        now=time.monotonic();t=self.st['t'];", "        from coverage_compare17 import choose\n        choose(self)\n        now=time.monotonic();t=self.st['t'];")
    p.write_text(s);sys.path.insert(0,str(scripts));from runtime14 import run_case
    run_case(a.out,'zone_a','nominal_spray_cluster',900,a.wall,seed=a.seed,gui=False,kind='diff',controller='DWB')
    from evaluate17 import evaluate
    spec=dict(id=a.out.name,seed=a.seed,cluster='A1',vehicle='diff',controller='DWB',scope='PAIRED_POINT_SERVICE')
    m=evaluate(a.out,spec,BASE/'build_s17/monitor');contract=read(a.out/'client/coverage_contract17.json');mission=read(a.out/'client/mission.json')
    required={j['id'] for j in contract['targets']+contract['return_jobs']}
    complete=mission['state']=='SUCCEEDED' and mission['required_points']==6 and mission['confirmed_points']==6 and set(mission['done'])==required and not mission['uncertainty_pending']
    m.update(status='PASS' if m['safety'] and m['localization_pass'] and complete else ('PARTIAL' if m['safety'] and m['localization_pass'] else 'FAIL'),task_completed=complete,complete=complete,required_points=6,original_denominator=False,benchmark_denominator_valid=mission['required_points']==6,task_hash=sha(a.out/'client/coverage_contract17.json'),scenario='six_point_coverage',method=a.method,formal_credit=False,limits_hash=canonical(dict(vehicle=sha(BASE/'config/vehicles/diff.yaml'),gate=sha(BASE/'config/controller_profiles.yaml'),seconds=900,rtf=a.rtf)),scope='six original point-service obligations with live Nav2 in both groups; not a whole-cluster or mowing result')
    dump(a.out/'metrics17.json',m);return m

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seed',type=int,required=True);ap.add_argument('--method',choices=['offline_repair','pure_online'],required=True);ap.add_argument('--rtf',type=float,default=1.);ap.add_argument('--wall',type=float,default=1800);a=ap.parse_args()
    if not .4<=a.rtf<=1 or not 300<=a.wall<=3000:ap.error('BOUNDED_STUDY_REQUIRED')
    raise SystemExit(0 if run(a)['safety'] else 1)
