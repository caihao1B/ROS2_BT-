"""Fresh process per case: no cached module can carry a preceding cluster."""
from common17 import *
from overlay17 import build
import argparse,math

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--spec',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--wall',type=float,required=True);ap.add_argument('--seconds',type=float,default=1800);ap.add_argument('--rtf',type=float,default=.4);ap.add_argument('--gui',action='store_true');ap.add_argument('--resume-from',type=Path);a=ap.parse_args();s=read(a.spec)
    if s['cluster'] not in ['A1','A2','A3','B1','B2','B3']:raise ValueError('INVALID_CLUSTER')
    if (s['vehicle']=='diff')!=(s['cluster'][0]=='A') or s['vehicle'] not in ['diff','ackermann']:raise ValueError('CLUSTER_CHASSIS_MISMATCH')
    if not a.out.resolve().is_relative_to(RESULTS) or not (1<=a.seconds<=1800 and 1<=a.wall<=5000 and math.isfinite(a.rtf) and .2<=a.rtf<=1.):raise ValueError('BOUNDED_CASE_CONTRACT')
    os.environ['S17_CLUSTER']=s['cluster']
    os.environ['S17_OPERATION']=s.get('operation','cluster')
    os.environ['S17_CONTROLLER']=s['controller']
    os.environ['S17_BENCHMARK']=s.get('benchmark','')
    os.environ['S17_PREDICTION']=s.get('prediction','disabled')
    if os.environ['S17_PREDICTION'] not in ['disabled','imm'] or (os.environ['S17_PREDICTION']!='disabled' and s.get('scope')!='DEVELOPMENT_NOT_FORMAL'):raise ValueError('PREDICTOR_CANDIDATE_NOT_PRODUCTION_QUALIFIED')
    if os.environ['S17_BENCHMARK'] not in ['', 'controller_prefix','dynamic_prefix'] or (os.environ['S17_BENCHMARK'] and (s.get('scope') not in ['PAIRED_PREFIX','DEVELOPMENT_NOT_FORMAL'] or s.get('operation')!='inspection')):raise ValueError('BENCHMARK_SCOPE')
    if os.environ['S17_BENCHMARK']=='dynamic_prefix' and s.get('scope')!='DEVELOPMENT_NOT_FORMAL':raise ValueError('DEVELOPMENT_ONLY')
    if s['controller'] not in (['DWB','RPP','MPPI'] if s['vehicle']=='diff' else ['RPP','MPPI']):raise ValueError('CONTROLLER_CHASSIS_CONTRACT')
    if os.environ['S17_OPERATION'] not in ['cluster','inspection']:raise ValueError('UNKNOWN_OPERATION')
    from inputs17 import snapshot
    snapshot(a.out)
    os.environ.pop('S17_SURVEY_STATIC',None);os.environ.pop('S17_RESUME_MANIFEST',None)
    if a.resume_from:
        from resume17 import prepare
        os.environ['S17_RESUME_MANIFEST']=str(prepare(a.resume_from,a.out,s['cluster']))
    overlay=build(a.out/'overlay',RESULTS/'qualification/stations.json',a.rtf,full=True);sys.path.insert(0,str(overlay))
    from runtime14 import run_case
    r=run_case(a.out,'zone_a','nominal_spray_cluster' if s['vehicle']=='diff' else 'nominal_mow_cluster',a.seconds,a.wall,seed=s['seed'],gui=a.gui,kind=s['vehicle'],controller=s['controller'])
    return 0 if r.get('success') else 1
if __name__=='__main__':raise SystemExit(main())
