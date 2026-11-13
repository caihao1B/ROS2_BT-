"""Sequential actual runs; stop at the first unresolved job, never fake PASS."""
from common_showcase import *
import subprocess,argparse,datetime
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--start',choices=['A1','A2','A3','B1','B2','B3','road_diff','road_ackermann'],default='A1');ap.add_argument('--work-only',action='store_true');a=ap.parse_args()
    jobs=[(k,'work',k) for k in ['A1','A2','A3','B1','B2','B3']]
    if not a.work_only:jobs += [('A1','inspection','road_diff'),('B1','inspection','road_ackermann')]
    start=next(i for i,j in enumerate(jobs) if j[2]==a.start);batch=OUT/('batch_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.json');results=[]
    for cluster,operation,key in jobs[start:]:
        args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/run_showcase.py'),'--cluster',cluster,'--operation',operation]
        previous=read(OUT/'current.json').get('run') if (OUT/'current.json').exists() else None
        print('START',key,flush=True);code=subprocess.call(args);current=read(OUT/'current.json');out=ROOT/current['run'];created=current['run']!=previous and current['spec']['cluster']==cluster and current['spec']['operation']==operation
        result=read(out/'showcase_result.json') if created and (out/'showcase_result.json').exists() else dict(status='ERROR' if created else 'NOT_RUN',complete=False)
        results.append(dict(key=key,run=current['run'] if created else None,exit_code=code,status=result['status']));dump(batch,dict(scope='INDEPENDENT_SHOWCASE_ONLY',results=results,formal_credit=False));print('END',key,result['status'],flush=True)
        if code or result['status']!='PASS':return 1
    return 0
if __name__=='__main__':raise SystemExit(main())
