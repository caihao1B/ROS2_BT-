"""S18 stage protocol. Early delivery audits are explicitly partial."""
from common18 import *
import argparse,subprocess,time
def main():
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['demo','test','stop']);ap.add_argument('--case',choices=['delivery_audit','release_smoke'])
    ap.add_argument('--suite',choices=['acceptance']);ap.add_argument('--seed',type=int,default=41)
    ap.add_argument('--sim-seconds',type=int,default=540);ap.add_argument('--wall-timeout',type=int,default=1800);a=ap.parse_args()
    if a.mode=='stop':return subprocess.call(['bash',str(BASE/'run.sh'),'stop'])
    if a.seed!=41 or not 1<=a.sim_seconds<=1800 or not 1<=a.wall_timeout<=7200:ap.error('frozen seed and bounded budgets required')
    if a.mode=='demo' and not a.case:ap.error('demo requires --case')
    if a.mode=='test' and a.suite!='acceptance':ap.error('test requires --suite acceptance')
    if a.mode=='demo' and a.case=='release_smoke':
        if a.sim_seconds<540:ap.error('three smoke entries require at least 540 sim seconds total (180 per entry)')
        remaining=a.wall_timeout;start=time.monotonic()
        for command,args in [('spray',['--cluster','A1']),('mow',['--cluster','B1']),('inspect',['--vehicle','diff'])]:
            remaining=a.wall_timeout-(time.monotonic()-start)
            if remaining<180:raise ValueError('REMAINING_SMOKE_BUDGET_INSUFFICIENT')
            code=subprocess.call(['bash',str(BASE/'run.sh'),command,*args,'--sim-seconds',str(a.sim_seconds//3),'--wall-timeout',str(int(min(remaining,720)))])
            if code not in [0,2]:return code
    from audit18 import audit
    d=audit(RESULTS/(a.mode+'_'+stamp()))
    # Exit zero only means an explicitly offline demo completed; acceptance
    # remains nonzero until every requirement and S17 are actually satisfied.
    return 0 if a.mode=='demo' and a.case=='delivery_audit' else (0 if d['status']=='PASS' else 2)
if __name__=='__main__':raise SystemExit(main())
