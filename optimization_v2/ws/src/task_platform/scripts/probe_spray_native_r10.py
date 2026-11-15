"""Isolated actual C++ guards and candidate-GNSS recovery; no work credit."""
from pathlib import Path
import sys,hashlib
from common import APP,read,dump,sha

def run(variant):
    template=Path(__file__).with_name('probe_global_r10.py' if variant=='global' else 'probe_revalidation_r10.py')
    source=template.read_text();old='build/r10_global/r10_global_manager';assert source.count(old)==1
    source=source.replace(old,'build/r10_spray_global/r10_global_manager')
    folder='global_filter_probe' if variant=='global' else 'revalidation_probe'
    if variant=='candidate':
        source=source.replace('results/R10/revalidation_probe','results/R10/candidate_gnss_probe');folder='candidate_gnss_probe'
        source=source.replace('t=0.;yaw=0.;', 't=0.;yaw=0.;mx=0.;gx=0.;fix_valid=True;')
        source=source.replace('pose=[0.,0.,0.,0.,0.,math.sin(yaw/2)', 'pose=[mx,0.,0.,0.,0.,math.sin(yaw/2)')
        source=source.replace("gnss=dict(t=t,valid=True,mode='fixed'", "gnss=dict(t=t,valid=fix_valid,mode='fixed'")
        source=source.replace('antenna=[0.,0.,0.]','antenna=[gx,0.,0.]')
        marker="moving=False;until(lambda:status[-1]['recovery_ready'],8.);r=call(prepare)"
        assert source.count(marker)==1
        replacement="""moving=False;mx=.14;gx=.32
  for _ in range(40):tick()
  checks['candidate_rejects_independent_gnss_conflict']=not status[-1]['recovery_ready'] and not call(prepare).success
  gx=.16;fix_valid=False
  for _ in range(20):tick()
  checks['candidate_rejects_invalid_fix']=not status[-1]['recovery_ready'] and not call(prepare).success
  fix_valid=True
  until(lambda:status[-1]['gnss_reason']=='INNOVATION_REJECTED_FIXED' and status[-1]['gps_count']==0,4.)
  checks['old_reference_gnss_stays_rejected']=not status[-1]['navigation_allowed']
  until(lambda:status[-1]['recovery_ready'],8.)
  checks['independent_candidate_with_same_numeric_gates_ready']=status[-1]['gps_count']==0 and status[-1]['plan_revalidation_required'] and not status[-1]['navigation_allowed']
  r=call(prepare)"""
        source=source.replace(marker,replacement)
    if variant!='global':
        old="proof['path_valid']=True;tick();r=call(commit);checks['valid_transaction_commits']=r.success;"
        assert source.count(old)==1
        new="proof['path_valid']=True\n  for _ in range(3):\n   tick(.10);r=call(commit)\n   if r.success:break\n  checks['valid_transaction_commits']=r.success;"
        source=source.replace(old,new)
    namespace=dict(globals(),__name__='spray_native_probe');exec(compile(source,str(template),'exec'),namespace)
    code=namespace['run']();path=APP/'results/R10'/folder/'result.json';report=read(path)
    report.update(template_sha256=sha(template),derived_source_sha256=hashlib.sha256(source.encode()).hexdigest(),variant=variant)
    dump(path,report);return code

if __name__=='__main__':raise SystemExit(run(sys.argv[1] if len(sys.argv)>1 else 'candidate'))
