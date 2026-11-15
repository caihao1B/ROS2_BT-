"""Execute both protected coarse-policy and precision-motion tests with actual BT XML."""
from common import *

def main():
 out=APP/'results/R10/precision_progress_probe';out.mkdir(parents=True,exist_ok=True)
 commands=[['original_policy_test',str(edition('R06')/'spray.json'),str(PKG/'r08_backend/common.xml')],
           ['precision_progress_test',str(edition('R06')/'spray.json'),str(edition('R06')/'common.xml')]]
 records=[]
 for name,*args in commands:
  r=subprocess.run([str(APP/'build/r10_spray_bt'/name),*args],capture_output=True,text=True,timeout=10)
  records.append(dict(name=name,exit=r.returncode,stdout=r.stdout,stderr=r.stderr))
 result=dict(passed=all(r['exit']==0 for r in records),cases=records,
             binary_sha256=sha(APP/'build/r10_spray_bt/r10_spray_supervisor'),
             policy_sha256=sha(edition('R10')/'spray/mission.json'),
             header_sha256=sha(PKG/'r10_backend/precision_progress.hpp'),
             driver_sha256=sha(Path(__file__)),generator_sha256=sha(PKG/'scripts/precision_supervisor_r10.py'),
             scope='Actual BehaviorTree.CPP policy/XML tests, separate from full A1 execution')
 dump(out/'result.json',result);print(json.dumps(result));return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
