"""Meaningful inference invariants, not stage acceptance or synthetic benchmarks."""
from pathlib import Path
import sys,json,time,copy
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np
from history15 import prepare
from imm15 import IMM
from trajectron15 import Trajectron

def main(out):
    rows=[]
    for i in range(101):
        t=i*.1
        rows.append(dict(t=t,id=1,position=[2*np.sin(.4*t),2*(1-np.cos(.4*t))],velocity=[.8*np.cos(.4*t),.8*np.sin(.4*t)],covariance=[[.01,0],[0,.01]],classification='person',confidence=.8,size=[.5,.5,1.8]))
    prefix=prepare(rows,6.)
    changed=copy.deepcopy(rows)
    for r in changed:
        if r['t']>6:r['position']=[999.,-999.];r['classification']='other'
    assert prepare(changed,6.)==prefix
    imm=IMM(rows[0]['t'],rows[0]['position'],rows[0]['covariance'],rows[0]['velocity'])
    for r in rows[1:61]:imm.update(r['t'],r['position'],r['covariance'])
    ip=imm.predict();assert ip['modes'][2]['probability']>ip['modes'][3]['probability']
    model=Trajectron();r=model.predict(prefix);again=model.predict(prepare(changed,6.))
    assert r['modes']==again['modes']
    neighbor=copy.deepcopy(prefix);neighbor['id']=2
    for state in neighbor['state']:state[0]+=.8;state[1]+=.4;state[2]*=-.5
    nr=model.predict(prefix,[neighbor]);assert nr['neighbors']==[2]
    assert np.max(np.abs(np.array([m['mean'] for m in nr['modes']])-np.array([m['mean'] for m in r['modes']])))>1e-5
    shapes=[]
    for label,p in [('imm',ip),('learned',r)]:
        weights=np.array([m['probability'] for m in p['modes']]);means=np.array([m['mean'] for m in p['modes']]);cov=np.array([m['covariance'] for m in p['modes']])
        assert abs(weights.sum()-1)<1e-5 and np.all(weights>=0)
        assert np.isfinite(means).all() and np.isfinite(cov).all() and np.linalg.eigvalsh(cov).min()>=0
        assert np.max(np.linalg.norm(means[:,-1]-means[0,-1],axis=1))>.1
        shapes.append(dict(model=label,modes=len(weights),minimum_eigenvalue=float(np.linalg.eigvalsh(cov).min()),probability_sum=float(weights.sum())))
    # Keeping the upstream final sample calculation must not change its distribution.
    full=model.predict(prefix,horizon=2.4,distribution_only=False)
    fast=model.predict(prefix,horizon=2.4,distribution_only=True)
    assert full['modes']==fast['modes']
    stopped=copy.deepcopy(prefix)
    for state in stopped['state'][-4:]:state[:2]=stopped['state'][-1][:2];state[2:]=[0.,0.,0.,0.]
    stop=model.predict(stopped);assert stop['modes']!=r['modes']
    try:imm.update(imm.t-1,[0,0],[[1,0],[0,1]]);raise AssertionError('clock rewind accepted')
    except ValueError:pass
    results=dict(passed=True,scope='S15_COMPONENT_INFERENCE_ONLY',future_prefix_invariant=True,neighbor_conditioning_changes_output=True,stop_go_changes_distribution=True,distribution_patch_equivalent=True,clock_rewind_rejected=True,probability_covariance=shapes,first_inference_ms=r['wall_ms'],full_unused_grid_ms=full['wall_ms'],distribution_only_ms=fast['wall_ms'],patch_hashes=model.patch_hashes,module_keys=model.module_keys)
    Path(out).parent.mkdir(parents=True,exist_ok=True);Path(out).write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results))
if __name__=='__main__':main(sys.argv[1])
