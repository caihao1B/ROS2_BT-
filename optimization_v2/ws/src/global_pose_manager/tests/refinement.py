"""Verify real rigid fit and refusal outside the tiny correction basin."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from s07_common import *
from map_refinement import refine
from scipy.spatial import cKDTree
rng=np.random.default_rng(41);raw=rng.uniform([-5,-4,.3],[5,4,3],(1600,3));p=np.array([4.,2.,0.]);q=Rotation.from_euler('z',.3).as_quat();tree=cKDTree(transform(raw,p,q));initial=q.copy();initial=Rotation.from_euler('z',.31).as_quat();r,d=refine(raw,p+[.03,0,0],initial,tree);checks=dict(small_error_refined=r is not None and np.linalg.norm(r['position']-p)<.005 and np.quantile(r['distance'],.95)<.01)
r,d=refine(raw,p+[.3,0,0],q,tree);checks['large_change_rejected']=r is None
r,d=refine(raw[:50],p,q,tree);checks['too_few_points_rejected']=r is None
line=np.c_[np.linspace(-3,3,500),np.zeros(500),np.ones(500)];r,d=refine(line,[0,0,0],[0,0,0,1],cKDTree(line));checks['degenerate_line_rejected']=r is None
checks={k:bool(v) for k,v in checks.items()};print(json.dumps(checks));raise SystemExit(0 if all(checks.values()) else 1)
