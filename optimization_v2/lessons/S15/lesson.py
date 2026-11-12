"""Runnable use of the production IMM; synthetic teaching data, not acceptance."""
from pathlib import Path
import sys,math
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws/src/motion_prediction/scripts'))
from imm15 import IMM
f=IMM(0.,[0.,0.],[[.01,0.],[0.,.01]],[.8,0.])
for i in range(1,81):
    t=i*.1;f.update(t,[2*math.sin(.4*t),2*(1-math.cos(.4*t))],[[.01,0.],[0.,.01]])
p=f.predict()
for m in p['modes']:print(m['mode'],round(m['probability'],4),'future 3.2 s',m['mean'][-1])
assert abs(sum(m['probability'] for m in p['modes'])-1)<1e-6
try:f.update(7.,[0,0],[[.01,0],[0,.01]])
except ValueError:print('PASS: clock rewind rejected, reset must be explicit')
else:raise AssertionError('Invalid time accepted')
