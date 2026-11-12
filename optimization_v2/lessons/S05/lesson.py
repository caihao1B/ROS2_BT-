"""Fixed-frame ATE/RPE and the effect of covariance; intentionally no trajectory refitting."""
import math
import numpy as np
t=np.arange(0,10,.1);truth=np.column_stack((t,np.zeros_like(t)));drifted=truth+np.column_stack((.1*t,np.zeros_like(t)))
ate=float(np.sqrt(np.mean(np.sum((drifted-truth)**2,axis=1))));rpe=float(np.sqrt(np.mean(np.sum(((drifted[10:]-drifted[:-10])-(truth[10:]-truth[:-10]))**2,axis=1))))
assert .56<ate<.58 and abs(rpe-.1)<1e-12
def update(r):p=.04;return .5+p/(p+r)*(.8-.5)
assert update(.0025)>update(4.)>.5
assert not np.allclose(drifted,truth)
print(f'PASS fixed-frame ATE={ate:.3f} m; 1 s RPE={rpe:.3f} m; never fit away drift; covariance update {update(.0025):.3f} vs {update(4.):.3f}')
