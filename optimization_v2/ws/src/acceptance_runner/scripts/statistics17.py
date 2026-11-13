"""Intervals describe sampled seeds; never claim population-wide safety."""
import math,numpy as np
def wilson(success,total,z=1.959963984540054):
    if not total:return None
    p=success/total;d=1+z*z/total;c=(p+z*z/(2*total))/d
    e=z*math.sqrt(p*(1-p)/total+z*z/(4*total*total))/d
    return [max(0.,c-e),min(1.,c+e)]
def quantiles(values):
    a=np.array(values,float)
    if not len(a):return None
    if not np.isfinite(a).all():raise ValueError('NONFINITE_MEASUREMENT')
    return dict(n=len(a),p50=float(np.quantile(a,.5)),p95=float(np.quantile(a,.95)),maximum=float(a.max()))
def paired(rows,left,right,metric):
    groups={}
    for r in rows:
        if r['method'] not in [left,right]:continue
        k=(r['scenario'],r['seed'],r['task_hash'],r['limits_hash'])
        if r['method'] in groups.setdefault(k,{}):raise ValueError('DUPLICATE_PAIR')
        groups[k][r['method']]=r
    matches=[v for v in groups.values() if left in v and right in v]
    return dict(pairs=len(matches),unpaired=len(groups)-len(matches),difference=quantiles([v[right][metric]-v[left][metric] for v in matches]))
