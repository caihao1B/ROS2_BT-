"""DARP-inspired weighted graph adaptation, not the original STC guarantee.

Source: Kapoutsis et al., DARP, https://kapoutsis.info/wp-content/uploads/2017/02/j3.pdf
Distance evaluation, load multipliers and disconnected-component correction;
irregular partial cells use workload weights and the common connectivity repair.
"""
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

def divide(g,k,seed,max_iterations=200):
    from geometry11 import components
    n=len(g['cells']);row=[];col=[];cost=[]
    for i,neighbors in enumerate(g['adj']):
        for j in neighbors:row.append(i);col.append(j);cost.append(float(np.linalg.norm(g['xy'][i]-g['xy'][j])))
    graph=csr_matrix((cost,(row,col)),shape=(n,n));rng=np.random.default_rng(seed);seeds=[int(rng.integers(n))];distance=dijkstra(graph,indices=seeds[0],directed=False)
    for _ in range(1,k):
        nxt=int(np.argmax(distance));seeds.append(nxt);distance=np.minimum(distance,dijkstra(graph,indices=nxt,directed=False))
    base=dijkstra(graph,indices=seeds,directed=False)+1e-6
    if not np.isfinite(base).all():raise ValueError('DISCONNECTED_DARP_INPUT')
    multipliers=np.zeros(k);correction=np.ones((k,n));target=sum(g['weights'])/k;best=None;trace=[]
    for iteration in range(max_iterations):
        score=base*np.exp(multipliers[:,None])*correction;score[:,seeds]=np.inf
        for i,s in enumerate(seeds):score[i,s]=0.
        labels=score.argmin(0);loads=np.bincount(labels,weights=g['weights'],minlength=k);parts=[components(g['adj'],np.flatnonzero(labels==i)) for i in range(k)];bad=sum(len(p)-1 for p in parts);imbalance=float(np.max(np.abs(loads/target-1)))
        criterion=(bad,imbalance)
        if best is None or criterion<best[0]:best=(criterion,labels.copy())
        trace.append(dict(iteration=iteration,disconnected_components=bad,relative_load_error=imbalance))
        if bad==0 and imbalance<=.05:break
        multipliers+=.04*(loads/target-1);multipliers-=multipliers.mean();multipliers=np.clip(multipliers,-4,4)
        correction[:]=1.
        for i,ps in enumerate(parts):
            if len(ps)<2:continue
            core=next(p for p in ps if seeds[i] in p);other=[u for p in ps if p!=core for u in p]
            near=dijkstra(graph,indices=core,directed=False,min_only=True);far=dijkstra(graph,indices=other,directed=False,min_only=True);delta=near-far;scale=max(1e-9,np.max(np.abs(delta)));correction[i]=1+.15*delta/scale
    return best[1],dict(method='DARP-inspired weighted graph adaptation + shared repair; no original uniform-grid optimality claim',seeds=seeds,iterations=len(trace),trace=trace,best_disconnected_components=best[0][0],best_relative_load_error=best[0][1])
