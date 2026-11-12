"""Teaching: fixed-heading linear 2D pose graph; production is RTAB-Map/GTSAM SE(3)."""
import numpy as np
def solve(edges):
    h=np.zeros((5,5));b=np.zeros((5,2));h[0,0]=1e6
    for i,j,delta,information in edges:
        if not np.isfinite(information) or information<=0:raise ValueError('non-positive information')
        a=np.zeros(5);a[i]=-1;a[j]=1;h+=information*np.outer(a,a);b+=information*np.outer(a,delta)
    if np.linalg.matrix_rank(h)!=5:raise ValueError('disconnected unobservable graph')
    return np.linalg.solve(h,b)
def main():
    edges=[(0,1,[2.1,0],100),(1,2,[0,2.1],100),(2,3,[-2,0],100),(3,4,[0,-2],100)]
    raw=solve(edges);optimized=solve(edges+[(4,0,[0,0],400)])
    before=np.linalg.norm(raw[4]-raw[0]);after=np.linalg.norm(optimized[4]-optimized[0]);assert after<before
    for bad in [[(0,1,[2,0],-1)],edges[:1]]:
        try:solve(bad)
        except ValueError:pass
        else:raise AssertionError('invalid factor graph accepted')
    print(f'Teaching graph closure: {before:.6f} -> {after:.6f} m; both boundary cases rejected')
if __name__=='__main__':main()
