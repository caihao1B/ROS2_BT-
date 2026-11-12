"""Offline teaching: connectivity is NOT equivalent to equal rectangular partitions."""
from shapely.geometry import Polygon, MultiPolygon, box
from shapely import constrained_delaunay_triangles
def validate(p):
    if p.geom_type!='Polygon' or not p.is_valid or p.is_empty:raise ValueError('not one valid workspace')
    return p.area
def main():
    p=Polygon([(0,0),(10,0),(10,4),(6,4),(6,8),(0,8)],holes=[[(1,1),(1,2),(2,2),(2,1)]])
    area=validate(p);assert area==63
    assert abs(sum(t.area for t in constrained_delaunay_triangles(p).geoms)-area)<1e-9
    rejected=0
    for bad in [Polygon([(0,0),(2,2),(2,0),(0,2)]),MultiPolygon([box(0,0,1,1),box(3,0,4,1)])]:
        try:validate(bad)
        except ValueError:rejected+=1
    assert rejected==2
    print('PASS concave polygon with a hole: area=63 m2; triangles preserve holes; 2 invalid cases rejected')
if __name__=='__main__':main()
