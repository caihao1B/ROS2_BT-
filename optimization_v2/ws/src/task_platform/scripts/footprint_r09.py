"""Rear-axle body coordinates must rotate about the vehicle origin, not centroid."""
from shapely.geometry import Polygon
from shapely.affinity import rotate,translate

def body_polygon(vertices,x,y,yaw):
 return translate(rotate(Polygon(vertices),yaw,origin=(0,0),use_radians=True),x,y)
