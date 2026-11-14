import unittest,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from footprint_r09 import body_polygon
from shapely.geometry import Point
class RearAxleFootprint(unittest.TestCase):
 def setUp(self):self.fp=[[-.571,-1.385],[2.209,-1.385],[2.209,1.385],[-.571,1.385]]
 def test_quarter_turn_preserves_rear_axle_origin(self):
  p=body_polygon(self.fp,10,20,math.pi/2);self.assertAlmostEqual(p.centroid.x,10);self.assertAlmostEqual(p.centroid.y,20.819);self.assertAlmostEqual(p.bounds[3],22.209)
 def test_front_corner_is_rotated_about_rear_axle(self):
  h=.261799;px=2.209*math.cos(h)-1.385*math.sin(h);py=2.209*math.sin(h)+1.385*math.cos(h);p=body_polygon(self.fp,0,0,h);self.assertLess(p.boundary.distance(Point(px,py)),1e-10)
 def test_padding_is_not_part_of_current_body(self):self.assertFalse(body_polygon(self.fp,0,0,0).covers(Point(0,1.43)))
