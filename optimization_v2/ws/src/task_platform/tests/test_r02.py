import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import *
from geometry_r02 import vehicle
from shapely.geometry import Polygon,box
from shapely.affinity import rotate,translate
class Contracts(unittest.TestCase):
 def test_all_three_profiles_are_versioned(self):
  versions=[]
  for t in TASKS:
   c=read(edition('R02')/'vehicles'/f'{t}.json');self.assertEqual(canonical({k:v for k,v in c.items() if k!='config_hash'}),c['config_hash']);versions.append(c['config_hash'])
  self.assertEqual(len(set(versions)),3)
 def test_physical_wheels_not_visual_dummies(self):
  for t in TASKS:
   c=read(edition('R02')/'vehicles'/f'{t}.json');m=vehicle(c,'test')
   for name in ['rear_left','rear_right','front_left','front_right']:
    self.assertIsNotNone(m.find(f"link[@name='{name}_wheel']/collision"));self.assertIsNotNone(m.find(f"joint[@name='{name}_joint']/axis"))
   self.assertEqual(len(m.findall("joint[@type='revolute']")),6 if t=='mowing' else 4)
 def test_steering_geometry_respects_inner_wheel_limit(self):
  c=read(edition('R02')/'vehicles/mowing.json');L=c['wheelbase'];T=c['wheel']['track'];k=c['steering']['max_curvature']
  a,b=math.atan(L*k/(1-T*k/2)),math.atan(L*k/(1+T*k/2))
  self.assertLessEqual(max(abs(a),abs(b)),c['steering']['joint_limit_rad'])
  self.assertAlmostEqual(L/math.tan(a)+T/2,L/math.tan(b)-T/2,places=9)
  self.assertAlmostEqual(1/k,c['physical_contract']['nominal_bicycle_radius_m'])
 def test_declared_footprint_covers_body_tool_and_steered_wheels(self):
  for t in TASKS:
   c=read(edition('R02')/'vehicles'/f'{t}.json');foot=Polygon(c['footprint']).buffer(1e-8);b=c['body'];x,y=b['center'][:2];a,d=b['size'][:2];self.assertTrue(foot.covers(box(x-a/2,y-d/2,x+a/2,y+d/2)))
   if c['tool']['mass_kg']:
    tool=c['tool'];x,y=tool['mount_xyz'][:2];a,d=tool['collision_size'][:2];self.assertTrue(foot.covers(box(x-a/2,y-d/2,x+a/2,y+d/2)))
   L=c['wheelbase'];T=c['wheel']['track'];r=c['wheel']['radius'];w=c['wheel']['width'];ack=t=='mowing'
   for axle,x in [('rear',0 if ack else -L/2),('front',L if ack else L/2)]:
    for sign in [-1,1]:
     for k in [-c['steering']['max_curvature'],0,c['steering']['max_curvature']]:
      angle=math.atan(L*k/(1-sign*T*k/2)) if ack and axle=='front' else 0
      wheel=translate(rotate(box(-r,-w/2,r,w/2),angle,origin=(0,0),use_radians=True),x,sign*T/2);self.assertTrue(foot.covers(wheel),t)
 def test_driver_does_not_subscribe_truth(self):
  src=(PKG/'scripts/driver_r02.py').read_text();self.assertNotIn('/s03/evaluation',src);self.assertNotIn('worldPose',src)
 def test_dino_cannot_spin_in_place_contract(self):
  c=read(edition('R02')/'vehicles/mowing.json');self.assertFalse(c['physical_contract']['spin_in_place_allowed']);self.assertGreater(c['physical_contract']['nominal_bicycle_radius_m'],2.9)
if __name__=='__main__':unittest.main(verbosity=2)
