"""Exercise the generated production input adapter against timestamped sensor cases."""
import sys,tempfile,unittest,importlib.util,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import *
from overlay_r07 import overlay
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
class Alignment(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name);overlay('spray',self.path/'overlay');spec=importlib.util.spec_from_file_location('r07_inputs_test',self.path/'overlay/input_adapter.py');self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module);rclpy.init(domain_id=125);self.obj=self.module.Inputs(self.path/'inputs')
 def tearDown(self):
  self.obj.f.close();self.obj.n.destroy_node();rclpy.shutdown();self.tmp.cleanup()
 def wheel(self,t,v):
  m=TwistWithCovarianceStamped();m.header.stamp.sec=int(t);m.header.stamp.nanosec=round((t-int(t))*1e9);m.twist.twist.linear.x=v;self.obj.t=t;self.obj.wheel(m)
 def quality(self,t,v,var,start=None):self.obj.icp_data=dict(t=t,valid=True,v=v,window_start=t-.8 if start is None else start,window_end=t,increment_variance=var)
 def test_acceleration_compares_actual_window(self):
  for i in range(1,126):
   t=i*.02;v=min(.4,t*.4)
   if i>=40 and i%10==0:
    values=[min(.4,j*.02*.4) for j in range(max(1,i-40),i+1)];self.quality(t,sum(values)/len(values),.0025)
   self.wheel(t,v);self.assertFalse(self.obj.slipping)
  self.assertGreater(len(self.obj.wheel_history),80)
 def test_uncertain_lidar_does_not_falsely_degrade_wheels(self):
  for i in range(1,101):
   t=i*.02;self.quality(t,.1,.25);self.wheel(t,.3)
  self.assertFalse(self.obj.slipping)
 def test_persistent_confident_disagreement_marks_suspected_slip(self):
  for i in range(1,101):
   t=i*.02;self.quality(t,.2,.0001);self.wheel(t,.7)
  self.assertTrue(self.obj.slipping)
 def test_confident_agreement_restores_normal_covariance(self):
  for i in range(1,101):
   t=i*.02;self.quality(t,.2,.0001);self.wheel(t,.7)
  self.assertTrue(self.obj.slipping)
  for i in range(101,181):
   t=i*.02;self.quality(t,.2,.0001);self.wheel(t,.2)
  self.assertFalse(self.obj.slipping)
if __name__=='__main__':unittest.main()
