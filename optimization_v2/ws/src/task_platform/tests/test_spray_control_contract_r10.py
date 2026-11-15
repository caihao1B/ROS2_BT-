"""Prevent the RPP angular command / actuator limit mismatch observed in short run."""
import unittest,sys,json,yaml,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import edition
class SprayCurvatureContract(unittest.TestCase):
 def test_nominal_curvature_regulation_matches_original_actuator_cap(self):
  root=edition('R10')/'spray';cfg=json.loads((root/'mission.json').read_text());rpp=yaml.safe_load((root/'nav2.yaml').read_text())['controller_server']['ros__parameters']['RPP']
  self.assertLessEqual(rpp['desired_linear_vel']/rpp['regulated_linear_scaling_min_radius'],cfg['yaw_cap_rps'])
  self.assertLessEqual(rpp['rotate_to_heading_angular_vel'],cfg['yaw_cap_rps'])
  self.assertLessEqual(rpp['desired_linear_vel'],cfg['speed_cap_mps'])
 def test_controller_does_not_latch_a_past_xy_arrival(self):
  root=edition('R10')/'spray';controller=yaml.safe_load((root/'nav2.yaml').read_text())['controller_server']['ros__parameters']
  self.assertFalse(controller['goal']['stateful']);self.assertFalse(controller['RPP']['stateful'])

 def test_near_goal_curvature_floor_cannot_force_angular_saturation(self):
  root=edition('R10')/'spray';cfg=json.loads((root/'mission.json').read_text());controller=yaml.safe_load((root/'nav2.yaml').read_text())['controller_server']['ros__parameters'];rpp=controller['RPP']
  # Outside the navigation XY gate, larger bearings trigger rotate-to-path.
  k=2*math.sin(rpp['rotate_to_heading_min_angle'])/controller['goal']['xy_goal_tolerance']
  self.assertLessEqual(k*rpp['regulated_linear_scaling_min_speed'],cfg['yaw_cap_rps'])
  self.assertLessEqual(controller['goal']['xy_goal_tolerance'],cfg['docking_admission']['navigation_goal_m'])

 def test_local_dock_controller_retains_original_motion_and_collision_limits(self):
  root=edition('R10')/'spray';cfg=json.loads((root/'mission.json').read_text());controller=yaml.safe_load((root/'nav2.yaml').read_text())['controller_server']['ros__parameters'];rpp=controller['DockRPP']
  self.assertLessEqual(rpp['desired_linear_vel']/rpp['regulated_linear_scaling_min_radius'],cfg['yaw_cap_rps'])
  self.assertLessEqual(2*math.sin(rpp['rotate_to_heading_min_angle'])/controller['dock_goal']['xy_goal_tolerance']*rpp['regulated_linear_scaling_min_speed'],cfg['yaw_cap_rps'])
  self.assertTrue(rpp['use_collision_detection']);self.assertFalse(rpp['stateful'])
  self.assertLessEqual(controller['dock_goal']['xy_goal_tolerance'],cfg['docking_admission']['navigation_goal_m'])
