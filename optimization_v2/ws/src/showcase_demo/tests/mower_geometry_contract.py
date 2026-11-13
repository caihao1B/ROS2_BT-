"""Physical reference, task-scale and connected partition regression checks."""
import sys, unittest, math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/showcase_demo/scripts'))
from mower_geometry_audit import vehicle_metrics, route_metrics, scale_warnings, read, BASE, OUT
import yaml
from mower_scale_candidate import balanced_strip_partition
from shapely.geometry import Polygon, shape


class MowerGeometryContract(unittest.TestCase):
    def vehicle(self):
        return vehicle_metrics(yaml.safe_load((BASE/'config/vehicles/ackermann.yaml').read_text()),
            read(BASE/'config/vehicles/qualified_profiles.json')['vehicles']['ackermann'],
            yaml.safe_load((BASE/'config/tool_profiles.yaml').read_text())['mower'])

    def test_inner_wheel_joint_limit_is_not_bicycle_angle(self):
        v=self.vehicle()
        self.assertAlmostEqual(v['kinematic_joint_limit_radius_m'],1/math.tan(.6)+.35)
        self.assertGreater(v['kinematic_joint_limit_radius_m'],1/math.tan(.6))

    def test_body_turning_envelope_exceeds_axle_diameter(self):
        v=self.vehicle()
        self.assertGreater(v['footprint_outer_circle_diameter_m'],v['rear_axle_turn_diameter_m'])
        self.assertTrue(v['physical_parameters_consistent'])

    def test_recorded_tiny_task_is_flagged_without_rewriting_it(self):
        zone=read(OUT/'v1/mission_plan.json')['zones']['B'];metrics=route_metrics(zone)
        self.assertAlmostEqual(metrics['task_area_m2'],4.104,places=5)
        self.assertIn('TASK_SHORTER_THAN_TURN_DIAMETER',scale_warnings(metrics,self.vehicle()))
        self.assertTrue(all(c['tool_off_distance_fraction']>.9 for c in metrics['clusters']))

    def test_convex_irregular_parcel_is_balanced_connected_and_complete(self):
        domain=Polygon([(0,0),(8,0),(9,3),(8,5),(1,5),(-1,2)])
        p=balanced_strip_partition(domain)
        self.assertTrue(p['verification']['passed'])
        for f in p['clusters']['features']:
            self.assertAlmostEqual(shape(f['geometry']).area,domain.area/3,places=8)

    def test_nonconvex_parcel_requires_other_partition_algorithm(self):
        domain=Polygon([(0,0),(8,0),(8,5),(5,5),(5,2),(3,2),(3,5),(0,5)])
        with self.assertRaisesRegex(ValueError,'CONVEX'):balanced_strip_partition(domain)

    def test_large_task_can_still_be_inefficient(self):
        m=dict(task_bbox_size_m=[20.,15.],clusters=[dict(tool_off_distance_fraction=.9)])
        self.assertEqual(scale_warnings(m,self.vehicle()),['TOOL_OFF_DISTANCE_DOMINATES_OVER_75_PERCENT'])


if __name__=='__main__':unittest.main()
