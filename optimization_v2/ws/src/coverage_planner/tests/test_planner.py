"""Production planner tests: barriers, kinematics, accounting and stale inputs."""
import sys,unittest,tempfile,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common12 import *
from motion12 import *
from spray12 import visibility,nearest_2opt
from planning12 import resized_domain,rebalanced
from contracts12 import original_infeasible,frame_binding,check_binding

class PlannerTests(unittest.TestCase):
    def setUp(self):self.c=config();self.motion=box(-20,-20,20,20)
    def test_obstacle_matrix_detours(self):
        motion=box(0,0,20,20).difference(box(8,6,12,14));cost,paths,_=visibility([[3,10],[17,10]],motion,self.c)
        self.assertGreater(cost[0,1],14);self.assertTrue(motion.covers(LineString(paths[0,1])))
    def test_disconnected_task_points_rejected(self):
        with self.assertRaisesRegex(ValueError,'DISCONNECTED'):visibility([[3,3],[17,17]],union_all([box(0,0,6,6),box(14,14,20,20)]),self.c)
    def test_unreachable_point_is_not_dropped(self):
        with self.assertRaisesRegex(ValueError,'UNREACHABLE'):visibility([[.01,.01],[5,5]],box(0,0,10,10),self.c)
    def test_tsp_permutation_and_two_opt(self):
        xy=np.array([[0,0],[8,0],[0,8],[8,8],[4,2]]);m=np.linalg.norm(xy[:,None]-xy[None,:],axis=2);o,v=nearest_2opt(m)
        self.assertEqual(sorted(o),list(range(5)));self.assertLess(v,40)
    def test_actual_ortools_backend(self):
        with tempfile.TemporaryDirectory() as t:
            r=backend(dict(mode='tsp',cost=[[0,4,1,8],[4,0,7,1],[1,7,0,4],[8,1,4,0]]),Path(t),'tsp');self.assertEqual(sorted(r['order']),[0,1,2,3]);self.assertEqual(r['cost'],10)
    def test_f2c_hole_swaths(self):
        with tempfile.TemporaryDirectory() as t:
            work=box(0,0,18,16).difference(box(7,5,10,11));r=backend(dict(mode='swaths',work=mapping(work),motion=mapping(box(-8,-8,26,24)),spacing=.65,headland=4.22),Path(t),'hole')
            self.assertGreater(len(r['swaths']),0)
            for row in r['swaths']:self.assertLess(LineString([row['a'],row['b']]).difference(work).length,1e-6)
    def test_actual_f2c_turn_validated(self):
        with tempfile.TemporaryDirectory() as t:
            r=backend(dict(mode='turns',radius=2.11,ds=.04,queries=[dict(id=0,start=[0,0,0],goal=[0,3,math.pi])]),Path(t),'turn')
            for row in r['turns']:
                s=segment(row['poses'],'TURN','ackermann',self.motion,self.c,stop_curvature=not row['continuous_curvature']);self.assertTrue(s['validation']['passed'])
                if not row['continuous_curvature']:self.assertGreater(len(s['timing']['steering_events']),0)
    def test_ackermann_stationary_yaw_refused(self):
        self.assertFalse(segment([[0,0,0],[0,0,1]],'ROTATE','ackermann',self.motion,self.c)['validation']['passed'])
    def test_diff_stationary_rotation_and_acceleration(self):
        s=rotate_at([0,0,0],math.pi/2,self.motion,self.c);self.assertTrue(s['validation']['passed']);t=s['timing'];self.assertEqual(t['yaw_rates'][0],0);self.assertAlmostEqual(t['yaw_rates'][-1],0);self.assertLessEqual(t['max_yaw_rate'],.3)
    def test_tight_radius_refused(self):
        poses=dubins.sample([0,0,0],[(1.,2.)],.05)['poses'];self.assertFalse(segment(poses,'TURN','ackermann',self.motion,self.c)['validation']['passed'])
    def test_lateral_motion_refused(self):
        self.assertFalse(segment([[0,0,0],[0,1,0]],'TRANSIT','ackermann',self.motion,self.c)['validation']['passed'])
    def test_swept_footprint_not_just_centre(self):
        poses=straight([0,.3],[8,.3],.05);s=segment(poses,'TRANSIT','ackermann',box(-2,0,12,10),self.c);self.assertFalse(s['validation']['collision_free'])
    def test_tiny_segment_has_positive_time(self):
        s=segment(straight([0,0],[.01,0],.08),'TRANSIT','diff',self.motion,self.c);self.assertGreater(s['timing']['motion_seconds'],0);self.assertTrue(s['validation']['passed'])
    def test_duplicate_pose_normalization_not_yaw_teleport(self):
        self.assertEqual(len(normalize_poses([[0,0,0],[0,0,0],[1,0,0]])),2);self.assertEqual(len(normalize_poses([[0,0,0],[0,0,1]])),2)
    def test_spray_time_not_double_counted(self):
        s=segment([[0,0,0]],'POINT_SERVICE','diff',self.motion,self.c,tool=True);t=summarize([s],self.c,1);self.assertEqual(t['tool_switch_s'],3.5);self.assertEqual(t['dwell_and_dock_s'],3)
    def test_scope_preserves_original_obligations(self):
        m,p=resized_domain('B',.2,self.c);self.assertGreater(m['original_domain'].area,m['domain'].area);self.assertAlmostEqual(m['domain'].area+m['original_domain'].difference(m['domain']).area,m['original_domain'].area,places=5)
    def test_original_budget_is_infeasible(self):
        r=original_infeasible(self.c);self.assertGreater(r['spray']['optimistic_total_s'],r['hard_total_budget_s']);self.assertGreater(r['mowing']['optimistic_total_s'],r['hard_total_budget_s'])
    def test_surveyed_frame_not_relabel(self):
        f=frame_binding();self.assertAlmostEqual(f['translation'][0],-28);self.assertAlmostEqual(f['translation'][1],-14);self.assertFalse(f['nonrigid_map_error_corrected'])
    def test_each_version_and_plan_tamper_refused(self):
        bindings={'map':'a','vehicle':'b','tool':'c','config':'d'};p=dict(status='VALIDATED',online_execution_allowed=False,versions=bindings,payload={'work':1});p['plan_revision']=identity(p)
        self.assertTrue(check_binding(p,bindings))
        for name in bindings:
            altered=bindings|{name:'CHANGED'}
            with self.assertRaisesRegex(ValueError,'STALE'):check_binding(p,altered)
        q=copy.deepcopy(p);q['payload']['work']=2
        with self.assertRaisesRegex(ValueError,'PLAN_HASH'):check_binding(q,bindings)
    def test_forward_speed_accel_and_decel(self):
        s=segment(straight([0,0],[5,0],.08),'COVERAGE','ackermann',self.motion,self.c,tool=True);t=s['timing'];self.assertTrue(s['validation']['passed']);self.assertLessEqual(t['max_speed'],.12);self.assertEqual(t['velocities'][0],0);self.assertEqual(t['velocities'][-1],0)
if __name__=='__main__':unittest.main(verbosity=2)
