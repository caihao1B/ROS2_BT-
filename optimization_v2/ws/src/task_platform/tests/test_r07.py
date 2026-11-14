import sys,unittest,tempfile,copy,yaml
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import *
sys.path.insert(0,str(BASE/'ws/src/work_ledger/scripts'))
from ledger_core import Ledger

class PointService(unittest.TestCase):
    def setUp(self):self.cfg=read(edition('R07')/'mission.json')
    def test_original_obligations_remain_complete(self):
        original=read(edition('R03')/'mission.json')
        self.assertEqual(self.cfg['source_plan_version'],original['version'])
        for route,old in zip(self.cfg['clusters'],original['task_plans']['spray']):
            self.assertEqual(route['original_point_count'],old['point_count'])
            self.assertEqual(len(route['points']),old['point_count'])
            self.assertEqual({x['id'] for x in route['points']},{x['id'] for x in old['service_points']})
    def test_original_supervisor_thresholds_preserved(self):
        original=yaml.safe_load((BASE/'config/localization_supervisor.yaml').read_text());new=read(edition('R07')/'manager.json')
        for key,value in original.items():
            if key not in ['map_id','diagnostic_fixture','nominal_speed']:self.assertEqual(new[key],value,key)
        self.assertFalse(new['diagnostic_fixture']);self.assertEqual(new['nominal_speed'],.4)
    def test_service_requires_position_and_separate_physical_stop(self):
        nav=yaml.safe_load((edition('R07')/'nav2.yaml').read_text());goal=nav['controller_server']['ros__parameters']['goal'];self.assertEqual(goal['plugin'],'nav2_controller::PositionGoalChecker');self.assertFalse(goal['stateful']);self.assertLessEqual(goal['xy_goal_tolerance'],.05);self.assertEqual(read(edition('R07')/'tool.json')['tool']['docking_error_m'],.08)
    def test_complete_vehicle_self_mask(self):
        c=self.cfg['vehicle'];terrain=read(edition('R07')/'terrain.json');a,b,cmin,d=terrain['self_mask']
        self.assertLess(a,b);self.assertLess(cmin,0);self.assertGreater(d,0)
        for x,y in c['footprint']:self.assertTrue(a<x<b and cmin<y<d)
        self.assertGreater(terrain['self_z_max'],c['tool']['mount_xyz'][2]+c['tool']['collision_size'][2]/2)
    def test_local_costmap_cannot_substitute_prior_for_observation(self):
        nav=yaml.safe_load((edition('R07')/'nav2.yaml').read_text());local=nav['local_costmap']['local_costmap']['ros__parameters']
        self.assertFalse(local['track_unknown_space']);self.assertEqual(local['plugins'],['observed','inflation']);self.assertEqual(local['observed']['topic'],'/r07/risk_local')
    def test_ledger_version_and_restart_conservative(self):
        cfg=read(edition('R07')/'tool.json');cfg['session']='unit_r07';p=self.cfg['clusters'][0]['points'][0]
        goal=dict(operation=1,task_id='unit',idempotency_key='unit:point',map_id=cfg['map_id'],map_revision=cfg['map_revision'],frame_epoch=cfg['frame_epoch'],plan_revision=cfg['plan_revision'],tool_revision=cfg['tool_revision'],segment_id='',frame='map',target=[*p['target'],0.],area=[],requested_quantity=3.,timeout_sim_s=20.)
        with tempfile.TemporaryDirectory() as d:
            db=Path(d)/'ledger.sqlite';ledger=Ledger(db,cfg);bad=dict(goal,plan_revision='old_plan');self.assertFalse(ledger.reserve(bad,'attempt')['ok']);self.assertTrue(ledger.reserve(goal,'attempt')['ok']);ledger.close();ledger=Ledger(db,cfg)
            self.assertEqual(ledger.snapshot(goal['idempotency_key'])['state'],'RECOVERY_REQUIRED');self.assertFalse(ledger.reserve(goal,'retry')['ok']);self.assertEqual(ledger.snapshot(goal['idempotency_key'])['confirmed_quantity'],0);ledger.close()
    def test_online_nodes_have_no_truth_subscription(self):
        for name in ['mission','control','reference','sources','risk','tool_observer']:
            text=(PKG/'scripts'/(name+'_r07.py')).read_text()
            self.assertNotIn('/s03/evaluation',text);self.assertNotIn('/s10/evaluation',text);self.assertNotIn('world.sdf',text)
if __name__=='__main__':unittest.main()
