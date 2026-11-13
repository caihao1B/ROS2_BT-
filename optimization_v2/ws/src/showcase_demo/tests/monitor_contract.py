import sys,unittest,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from motion_monitor import ProgressMonitor,connectivity,snapshot,current_goal
from startup_guard import pending_stale_context
from types import SimpleNamespace as NS
from tempfile import TemporaryDirectory
import json
class Contracts(unittest.TestCase):
    def test_segmented_work_transit_tracks_the_active_action_goal(self):
        j=dict(id='return',kind='TRANSIT',poses=[[8,9,0]],reference_parts=[dict(id='return_reference_0',poses=[[2,3,0]]),dict(id='return_reference_1',poses=[[5,6,0]])])
        m=NS(st={'gate':{'action_id':'return_reference_1_resume_1'}})
        self.assertEqual(current_goal(m,j),[5,6])
        m.st['gate']['action_id']='unrelated'
        self.assertEqual(current_goal(m,j),[8,9])
        j['kind']='INSPECTION';m.inspection_completed_parts=['return_reference_0'];m.inspection_executed_parts={'return_reference_1':[[5,7,0]]}
        self.assertEqual(current_goal(m,j),[5,7])
    def test_discovery_hold_never_relaxes_runtime_or_startup_deadline(self):
        self.assertTrue(pending_stale_context(False,'INPUT_STALE',3.8,90))
        for initialized,reason,t in [(True,'INPUT_STALE',3.8),(False,'SAFETY_STOP',3.8),(False,'INPUT_STALE',90)]:self.assertFalse(pending_stale_context(initialized,reason,t,90))
    def test_missing_or_stale_reference_never_proves_blockage(self):
        for fresh,valid in [(False,True),(True,False)]:self.assertFalse(connectivity(np.full((20,20),100),.1,[0,0],[.3,.3],[1,1],fresh,valid)['proves_map_blocked'])
    def test_actual_disconnected_grid(self):
        a=np.zeros((20,20));a[:,10]=100;self.assertTrue(connectivity(a,.1,[0,0],[.3,.3],[1.5,1.5],True,True)['proves_map_blocked'])
    def test_open_grid_does_not_prove_kinematic_path(self):
        d=connectivity(np.zeros((20,20)),.1,[0,0],[.3,.3],[1.5,1.5],True,True);self.assertEqual(d['code'],'POINT_CONNECTED_KINEMATICS_UNRESOLVED');self.assertFalse(d['proves_map_blocked'])
    def test_goal_outside_is_not_a_blocked_path(self):self.assertFalse(connectivity(np.zeros((20,20)),.1,[0,0],[.3,.3],[5,5],True,True)['proves_map_blocked'])
    def test_stationary_navigation_escalates_but_work_does_not(self):
        p=ProgressMonitor()
        for t in range(61):d=p.update(t,[0,0],'a',True,False)
        self.assertTrue(d['manual']);self.assertFalse(p.update(61,[0,0],'a',True,True)['manual'])
    def test_motion_and_new_obligation_reset_stall(self):
        p=ProgressMonitor()
        for t in range(100):self.assertFalse(p.update(t,[t*.12,0],'a',True,False)['manual'])
        self.assertEqual(p.update(101,[11.88,0],'b',True,False)['stalled_s'],0)
    def fixture(self,out,vehicle='ackermann',working=False):
        pose=NS(pose=NS(pose=NS(orientation=NS(x=0.,y=0.,z=0.,w=1.))))
        return NS(a=NS(out=Path(out),vehicle=vehicle),st=dict(t=0.,pose=pose,tool=dict(t=0.,stopped_confirmed=True,active=False),gate=dict(t=0.,measured_standstill=True)),api={'xy':lambda:np.zeros(2)},pending=[dict(id='job',kind='COVERAGE',poses=[[1,1,0]])],initialized=True,complete=False,stopping=False,working=working,busy=True,platform_ok=True,blocked=False,unsafe=False,error=None,reason='RUN',directive={},done=[],replans=0,deferrals=0)
    def test_legitimate_stationary_heading_alignment_is_progress(self):
        p=ProgressMonitor()
        for t in range(100):self.assertFalse(p.update(t,[0,0],'dock',True,False,t*.03)['manual'])
    def test_mowing_stall_escalates_through_owner_and_latches(self):
        with TemporaryDirectory() as out:
            m=self.fixture(out,working=True)
            for t in range(61):m.st['t']=t;snapshot(m)
            self.assertTrue(m.unsafe);self.assertTrue(json.loads((Path(out)/'manual_handoff.json').read_text())['requires_manual'])
            self.assertFalse(json.loads((Path(out)/'manual_handoff.json').read_text())['stop_confirmed'])
            m.st['t']=61;m.api['xy']=lambda:np.array([2,0]);snapshot(m)
            self.assertTrue(json.loads((Path(out)/'motion_monitor.json').read_text())['requires_manual'])
    def test_final_handoff_not_overwritten_by_shutdown_tick(self):
        with TemporaryDirectory() as out:
            m=self.fixture(out);snapshot(m,terminal=True);before=(Path(out)/'motion_monitor.json').read_text();m.st['t']=10;snapshot(m)
            self.assertEqual(before,(Path(out)/'motion_monitor.json').read_text());self.assertTrue(json.loads(before)['stop_confirmed'])
    def test_spray_dwell_is_exempt(self):
        with TemporaryDirectory() as out:
            m=self.fixture(out,'diff',True);m.pending[0]['kind']='POINT_SERVICE'
            for t in range(61):m.st['t']=t;snapshot(m)
            self.assertFalse(m.unsafe)
if __name__=='__main__':unittest.main()
