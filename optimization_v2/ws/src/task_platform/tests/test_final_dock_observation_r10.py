"""Exercise the production docking loop after both physical corrections are spent.

Only ROS IO is doubled. Real stationary admission and offset predicates operate
on an evolving pose stream; convergence is allowed only inside the original 8 s.
"""
import ast,math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as N
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts';sys.path.insert(0,str(SCRIPTS))
from docking_r10 import stable_dock
from local_dock_r10 import offset_observable

class FinalObservation(unittest.TestCase):
    def exercise(self,converges):
        source=ast.parse((SCRIPTS/'mission_spray_r10.py').read_text())
        main=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        fn=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='settle_and_redock')
        # A standalone namespace supplies the same shared active-handle binding.
        fn.body=[ast.copy_location(ast.Global(names=n.names),n) if isinstance(n,ast.Nonlocal) else n for n in fn.body]
        calls=[];events=[];observation_start=[None];final_started=[None]
        st=dict(t=10.,bt=dict(action='EXECUTE'),gate=dict(stopped=True,tool_stopped=True),loc=dict(valid=True,t=10.,pose=[.07,0.,0.]),phase='STOPPING',nav_idle=True)
        local_odom=dict(t=10.,pose=[0.,0.,0.])
        def tick(*_):
            st['t']+=.03
            if len(calls)==2:
                if observation_start[0] is None:observation_start[0]=st['t'];final_started[0]=st['t']
                elapsed=st['t']-observation_start[0]
                distance=.02+.06*math.exp(-elapsed/3.) if converges else .07
            else:distance=.07
            st['loc'].update(t=st['t'],pose=[distance,0.,0.]);local_odom['t']=st['t']
        def wait(predicate,seconds=20,*_):
            begin=st['t']
            while not predicate():
                tick()
                if st['t']-begin>seconds+.06:raise TimeoutError('fixture deadline')
        def pose():return N(header=N(frame_id=''),pose=N(position=N(x=0.,y=0.),orientation=N(z=0.,w=1.)))
        def send(_):
            calls.append(st['t']);return N(accepted=True,get_result_async=lambda:N(done=lambda:True,result=lambda:N(status=4)))
        ns=dict(st=st,active_nav=None,cfg=dict(docking_settle_s=1.5,docking_realign_max_attempts=2,local_docking=dict(maximum_translation_m=.15,maximum_sim_seconds=45.)),wait=wait,tick=tick,stable_dock=stable_dock,offset_observable=offset_observable,localization_hold=lambda:False,recover_localization=lambda _:None,log=lambda event,**kw:events.append(dict(t=st['t'],event=event,**kw)),math=math,local_odom=local_odom,propose=lambda *args:dict(correction_m=.07,map_path=[[.07,0.,0.],[0.,0.,0.]],odom_path=[[0.,0.,0.],[-.07,0.,0.]]),validate_path=lambda *_:(True,'VALID'),NavPath=lambda:N(header=N(frame_id=''),poses=[]),PoseStamped=pose,pathcheck=N(service_is_ready=lambda:True,call_async=lambda _:N(is_valid=True)),IsPathValid=N(Request=N),result=lambda value,*_:value,FollowPath=N(Goal=N),follow=N(server_is_ready=lambda:True,send_goal_async=send),path_hash=lambda _:'hash',cancel_nav=lambda:None)
        exec(compile(ast.Module(body=[fn],type_ignores=[]),str(SCRIPTS/'mission_spray_r10.py'),'exec'),ns)
        if converges:ns['settle_and_redock'](dict(id='same-point',target=[0.,0.]))
        else:
            with self.assertRaisesRegex(RuntimeError,'DOCKING_REALIGNMENT_EXHAUSTED'):ns['settle_and_redock'](dict(id='same-point',target=[0.,0.]))
        self.assertEqual(len(calls),2,'A longer observation must not create another physical attempt')
        last=next(e for e in reversed(events) if e['event']=='DOCKING_SETTLED_CHECK')
        self.assertEqual(last['attempt'],2);self.assertEqual(last['stable'],converges)
        if converges:
            self.assertLessEqual(last['stability']['median_distance_m'],.035)
            self.assertLessEqual(last['stability']['latest_distance_m'],.045)
            self.assertLess(st['t']-final_started[0],8.+1.5+.1)
        else:self.assertGreaterEqual(st['t']-final_started[0],8.)
    def test_final_observation_can_converge_without_extra_motion(self):self.exercise(True)
    def test_persistent_offset_still_fails_at_original_observation_bound(self):self.exercise(False)

if __name__=='__main__':unittest.main()
