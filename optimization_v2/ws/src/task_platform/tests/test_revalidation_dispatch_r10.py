"""Run the actual nested production recovery dispatchers with bounded IO doubles.

Native policy and geometry have separate tests. This covers Python transaction
ordering, the real event writer and refusal to commit a blocked observed path.
"""
import ast,json,math,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as N

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'

def pose_msg(p=(0.,0.,0.)):
    return N(header=N(frame_id='map'),pose=N(position=N(x=p[0],y=p[1],z=0.),orientation=N(x=0.,y=0.,z=math.sin(p[2]/2),w=math.cos(p[2]/2))))

class RecoveryDispatcher(unittest.TestCase):
    def exercise(self,kind,blocked=False):
        source=ast.parse((SCRIPTS/('mission_'+kind+'_r10.py')).read_text())
        main=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        functions=[n for n in main.body if isinstance(n,ast.FunctionDef) and n.name in ['log','recover_localization']]
        self.assertEqual(len(functions),2)
        calls=[];events=[];st=dict(t=10.,bt=dict(action='EXECUTE'),gate=dict(stopped=True,tool_stopped=True),loc=dict(pose=[0.,0.,0.],valid=True,planning_valid=True,native_supervisor=dict(plan_revalidation_required=True,recovery_ready=True)),safety={})
        def wait(predicate,*args):
            st['t']+=.1;st['safety']=dict(valid=True,localization_epoch=1,t=st['t'])
            if not predicate():raise AssertionError('Production wait condition not satisfied in fixture')
        def service(name,value):
            def call(_):calls.append(name);return value
            return N(service_is_ready=lambda:True,call_async=call)
        path=N(poses=[pose_msg(),pose_msg((.1,0.,0.))])
        handle=N(accepted=True,get_result_async=lambda:N(status=4,result=N(path=path)))
        prepared=dict(t=10.,epoch=1,reason='STOPPED_CANDIDATE_PREPARED_HOLD_RETAINED')
        with tempfile.TemporaryDirectory() as folder:
            ns=dict(json=json,math=math,st=st,out=Path(folder),events=events,log=None,revalidations={},cfg=dict(version='plan',map_revision='map',stopped_revalidation=dict(max_attempts_per_point=2,max_attempts_per_segment=2,observation_window_s=15.,maximum_path_m=12.)),active_tool=None,localization_hold=lambda:True,cancel_nav=lambda:calls.append('cancel_nav'),stop_all=lambda:calls.append('stop_all'),wait=wait,result=lambda answer,*_:answer,Trigger=N(Request=N),prepare=service('prepare',N(success=True,message=json.dumps(prepared))),commit=service('commit',N(success=True,message='accepted')),planner=N(server_is_ready=lambda:True,send_goal_async=lambda _:handle),pathcheck=service('pathcheck',N(is_valid=not blocked)),proofpub=N(publish=lambda _:calls.append('proof')),String=lambda **k:N(**k),ComputePathToPose=N(Goal=N),PoseStamped=pose_msg,pose=pose_msg,IsPathValid=N(Request=N),validate_path=lambda *_:(True,'VALID'),validate_candidate=lambda *_:(True,'VALID'),path_hash=lambda _: 'a'*64,gear_runs=lambda _:None)
            exec(compile(ast.Module(body=functions,type_ignores=[]),str(SCRIPTS/('mission_'+kind+'_r10.py')),'exec'),ns)
            goal=dict(id='original-job',target=[.1,0.,0.],yaw=0.,poses=[[0.,0.,0.],[.1,0.,0.]])
            if blocked:
                with self.assertRaisesRegex(RuntimeError,'REVALIDATION_OBSERVED_PATH_BLOCKED'):ns['recover_localization'](goal)
                self.assertNotIn('commit',calls)
            else:
                ns['recover_localization'](goal)
                self.assertLess(calls.index('prepare'),calls.index('pathcheck'))
                self.assertLess(calls.index('pathcheck'),calls.index('proof'))
                self.assertLess(calls.index('proof'),calls.index('commit'))
            logged=[json.loads(l) for l in (Path(folder)/'mission_events.jsonl').read_text().splitlines()]
            if kind=='spray':self.assertEqual(logged,events)
            if blocked:self.assertFalse(any(e['event']=='LOCALIZATION_REVALIDATED_RESUME' for e in logged))
            else:
                self.assertEqual(logged[-1]['event'],'LOCALIZATION_REVALIDATED_RESUME')
                preview=next(e for e in logged if e['event']=='LOCALIZATION_PREVIEW_PREPARED')
                self.assertEqual(preview['prepared_t'],10.)
                self.assertGreater(preview['t'],preview['prepared_t'])
                self.assertEqual(preview['epoch'],1)

    def test_spray_transaction_and_distinct_evidence_time(self):self.exercise('spray')
    def test_mowing_transaction_and_distinct_evidence_time(self):self.exercise('mowing')
    def test_spray_blocked_path_cannot_commit(self):self.exercise('spray',True)
    def test_mowing_blocked_path_cannot_commit(self):self.exercise('mowing',True)

if __name__=='__main__':unittest.main()
