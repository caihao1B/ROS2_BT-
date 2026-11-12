"""Narrow S14 orchestration extension; original qualified source is unchanged."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'motion_prediction/scripts'))
from common15 import add14,S14,BASE
once=add14()
source=(S14/'scripts/mission14.py').read_text()
source=once(source,'        self.platform_ok=False','''        self.platform_ok=False
        self.prediction_recovery=AdmissionRecovery(policy['clear_hold_s'],policy['max_block_s'])
        self.prediction={};self.prediction_wall=-1.;self.prediction_hold=False
        node.create_subscription(String,'/s15/risk',self.receive_prediction,5)''')
source=once(source,'    def make_jobs(self):','''    def make_jobs(self):
        if self.a.case.startswith('collect_'):return []
        benchmark=read(self.a.out.parent/'scene/benchmark_task.json')
        x,y,h=benchmark['start_map']
        return [dict(id=f's15_point_{i}',kind='POINT_SERVICE',poses=[[x+d*math.cos(h),y+d*math.sin(h),h]],target=[x+(d+.42)*math.cos(h),y+(d+.42)*math.sin(h)],task_id=f's15_point_{i}',planned_s=10.,steering_events=[]) for i,d in enumerate(benchmark['distances'])]

    def unused_original_make_jobs(self):''')
source=once(source,'    def receive(self,m):','''    def receive_prediction(self,m):
        self.prediction=json.loads(m.data);self.prediction_wall=time.monotonic()

    def prediction_context(self):
        t=self.st['t'];p=self.prediction
        fresh=-.05<=t-p.get('t',-100.)<.2 and time.monotonic()-self.prediction_wall<.35
        self.prediction_hold=bool(fresh and p.get('hold',False))
        return dict(prediction_hold=self.prediction_hold,prediction_recovery=self.prediction_recovery.since is not None,prediction_exhausted=self.prediction_recovery.exhausted,prediction_resumes=self.prediction_recovery.resumes,prediction_source=p.get('method','missing') if fresh else 'BASELINE_FALLBACK',prediction_reason=p.get('reason','NOT_READY') if fresh else 'RISK_CHANNEL_STALE_FALLBACK',prediction_envelope_mass=p.get('mode_envelope_mass',0.) if fresh else None)

    def receive(self,m):''')
source=once(source,'        self.platform_ok=bool(platform_ok)','''        self.platform_ok=bool(platform_ok)
        self.prediction_context()
        self.prediction_recovery.max_resumes=max(0,12-self.admission_recovery.resumes)
        self.admission_recovery.max_resumes=max(0,12-self.prediction_recovery.resumes)
        if self.initialized and self.prediction_hold and not self.stopping and self.pending:
            self.prediction_recovery.begin(t)
        if self.prediction_recovery.since is not None:
            qualified=bool(not self.prediction_hold and platform_ok and tool.get('stopped_confirmed') and not tool.get('active') and self.st.get('pose') is not None and self.st.get('odom') is not None and self.api['stopped']() and self.api['costmaps_ready']())
            recovery=self.prediction_recovery.update(t,qualified)
            if recovery=='RESTORED':
                self.log('prediction_admission_restored',resumes=self.prediction_recovery.resumes,original_obligation=self.pending[0]['id'] if self.pending else None,no_work_credit=True)
''')
source=once(source,'dict(admission_recovery=self.admission_recovery.since is not None','dict(**self.prediction_context(),admission_recovery=self.admission_recovery.since is not None')
source=once(source,'    def run(self):\n        self.save()','''    def run(self):
        if self.a.case.startswith('collect_'):
            self.scope='OBSERVATION_DATA_COLLECTION_NO_WORK_CLAIM'
            self.save()
            while self.st['t']<self.a.seconds-10:
                self.spin()
            self.complete=True;self.reason='OBSERVATIONS_COLLECTED';self.halt();self.save();return True
        self.save()''')
source=once(source,"                interrupted_t=self.st['t']", "                recover_prediction=isinstance(exc,Suspended) and str(exc)=='PREDICTED_CONFLICT_WAIT'\n                interrupted_t=self.st['t']")
source=once(source,"failure_class='OBSERVATION_ADMISSION' if recover_observations else 'ACTION_OR_OBSTRUCTION'", "failure_class='PREDICTED_CONFLICT' if recover_prediction else 'OBSERVATION_ADMISSION' if recover_observations else 'ACTION_OR_OBSTRUCTION'")
source=once(source,'                if recover_observations and not self.unsafe:', '''                if recover_prediction and not self.unsafe:
                    self.blocked=False;self.prediction_recovery.begin(interrupted_t)
                    self.log('prediction_wait',obligation=j['id'],max_wait_s=45.,original_endpoint=j['poses'][-1],no_work_credit=True)
                if recover_observations and not self.unsafe:''')
exec(compile(source,str(Path(__file__).resolve()),'exec'),globals())
