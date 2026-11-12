"""Observation-only predictors. Simulator identity and labels are not inputs."""
import time
import numpy as np
from history15 import History
from imm15 import IMM


class Engine:
    def __init__(self, method, horizon=3.2, max_objects=6, imm_parameters=None):
        if method not in ['no_prediction','imm','learned']:
            raise ValueError('Unknown predictor')
        self.method,self.horizon,self.max_objects=method,horizon,max_objects
        if not 2<=horizon<=4 or abs(round(horizon/.4)*.4-horizon)>1e-8 or max_objects<1:
            raise ValueError('Invalid prediction horizon or object limit')
        self.imm_parameters=imm_parameters or {}
        self.history=History();self.filters={};self.learned=None
        if method=='learned':
            from trajectron15 import Trajectron
            self.learned=Trajectron()

    def observe(self,frame):
        accepted=self.history.add(frame)
        if self.method=='no_prediction':return
        for r in accepted:
            identity=r['id'];old=self.filters.get(identity)
            if old is None or r['t']-old.t>1.2:
                self.filters[identity]=IMM(r['t'],r['position'],r['covariance'],r['velocity'],**self.imm_parameters)
            else:old.update(r['t'],r['position'],r['covariance'])
        for identity in list(self.filters):
            if identity not in self.history.tracks:del self.filters[identity]

    def predict(self,frame):
        start=time.perf_counter();cpu=time.process_time()
        features=self.history.features(frame['t']);output=[]
        for f in features:
            r=self.history.tracks[f['id']][-1]
            if frame['t']-f['t']>.6:continue
            # Static geometry remains the existing costmap's responsibility.
            if f.get('classification')!='person' and np.linalg.norm(r['velocity'])<.05:continue
            if self.method=='no_prediction':continue
            if len(output)>=self.max_objects:break
            if self.method=='learned' and f.get('classification')=='person':
                p=self.learned.predict(f,features,self.horizon)
                p['candidate_status']='LEARNED_PERSON'
            else:
                p=self.filters[f['id']].predict(self.horizon)
                p['candidate_status']='IMM_BASELINE' if self.method=='imm' else 'IMM_NONPERSON_FALLBACK'
            p.update(id=f['id'],size=f['size'],classification=f['classification'],observed_age_s=frame['t']-f['t'])
            output.append(p)
        return dict(t=frame['t'],frame='odom',method=self.method,forecasts=output,
                    status='DISABLED_BY_EXPERIMENT' if self.method=='no_prediction' else 'OK',
                    wall_ms=(time.perf_counter()-start)*1000,cpu_ms=(time.process_time()-cpu)*1000,
                    confirmed_histories=len(features),future_labels_used=False)
