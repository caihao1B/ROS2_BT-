"""Persistent isolated CPU process. JSONL request/reply; caller owns deadlines."""
import argparse,json,sys,time
import yaml
from common15 import BASE
from engine15 import Engine

def main():
    p=argparse.ArgumentParser();p.add_argument('--method',required=True);a=p.parse_args()
    cfg=yaml.safe_load((BASE/'config/prediction.yaml').read_text())
    if cfg['dt_s']!=.4 or cfg['history_steps']!=8 or cfg['cpu_threads']!=1:
        raise ValueError('Fixed checkpoint requires 0.4 s grid, 8 histories, one CPU thread')
    engine=Engine(a.method,cfg['horizon_s'],cfg['max_objects'],cfg['imm'])
    print(json.dumps(dict(ready=True,method=a.method)),flush=True)
    for line in sys.stdin:
        request=json.loads(line)
        if request.get('stop'):break
        start=time.perf_counter();cpu=time.process_time()
        try:
            for frame in request['observations']:engine.observe(frame)
            result=engine.predict(request['observations'][-1])
            result.update(wall_ms=(time.perf_counter()-start)*1000,cpu_ms=(time.process_time()-cpu)*1000)
        except Exception as exc:
            result=dict(t=request['observations'][-1]['t'],frame='odom',method=a.method,
                        status='INFERENCE_ERROR',forecasts=[],error=repr(exc))
        print(json.dumps(dict(id=request['id'],result=result),allow_nan=False),flush=True)
if __name__=='__main__':main()
