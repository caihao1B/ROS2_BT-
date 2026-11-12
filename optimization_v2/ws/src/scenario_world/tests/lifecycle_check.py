"""Integration test of the public S02 launcher: mutual exclusion + owned stop."""
import json,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';OUT=BASE/'setup_logs/s02_lifecycle'
def main():
    OUT.mkdir(parents=True,exist_ok=True);ctl=['bash',str(BASE/'stepctl.sh')];sentinel=subprocess.Popen(['sleep','300'])
    stage=None;log=(OUT/'demo.log').open('w');result={}
    try:
        stage=subprocess.Popen(ctl+['demo','S02','--case','prior_vs_unknown','--seed','41','--sim-seconds','120','--wall-timeout','200'],stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+45;run=None
        while time.monotonic()<deadline:
            if stage.poll() is not None:raise RuntimeError('demo exited before stop test')
            session=BASE/'runtime/S02.json'
            if session.exists():
                run=BASE/'results/S02'/json.loads(session.read_text())['run_id']
                if (run/'demo/online/cloud_000.npz').exists():break
            time.sleep(.2)
        else:raise TimeoutError('demo did not produce observation')
        second=subprocess.run(ctl+['demo','S02','--case','world_overview','--headless','--sim-seconds','10','--wall-timeout','60'],capture_output=True,text=True,timeout=15)
        result['mutual_exclusion']=dict(exit_code=second.returncode,output=second.stdout+second.stderr)
        stopped=subprocess.run(ctl+['stop','S02'],capture_output=True,text=True,timeout=30)
        result['stop']=dict(exit_code=stopped.returncode,output=stopped.stdout+stopped.stderr)
        result['demo_exit_code']=stage.wait(timeout=10)
        result['unrelated_process_survived']=sentinel.poll() is None
        result['session_removed']=not (BASE/'runtime/S02.json').exists()
        result['run']=str(run.relative_to(ROOT));result['demo_status']=json.loads((run/'result.json').read_text())['status']
        rt=json.loads((run/'demo/runtime.json').read_text())
        result['children_gone']=all(not Path('/proc',str(p['pid'])).exists() for p in rt['commands'])
        result['passed']=second.returncode==2 and stopped.returncode==0 and result['unrelated_process_survived'] and result['session_removed'] and result['children_gone'] and result['demo_status']=='PARTIAL'
    finally:
        if stage and stage.poll() is None:
            subprocess.run(ctl+['stop','S02'],capture_output=True,timeout=30);stage.wait(timeout=10)
        sentinel.terminate();sentinel.wait(timeout=5);log.close()
        (OUT/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result.get('passed') else 1
if __name__=='__main__':raise SystemExit(main())
