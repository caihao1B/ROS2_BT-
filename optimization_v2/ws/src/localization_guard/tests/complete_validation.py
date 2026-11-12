"""Targeted validation after measured reset/fixture defects; keeps superseded evidence.

Not a success override: rescoring and the same full nine-case aggregate determine PASS.
Normal no-reset cases retain their original default Q/P, identical to explicit values.
"""
import argparse,fcntl,os,signal,sys,time,traceback,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stage_runner import *

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--wall-timeout',type=float,default=650);p.add_argument('--resume',action='store_true');a=p.parse_args();out=a.run.resolve();data=read(out/'result.json');start=time.monotonic()
    if out.parent!=RESULTS:raise ValueError('S05 run required')
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if os.getpgrp()!=os.getpid():os.setsid()
    save_session(SESSION,str(Path(__file__).resolve()),out.name+'_targeted');signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    targets=[('diff','reset',32),('ackermann','slope',70),('diff','imu_loss',45),('ackermann','imu_loss',45)]
    extension=dict(command=sys.argv,reason='Fix RTAB private reset service and explicit default Q/P restoration; verify Ackermann on the same 5-degree ramp from a supported slope start; independently test IMU outage while physically moving.',targets=[k+'_'+c for k,c,_ in targets],status='PARTIAL')
    try:
        command(out,'targeted_build',['bash',str(BASE/'scripts/build_s05.sh')]);command(out,'targeted_units',[sys.executable,str(PKG/'tests/math_checks.py')]);command(out,'targeted_ekf_probe',[str(BASE/'ws/install/localization_guard/lib/localization_guard/ekf_probe'),str(out/'ekf_internal_trace.txt')])
        for kind,case,seconds in targets:
            name=kind+'_'+case
            if a.resume and (out/name/'runtime.json').exists() and read(out/name/'runtime.json')['success']:
                data['cases'][name]=analyze(out/name);print('RESCORE '+name+' '+str(data['cases'][name]['passed']),flush=True);continue
            if (out/name).exists():
                dest=out/'superseded'/name;dest.parent.mkdir(exist_ok=True)
                if dest.exists():raise ValueError('refuse overwrite of previous evidence')
                shutil.move(out/name,dest)
            remaining=a.wall_timeout-(time.monotonic()-start)
            if remaining<60:raise TimeoutError('targeted validation wall limit')
            print('TARGETED RUN '+name,flush=True);run_case(out/name,kind,case,seconds,min(remaining,180),41,False);m=analyze(out/name);data['cases'][name]=m;dump(out/'progress_targeted.json',data);print(('PASS ' if m['passed'] else 'FAIL ')+name+' '+str({k:v for k,v in m['checks'].items() if not v}),flush=True)
        # Recheck unchanged measured cases with the strengthened raw-covariance audit.
        for name in data['cases']:data['cases'][name]=analyze(out/name)
        data['checks'],passed=aggregate(data['cases']);data['status']='PASS' if passed else 'FAIL';extension['status']=data['status'];data.pop('error',None)
        data['next_step_verdict']='可以执行下一步 S06：S05 九案例全部通过且 S04 有效；未自动执行 S06' if passed else '暂不能执行下一步：仍有实测检查未通过'
    except BaseException as exc:
        data.update(status='PARTIAL' if isinstance(exc,KeyboardInterrupt) else 'FAIL',error=repr(exc));extension['error']=traceback.format_exc();print(traceback.format_exc(),flush=True)
    finally:
        try:
            extension['wall_seconds']=time.monotonic()-start;data['targeted_validation']=extension;dump(out/'result.json',data);render(out,data)
            command(out,'targeted_browser_check',['/usr/bin/python3',str(PKG/'tests/report_check.py'),str(out/'index.html')],45)
            data['evidence_compaction']=compact_evidence(out);dump(out/'result.json',data);receipt(out,data)
        finally:SESSION.unlink(missing_ok=True);lock.close()
    print(json.dumps(dict(status=data['status'],report=str(out/'index.html'),checks=data['checks']),ensure_ascii=False));return 0 if data['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
