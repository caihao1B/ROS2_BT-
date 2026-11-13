"""Independent monitor audit of every station attempt, including failed ones."""
from common17 import *
import subprocess

def main():
    directory=RESULTS/'qualification';out=directory/'attempt_audit';out.mkdir(exist_ok=True);audit=[]
    for group in ['survey','failed_stations']:
        for case in sorted((directory/group).iterdir()):
            if not (case/'runtime.json').exists():continue
            runtime=read(case/'runtime.json');clock=[r['t'] for r in rows(case/'evaluation/truth.jsonl')];contacts=read(case/'evaluation/contacts.json');key=group+'_'+case.name
            data=dict(clock=clock,contacts=contacts,tool_stopped=runtime.get('tool_shutdown',{}).get('state')=='STOPPED',limit_s=runtime['seconds']);dump(out/(key+'_input.json'),data)
            r=subprocess.run([str(BASE/'build_s17/monitor'),str(out/(key+'_input.json')),str(out/(key+'.json'))],capture_output=True,text=True,timeout=15)
            m=read(out/(key+'.json'));audit.append(dict(case=str(case.relative_to(ROOT)),group=group,monitor=m,exit_code=r.returncode,wall_seconds=runtime['wall_seconds']))
    failed=[r for r in audit if r['group']=='failed_stations'];normal=[r for r in audit if r['group']=='survey']
    passed=len(normal)==16 and all(r['monitor']['passed'] for r in normal) and any(bool(r['monitor']['obstacle_contacts']) for r in failed) and any(not r['monitor']['tool_stopped'] for r in failed)
    dump(out/'summary.json',dict(passed=passed,attempts=len(audit),qualified_station_attempts=len(normal),retained_failed_attempts=len(failed),audit=audit,scope='station captures only; zero formal-cluster credit'));print(json.dumps(dict(passed=passed,attempts=len(audit),qualified=len(normal),failed_retained=len(failed))))
    return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
