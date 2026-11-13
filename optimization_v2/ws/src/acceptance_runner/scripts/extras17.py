"""Serial connector, fault and restart evidence for the fixed S17 matrix."""
from common17 import *
from checkpoint17 import Journal
from matrix17 import freeze
from evaluate17 import evaluate

def execute(out,binding,remaining,launch,resume,rtf):
    journal=Journal(EVAL/'checkpoints');matrix=freeze()[1]
    def commit(spec,directory,m):
        if source_binding()!=binding['sources'] or dependencies()!=binding['dependencies']:raise ValueError('EXTRA_INPUTS_CHANGED')
        dump(directory/'metrics17.json',m);journal.commit(spec,binding,directory,m)
    def previous(spec):
        old=journal.load(spec,binding)
        if old and not resume:raise ValueError('ALREADY_EXECUTED: use --resume')
        if old:
            from stage17 import require_completed_checkpoint
            require_completed_checkpoint(old)
        return old
    for spec in matrix['connector']:
        if previous(spec):continue
        if remaining()<1800/min(rtf,.6)+120:raise TimeoutError('CONNECTOR_BATCH_WALL_BUDGET')
        directory=out/spec['id'];directory.mkdir();kind=spec['vehicle'];run_spec=dict(spec,seed=42001,cluster='A1' if kind=='diff' else 'B1',controller='DWB' if kind=='diff' else 'RPP',operation='inspection',scope='FULL_CONNECTOR')
        dump(directory/'spec.json',run_spec);wall=min(4900,remaining()-45)
        args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/one17.py'),'--spec',str(directory/'spec.json'),'--out',str(directory),'--seconds','1800','--wall',str(wall),'--rtf',str(rtf)]
        code=launch(args,directory/'launch.log',wall+20);dump(directory/'command.json',dict(argv=args,exit_code=code));m=evaluate(directory,run_spec,BASE/'build_s17/monitor');commit(spec,directory,m)
        if m['status']!='PASS':raise ValueError('FULL_CONNECTOR_FAILED '+spec['id'])
    for spec in matrix['faults']:
        if spec['fault']=='mission_restart':continue
        if previous(spec):continue
        if remaining()<200/min(rtf,.6)+120:raise TimeoutError('FAULT_BATCH_WALL_BUDGET')
        directory=out/spec['id'];directory.mkdir();dump(directory/'spec.json',spec);wall=min(750,remaining()-45)
        args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/faults17.py'),'--out',str(directory),'--seed',str(spec['seed']),'--fault',spec['fault'],'--wall',str(wall),'--rtf',str(rtf)]
        code=launch(args,directory/'launch.log',wall+20);dump(directory/'command.json',dict(argv=args,exit_code=code))
        if not (directory/'metrics17.json').exists():raise ValueError('FAULT_NO_MEASUREMENT '+spec['id'])
        m=read(directory/'metrics17.json');commit(spec,directory,m)
        if m['status']!='PASS':raise ValueError('FAULT_ACCEPTANCE_FAILED '+spec['id'])
    spec=next(s for s in matrix['faults'] if s['fault']=='mission_restart')
    old=previous(spec)
    if not old:
        directory=out/spec['id'];directory.mkdir();cases=[];prior=None
        for phase in [1,2]:
            if remaining()<120/min(rtf,.6)+120:raise TimeoutError('RESTART_BATCH_WALL_BUDGET')
            dest=directory/str(phase);dest.mkdir();task=dict(id='restart_'+str(phase),cluster='A2',vehicle='diff',controller='DWB',seed=spec['seed'],scope='FAULT_RESPONSE_NOT_FORMAL');dump(dest/'spec.json',task)
            wall=min(650,remaining()-45);args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/one17.py'),'--spec',str(dest/'spec.json'),'--out',str(dest),'--seconds','120','--wall',str(wall),'--rtf',str(rtf)]
            if prior:args+=['--resume-from',str(prior)]
            code=launch(args,dest/'launch.log',wall+20);dump(dest/'command.json',dict(argv=args,exit_code=code));m=evaluate(dest,task,BASE/'build_s17/monitor');cases.append(m);prior=dest
            if not m['safety'] or not m['original_denominator']:raise ValueError('RESTART_PHASE_UNSAFE')
        before=read(directory/'1/client/mission.json');after=read(directory/'2/client/mission.json');restored=read(directory/'2/client/restored17.json');keys={r['key'] for r in before['tool_results'] if r['state']=='SUCCEEDED' and not r.get('uncertain',True)}
        goals=[r for r in rows(directory/'2/client/events.jsonl') if r['event']=='tool_goal'];duplicates=[r for r in goals if r.get('key') in keys]
        passed=before['confirmed_points']>0 and restored['confirmed_points']==before['confirmed_points'] and after['confirmed_points']>before['confirmed_points'] and bool(goals) and not duplicates
        m=dict(status='PASS' if passed else 'FAIL',before=before['confirmed_points'],restored=restored['confirmed_points'],after=after['confirmed_points'],duplicate_goals=duplicates,cases=cases,scope='actual two-process reboot; same original 22-point denominator, not a full-cluster result');commit(spec,directory,m)
        if not passed:raise ValueError('RESTART_NOT_DEMONSTRATED')
    # The operational restart and journal integrity are distinct checks.
    contract=dict(id='resume_contract',scope='versioned atomic journal plus actual operational restart')
    if not previous(contract):
        dest=out/'resume_contract';dest.mkdir();args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'tests/contracts17.py')];code=launch(args,dest/'run.log',min(30,remaining()-20))
        restart=journal.load(spec,binding);m=dict(status='PASS' if code==0 and restart and restart['metrics']['status']=='PASS' else 'FAIL',contract_test_exit_code=code,restart_checkpoint=restart['identity'] if restart else None)
        commit(contract,dest,m)
