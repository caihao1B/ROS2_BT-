"""Paired study dispatcher; missing experiments remain explicit requirements."""
from common17 import *
from checkpoint17 import Journal
from matrix17 import freeze
from statistics17 import paired,quantiles

def execute(out,binding,remaining,launch,resume,rtf):
    journal=Journal(EVAL/'checkpoints');matrix=freeze()[1];by={s['id']:s for s in matrix['ablations']}
    def commit(spec,directory,m):
        dump(directory/'metrics17.json',m);journal.commit(spec,binding,directory,m)
    old=journal.load(by['partition'],binding)
    if old and not resume:raise ValueError('ALREADY_EXECUTED: use --resume')
    if not old:
        dest=out/'partition';dest.mkdir()
        # Fresh execution, not a copied earlier summary. This runner writes
        # the same predeclared 54 domains/algorithms/seeds each time.
        args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/partition17.py'),'--out',str(dest)]
        code=launch(args,dest/'run.log',min(1500,remaining()-45))
        if code:raise ValueError('PARTITION_EXECUTION_FAILED')
        from partition_report17 import summarize
        m=summarize(dest);commit(by['partition'],dest,m)
    for kind,methods in [('diff',['DWB','RPP','MPPI']),('ackermann',['RPP','MPPI'])]:
        family='controller_'+kind
        if journal.load(by[family],binding):
            if not resume:raise ValueError('ALREADY_EXECUTED: use --resume')
            continue
        measurements=[];records=[]
        for seed in read(EVAL/'seeds.json')['paired']:
            for method in methods:
                spec=dict(id=f'{family}_{seed}_{method}',cluster='A1' if kind=='diff' else 'B1',vehicle=kind,controller=method,method=method,seed=seed,operation='inspection',benchmark='controller_prefix',scope='PAIRED_PREFIX')
                old=journal.load(spec,binding)
                if old:
                    if not resume:raise ValueError('ALREADY_EXECUTED: use --resume')
                    m=old['metrics'];directory=ROOT/old['case_path']
                else:
                    if remaining()<200/min(rtf,.6)+120:raise TimeoutError('CONTROLLER_BATCH_WALL_BUDGET')
                    directory=out/spec['id'];directory.mkdir();dump(directory/'spec.json',spec)
                    args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/one17.py'),'--spec',str(directory/'spec.json'),'--out',str(directory),'--seconds','200','--wall',str(min(650,remaining()-45)),'--rtf',str(rtf)]
                    code=launch(args,directory/'launch.log',min(670,remaining()-25));dump(directory/'command.json',dict(argv=args,exit_code=code))
                    from evaluate17 import evaluate
                    m=evaluate(directory,spec,BASE/'build_s17/monitor');m.update(scenario=family,seed=seed,method=method,task_completed=m['complete'],task_hash=sha(directory/'client/inspection_contract17.json'),limits_hash=canonical(dict(vehicle=sha(BASE/f'config/vehicles/{kind}.yaml'),controller_config=sha(BASE/f'config/nav2_{kind}.yaml'),seconds=200,rtf=rtf)),formal_credit=False,comparison_scope='same approved eight-metre connector prefix; tool off')
                    if source_binding()!=binding['sources'] or dependencies()!=binding['dependencies']:raise ValueError('INPUTS_CHANGED_DURING_ABLATION')
                    commit(spec,directory,m)
                measurements.append(m);records.append(dict(spec=spec,metrics=m,case_path=str(directory.relative_to(ROOT))))
                if not m['safety'] or not m['localization_pass']:raise ValueError('CONTROLLER_SAFETY_OR_LOCALIZATION_FAILED '+spec['id'])
        for seed in read(EVAL/'seeds.json')['paired']:
            group=[x for x in measurements if x['seed']==seed]
            if len(group)!=len(methods) or any(len({x[k] for x in group})!=1 for k in ['task_hash','limits_hash']):raise ValueError('CONTROLLER_PAIRING_FAILED')
        dest=out/(family+'_summary');dest.mkdir();m=dict(status='PASS',scope='paired experiment executed; PARTIAL robot tasks remain incomplete and cannot be efficiency wins',cases=records,paired_successful_task_time=[paired([r for r in measurements if r.get('task_completed') and r.get('localization_pass',True)],methods[0],method,'sim_seconds_including_stop') for method in methods[1:]],limitations=['Eight-metre inspection prefix, no full-cluster controller superiority claim.','Simulation and sensor seeds fixed; Nav2 internal optimizer randomness is not asserted bitwise deterministic.'])
        commit(by[family],dest,m)
    for family,methods in [('prediction',['no_prediction','imm','learned']),('memory',['full','event','cache'])]:
        if journal.load(by[family],binding):
            if not resume:raise ValueError('ALREADY_EXECUTED: use --resume')
            continue
        measurements=[];case_records=[]
        for scenario in ['turning','occlusion']:
            for seed in read(EVAL/'seeds.json')['paired']:
                for method in methods:
                    spec=dict(id=f'{family}_{scenario}_{seed}_{method}',family=family,scenario=scenario,seed=seed,method=method,scope='PAIRED_TWO_POINT_S14_MAP_NOT_FORMAL')
                    old=journal.load(spec,binding)
                    if old:
                        if not resume:raise ValueError('ALREADY_EXECUTED: use --resume')
                        m=old['metrics'];directory=ROOT/old['case_path']
                    else:
                        if remaining()<100/min(rtf,.6)+120:raise TimeoutError('ABLATION_BATCH_WALL_BUDGET; completed pairs remain resumable')
                        directory=out/spec['id'];directory.mkdir();dump(directory/'spec.json',spec)
                        args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/prediction_compare17.py'),'--out',str(directory),'--family',family,'--scenario',scenario,'--seed',str(seed),'--method',method,'--rtf',str(rtf),'--wall',str(min(550,remaining()-45))]
                        code=launch(args,directory/'launch.log',min(570,remaining()-25));dump(directory/'command.json',dict(argv=args,exit_code=code))
                        if not (directory/'metrics17.json').exists():raise ValueError('ABLATION_NO_MEASUREMENT '+spec['id'])
                        m=read(directory/'metrics17.json')
                        if source_binding()!=binding['sources'] or dependencies()!=binding['dependencies']:raise ValueError('INPUTS_CHANGED_DURING_ABLATION')
                        commit(spec,directory,m)
                    measurements.append(m);case_records.append(dict(spec=spec,metrics=m,case_path=str(directory.relative_to(ROOT))))
                    if m['status']!='PASS':raise ValueError('ABLATION_SAFETY_FAILED '+spec['id'])
        for scenario in ['turning','occlusion']:
            for seed in read(EVAL/'seeds.json')['paired']:
                group=[x for x in measurements if (x['scenario'],x['seed'])==(scenario,seed)]
                if len(group)!=3 or any(len({x[k] for x in group})!=1 for k in ['task_hash','fixture_hash','limits_hash']):raise ValueError('ABLATION_INPUT_PAIRING_FAILED')
        dest=out/(family+'_summary');dest.mkdir()
        groups={method:dict(executed=sum(x['method']==method for x in measurements),complete=sum(x['method']==method and x['task_completed'] for x in measurements),sim_s=quantiles([x['sim_seconds_including_stop'] for x in measurements if x['method']==method])) for method in methods}
        m=dict(status='PASS',scope='18 actual two-point trials; six same-input pairs, not six-cluster evidence',groups=groups,cases=case_records,paired_successful_task_time=[paired([r for r in measurements if r.get('task_completed') and r.get('localization_pass',True)],methods[0],method,'sim_seconds_including_stop') for method in methods[1:]],selection='Current production prediction and memory stay disabled; this narrow study alone does not authorize enabling them.',limitations=['Time for failed/partial tasks is reported, not interpreted as efficiency gain.','RTF and actual hardware recorded by this batch.','Frozen S14 benchmark map, not the new continuous atlas.'])
        commit(by[family],dest,m)
    for family,methods,seconds in [('localization',['wheel_imu','lidar','visual','gnss'],200),('coverage',['offline_repair','pure_online'],900)]:
        if journal.load(by[family],binding):
            if not resume:raise ValueError('ALREADY_EXECUTED: use --resume')
            continue
        records=[];measurements=[]
        for seed in read(EVAL/'seeds.json')['paired']:
            for method in methods:
                spec=dict(id=f'{family}_{seed}_{method}',family=family,scenario=family,method=method,seed=seed,scope='PAIRED_INPUT_ABLATION_NOT_FORMAL')
                previous=journal.load(spec,binding)
                if previous:
                    if not resume:raise ValueError('ALREADY_EXECUTED: use --resume')
                    m=previous['metrics'];directory=ROOT/previous['case_path']
                else:
                    if remaining()<seconds/min(rtf,.6)+120:raise TimeoutError('PAIRED_BATCH_WALL_BUDGET; checkpoints retained')
                    directory=out/spec['id'];directory.mkdir();dump(directory/'spec.json',spec)
                    wall=min(700 if family=='localization' else 2800,remaining()-45)
                    args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts'/(family+'_compare17.py')),'--out',str(directory),'--seed',str(seed),'--method',method,'--rtf',str(rtf),'--wall',str(wall)]
                    code=launch(args,directory/'launch.log',wall+20);dump(directory/'command.json',dict(argv=args,exit_code=code))
                    if not (directory/'metrics17.json').exists():raise ValueError('NO_PAIRED_MEASUREMENT '+spec['id'])
                    m=read(directory/'metrics17.json')
                    if source_binding()!=binding['sources'] or dependencies()!=binding['dependencies']:raise ValueError('INPUTS_CHANGED_DURING_ABLATION')
                    commit(spec,directory,m)
                records.append(dict(spec=spec,metrics=m,case_path=str(directory.relative_to(ROOT))));measurements.append(m)
                if not m['safety']:raise ValueError('PAIRED_INPUT_SAFETY_FAILED '+spec['id'])
                if family=='localization':
                    # An unavailable VO is an observed negative result, but a
                    # producer that never ran is not an input ablation.
                    if method in ['visual','gnss'] and m['visual_frames']<10:raise ValueError('STEREO_VO_DID_NOT_PRODUCE_OBSERVATIONS')
                    if method!='gnss' and m['gnss_dropout_frames']<10:raise ValueError('GNSS_REMOVAL_NOT_EXPOSED')
        for seed in read(EVAL/'seeds.json')['paired']:
            group=[m for m in measurements if m['seed']==seed]
            if len(group)!=len(methods) or any(len({m[k] for m in group})!=1 for k in ['task_hash','limits_hash']):raise ValueError('PAIRED_INPUT_MISMATCH')
        dest=out/(family+'_summary');dest.mkdir();complete=[m for m in measurements if m['task_completed'] and m['localization_pass']]
        m=dict(status='PASS',experiment_executed=True,cases=records,completion_by_method={method:sum(r['method']==method and r['task_completed'] for r in measurements) for method in methods},localization_pass_by_method={method:sum(r['method']==method and r['localization_pass'] for r in measurements) for method in methods},paired_successful_task_time=[paired(complete,methods[0],method,'sim_seconds_including_stop') for method in methods[1:]],limitations=['Failed and incomplete executions are retained; their shorter times do not earn efficiency credit.','Localization study removes/adds local increments under a common frozen lidar map and initialized global reference; not a lidar-free whole-vehicle claim.' if family=='localization' else 'Coverage study is online ordering of six known point-service obligations, with live Nav2 in both arms. It does not establish online mowing or unknown-map exploration.'])
        commit(by[family],dest,m)
    from extras17 import execute as extras
    extras(out,binding,remaining,launch,resume,rtf)
    # Never replace missing execution with an earlier-stage receipt.
    missing=[s['id'] for s in matrix['ablations'] if not journal.load(s,binding)]
    dump(out/'ablation_pending17.json',dict(missing=missing,formal_credit=False))
    if missing:raise ValueError('PAIRED_EXPERIMENTS_STILL_REQUIRED: '+','.join(missing))
