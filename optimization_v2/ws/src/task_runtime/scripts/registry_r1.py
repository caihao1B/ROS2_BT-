"""Independent task identity + explicit immutable source map/plan bindings."""
from common_r1 import *
TASKS=('spray','inspection','mowing')
def load_task(task):
    if task not in TASKS:raise ValueError('UNKNOWN_TASK')
    path=APP/'config/tasks'/f'{task}.json';d=read(path)
    if d['task_id']!=task or canonical({k:v for k,v in d.items() if k!='version'})!=d['version']:
        raise ValueError('TASK_VERSION_MISMATCH')
    if d['results_root']!=str((APP/'results'/task).relative_to(ROOT)):raise ValueError('TASK_RESULT_SCOPE')
    expected={'spray':('diff','cluster','DWB'),'inspection':('diff','inspection','DWB'),'mowing':('ackermann','cluster','RPP')}[task]
    if (d['active_vehicle'],d['operation'],d['controller'])!=expected:raise ValueError('R01_BASELINE_ROLE_MISMATCH')
    for field,item in d['bindings'].items():
        p=(ROOT/item['path']).resolve()
        if not p.is_relative_to(ROOT) or not p.is_file() or sha(p)!=item['sha256']:
            raise ValueError('TASK_INPUT_CHANGED '+field)
    return d

def validate_task(task,cluster=None):
    d=load_task(task);cluster=cluster or d['default_cluster']
    if cluster not in d['clusters']:raise ValueError('CLUSTER_NOT_IN_TASK')
    add17();from binding_check17 import validate
    atlas,plan=validate()
    if atlas['map_id']!=d['map_id'] or atlas['revision']!=d['atlas_revision'] or plan['plan_revision']!=d['plan_revision']:
        raise ValueError('TASK_MAP_PLAN_EDITION_MISMATCH')
    return d,dict(id='',task_id=task,task_version=d['version'],cluster=cluster,
        vehicle=d['active_vehicle'],operation=d['operation'],controller=d['controller'],seed=d['seed'],
        scope='R01_STARTUP_SHUTDOWN_NOT_FORMAL',formal_credit=False)

def initialize():
    add17();from binding_check17 import validate
    from inputs17 import active
    atlas,plan=validate();inputs=active()
    for task in TASKS:
        path=APP/'config/tasks'/f'{task}.json'
        if path.exists():continue
        kind='ackermann' if task=='mowing' else 'diff';cluster='B1' if task=='mowing' else 'A1'
        sources=dict(world=BASE/'assets/worlds/irregular_farm.sdf',atlas=inputs['atlas'],plan=inputs['plan'],
            input_selector=BASE/'evaluation/active_inputs17.json',vehicle=BASE/f'config/vehicles/{kind}.yaml',
            control=BASE/'config/controller_profiles.yaml',nav2=BASE/f'config/nav2_{kind}.yaml',
            behavior_tree=BASE/'behavior_trees/mission.xml',tool=BASE/'config/tool_profiles.yaml',
            frozen_history=APP/'baseline/frozen_s01_s18.json',shutdown=APP/'config/shutdown.json')
        d=dict(schema='THREE-TASK-PROFILE/1',task_id=task,revision='R01.1',
            active_vehicle=kind,active_vehicle_scope='EXISTING_QUALIFIED_LIGHTWEIGHT_BASELINE',
            planned_reference_vehicle={'spray':'Husky A200 scale, R02 pending','inspection':'Jackal scale, R02 pending','mowing':'Existing Dino adaptation, R02 pending'}[task],
            operation='inspection' if task=='inspection' else 'cluster',controller='RPP' if task=='mowing' else 'DWB',
            default_cluster=cluster,clusters=[cluster] if task=='inspection' else [cluster[0]+str(i) for i in range(1,4)],
            seed=17023,map_id=atlas['map_id'],atlas_revision=atlas['revision'],plan_revision=plan['plan_revision'],
            results_root=str((APP/'results'/task).relative_to(ROOT)),
            defaults=dict(sim_seconds=300,wall_timeout=720,rtf=.6),
            bindings={k:dict(path=str(v.relative_to(ROOT)),sha256=sha(v)) for k,v in sources.items()},
            future_task_tree='R07/R08/R09 pending; currently shared original task tree',
            initial_admission='Original map, localization, tool and command ownership gates unchanged')
        d['version']=canonical(d);dump(path,d);(APP/'results'/task).mkdir(parents=True,exist_ok=True)
        dump(APP/'versions'/task/(d['version']+'.json'),d)
    return [load_task(task) for task in TASKS]
