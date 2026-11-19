"""Run a source-bound continuous A1 selection under the original 1800 s task budget."""
import argparse
import importlib.util
from pathlib import Path

from common import APP, PKG, ROOT, canonical, edition, read, sha
import spray_precision_candidate_r10 as precision
import terrain_pairing_r10 as pairing
from spray_defer_fixture_r10 import once


REACQUISITION = '''    def global_reacquire(p):
        nonlocal active_nav
        goal=NavigateToPose.Goal()
        goal.pose.header.frame_id='map'
        goal.pose.pose.position.x,goal.pose.pose.position.y=p['target']
        goal.pose.pose.orientation.z=math.sin(p['yaw']/2)
        goal.pose.pose.orientation.w=math.cos(p['yaw']/2)
        wait(lambda:bool(st['loc'] and st['loc']['valid'] and st['bt'] and st['bt']['action']=='EXECUTE'),8)
        wait(nav.server_is_ready,5)
        st.update(phase='NAVIGATING',nav_idle=False,precision_translation_m=None)
        active_nav=result(nav.send_goal_async(goal),5)
        if not active_nav.accepted:
            active_nav=None
            st.update(phase='STOPPING',nav_idle=True)
            raise RuntimeError('LOCAL_DOCK_NAV_REJECTED')
        future=active_nav.get_result_async()
        began=st['t']
        log('GLOBAL_PRECISION_REACQUIRE_GOAL',point=p['id'],work_admitted=False)
        while not future.done():
            tick()
            action=(st['bt'] or {}).get('action')
            if action in ['CANCEL','STOP','HUMAN','WAIT','REPLAN','DEFER'] or st['t']-began>cfg['local_docking']['maximum_sim_seconds']:
                cancel_nav()
                if action in ['STOP','HUMAN']:
                    raise RuntimeError('LOCAL_DOCK_SAFETY_HOLD')
                log('GLOBAL_PRECISION_REACQUIRE_INTERRUPTED',point=p['id'],action=action)
                return
        status=future.result().status
        active_nav=None
        st.update(phase='STOPPING',nav_idle=True)
        log('GLOBAL_PRECISION_REACQUIRE_TERMINAL',point=p['id'],nav_status=status,work_admitted=False)
        wait(lambda:bool(st['gate'] and st['gate']['stopped'] and st['gate']['tool_stopped']),6)
'''

def worker():
    parent=precision.build()
    src=parent.read_text()
    inner=REACQUISITION.rstrip()
    src=once(src,'    def settle_and_redock(p,force=False):',inner+'\n    def settle_and_redock(p,force=False):')
    src=once(src,'attempt=0;observations=0','attempt=0;observations=0;coarse_retry_used=False')
    needle="            # A scatter-only failure calls for more stopped observations, not motion."
    src=once(src,needle,"""            if (not coarse_retry_used and offset_observable(metrics)
                    and cfg['local_docking']['maximum_translation_m'] < metrics['median_distance_m'] <= cfg['navigation_recovery']['near_radius_m']):
                coarse_retry_used=True;attempt+=1
                global_reacquire(p)
                continue
"""+needle)
    out=parent.parent.parent/'spray_precision_session'/'mission_spray_precision_session.py'
    out.parent.mkdir(parents=True,exist_ok=True)
    compile(src,str(out),'exec')
    out.write_text(src)
    return out


def terrain_version():
    # Keep the already recorded v1 source and executable reproducible.
    module_file=Path(pairing.__file__).resolve()
    spec=importlib.util.spec_from_file_location('terrain_pairing_session_build',module_file)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.OUT=APP/'build/spray_terrain_pairing_session_source'
    module.OUT.mkdir(parents=True,exist_ok=True)
    module.BINARY=APP/'build/spray_terrain_pairing_session/r10_terrain_pairing'
    module.INCLUDE=module.OUT/'terrain_pairing_session.inc'
    src=once(pairing.INCLUDE.read_text(), 'double pair_wait_ = 0.;','double pair_wait_ = 0.;\nbool pending_deferred_ = false;')
    src=once(src,'if (match || now_s < wait_start_ ||','''if (now_s < wait_start_) {
    pending_cloud_.reset();
    epoch_bad_ = true;
    fail(stamp, "NONMONOTONIC_SENSOR_TIME");
    return;
  }
  if (match ||''')
    src=once(src,'    process_pending();\n  }\n}', '    process_pending();\n  } else {\n    pending_deferred_ = true;\n  }\n}')
    src=once(src,'  pending_cloud_ = std::move(message);','  pending_deferred_ = false;\n  pending_cloud_ = std::move(message);')
    module.write_changed(module.INCLUDE,src)
    inject=module.write_changed
    def write(path,data):
        if path.name=='terrain.cpp':
            data=once(data,'{"pair_wait_sim_s",pair_wait_}','{"pair_wait_sim_s",pair_wait_},{"paired_after_defer",pending_deferred_}')
        inject(path,data)
    module.write_changed=write
    return module.build(),module


def run(first,count,seconds,rtf):
    fixture_path=PKG/'scripts/spray_defer_fixture_r10.py'
    loader=importlib.util.spec_from_file_location('spray_precision_session_fixture',fixture_path)
    fixture=importlib.util.module_from_spec(loader)
    loader.loader.exec_module(fixture)
    base=read(fixture.BASE_MISSION)
    full=next(c['points'] for c in base['clusters'] if c['id']=='A1')
    if not 0<=first<len(full) or not 2<=count<=len(full)-first:
        raise ValueError('Expected an original consecutive A1 selection of two or more points')
    if not 60<=seconds<=base['task_budget_s'] or not 0<rtf<=1.:
        raise ValueError('Stay within the original A1 task budget / RTF limits')
    chosen=full[first:first+count]
    fixture.FOUR=[p['id'] for p in chosen]
    fixture.original_points=lambda:(read(fixture.BASE_MISSION),chosen)
    binary,terrain=terrain_version()
    mission_worker=worker()
    scripts=PKG/'scripts'
    sources=[*fixture.FIXTURE_SOURCES,mission_worker,fixture.RECORDER,terrain.INCLUDE,terrain.OUT/'terrain.cpp',binary,
             pairing.SOURCE,pairing.CMAKE,pairing.INCLUDE,*(scripts/k for k in ('spray_handoff_candidate_r10.py',
             'mission_spray_r10.py','spray_precision_r10.py','spray_precision_candidate_r10.py','terrain_pairing_r10.py','spray_precision_session_r10.py'))]
    bound={str(p.relative_to(ROOT)):sha(p) for p in sources}
    def configuration():
        cfg=read(fixture.BASE_MISSION)
        parent=cfg.pop('version')
        cfg['precision_session']=dict(parent_version=parent,selected_ids=fixture.FOUR,first_index=first,
                                      original_obligations=56,formal_credit=False,bound_implementation=bound)
        cfg['version']=canonical(cfg)
        return cfg
    def scene(out,task,session,**kwargs):
        return fixture.fixture_scene(out,task,session,False,configuration()['version'],**kwargs)
    target=APP/'results/experiments/spray_precision_session'
    scope=dict(__file__=str(fixture_path),__name__='spray_precision_session_runtime',compose=scene,candidate_config=configuration,
               EXPERIMENTS=target,latest_name='latest_natural.json',selected_ids=fixture.FOUR,worker=mission_worker,RECORDER=fixture.RECORDER,FIXTURE_SOURCES=sources)
    template=once(fixture.runtime_source(),"launch('terrain',[str(APP/'build/r08/r08_terrain')",
                  "launch('terrain',[str(APP/'"+str(binary.relative_to(APP))+"')")
    exec(compile(template,str(fixture.RECORDER),'exec'),scope)
    return scope['run'](task='spray',seconds=seconds,rtf=rtf,cluster='A1',point_limit=count,case='precision_natural')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode',choices=['prepare','run'])
    ap.add_argument('--first-index',type=int,default=21)
    ap.add_argument('--count',type=int,default=4)
    ap.add_argument('--seconds',type=int,default=300)
    ap.add_argument('--rtf',type=float,default=.55)
    a=ap.parse_args()
    print((worker(),terrain_version()[0]) if a.mode=='prepare' else run(a.first_index,a.count,a.seconds,a.rtf))


if __name__=='__main__':
    main()
