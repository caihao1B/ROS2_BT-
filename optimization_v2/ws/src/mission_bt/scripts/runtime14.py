"""Version-checked S13 launch reuse. S14 owns new task/gate processes only."""
from common14 import *
import types
import signal
from owned14 import cleanup

def runtime(probe=False, survey=False, registered=False):
    source = (NAV / 'scripts/runtime13.py').read_text()
    # Headless sensor rendering should not depend on the desktop X/GLX path.
    # EGL is a Gazebo/OGRE2 application option; sensor rates and safety ages stay fixed.
    source=replace_once(source,"['gz','sim','--force-version','8','-s','--seed',str(seed),str(out/'scene/world.sdf')]", "['gz','sim','--force-version','8','-s']+(['--headless-rendering'] if not gui else [])+['--seed',str(seed),str(out/'scene/world.sdf')]")
    source=replace_once(source,"        server=launch('server'", "        dump(out/'render_backend.json',dict(backend='OGRE2_EGL' if not gui else 'OGRE2_DESKTOP',headless=not gui,sensor_config_unchanged=True))\n        server=launch('server'")
    source = replace_once(source, 'from scene13 import compose', 'from scene14 import compose')
    source=replace_once(source,"common10.LEDGER/'scripts/ledger_node.py'", "S14PKG/'scripts/ledger14.py'")
    source=replace_once(source,"        slam=start_slam('slam');", "        launch('sensor_diagnostic',['/usr/bin/python3',str(S14PKG/'scripts/capture14.py'),'--out',str(out/'sensor_diagnostic')])\n        slam=start_slam('slam');")
    source=replace_once(source,"        if aux:auxiliary.add(p)","        dump(out/'processes.json',dict(partition=env['GZ_PARTITION'],created_wall=time.time()-(time.monotonic()-start),commands=commands))\n        if aux:auxiliary.add(p)")
    source=replace_once(source,'source_restore_t=t+5.;','source_restore_t=t+(90. if case==\'persistent_block\' else 5.);')
    source=replace_once(source,"dict(X=4.,Y=0.,Z=.5)","dict(X=float(hint[0]),Y=float(hint[1]),Z=.5)")
    source=replace_once(source,"        time.sleep(2.2)\n        if 'slam'", "        time.sleep(.5)\n        if 'server' in locals() and server.poll() is None:\n            paused=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'])\n            result['shutdown_clock_paused']=paused.returncode==0 and 'true' in paused.stdout\n        if 'slam'")
    if not probe:
        source=replace_once(source,"str(BASE/'ws/install/navigation_adapter/lib/navigation_adapter/command_gate')", "str(BASE/'ws/install/mission_bt/lib/mission_bt/command_gate')")
        source=replace_once(source,"str(common13.PKG/'scripts/client13.py')", "str(S14PKG/'scripts/worker14.py')")
        source=replace_once(source,"        controller=controller or ('DWB' if kind=='diff' else 'RPP')", "        dump(out/'decision_policy.json',yaml.safe_load((BASE/'config/decision_policy.yaml').read_text()))\n        launch('mission_bt',[str(BASE/'ws/install/mission_bt/lib/mission_bt/mission_bt'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'decision_policy.json'),'-p','tree_file:='+str(BASE/'behavior_trees/mission.xml'),'-p','log_file:='+str(out/'decisions.jsonl')])\n        controller=controller or ('DWB' if kind=='diff' else 'RPP')")
    if survey:
        if not probe:raise ValueError('Survey is stationary, not an active mission')
        source=replace_once(source,"str(common13.PKG/'scripts/client13.py')", "str(S14PKG/'scripts/survey_client14.py')")
        source=replace_once(source,"str(S14PKG/'scripts/capture14.py')", "str(S14PKG/'scripts/survey_capture14.py')")
    if registered:
        from binding14 import region
        atlas,reg=region()
        source=replace_once(source,"terrain=read(out/'terrain.json');footprint=", "from collision_volume14 import envelope\n        volume=envelope();dump(out/'collision_volume.json',volume)\n        terrain=read(out/'terrain.json');terrain['collision_ceiling_m']=volume['ceiling_m'];footprint=")
        source=replace_once(source,"str(BASE/'ws/install/navigation_adapter/lib/navigation_adapter/terrain_s13')", "str(BASE/'ws/install/mission_bt/lib/mission_bt/terrain14')")
        start=source.index('    def start_slam(name):')
        end=source.index('    def set_hint(',start)
        source=source[:start]+"    def start_slam(name):\n        return launch(name,[str(BASE/'venvs/scene/bin/python'),str(S14PKG/'scripts/regional_localizer14.py'),'--out',str(out/'quality'),'--request',str(out/'request.json')])\n"+source[end:]
        source=replace_once(source,"slam=start_slam('slam');launch('quality',['/usr/bin/python3',str(LOCAL/'localization_quality.py'),'--out',str(out/'quality'),'--request',str(out/'request.json')])", "slam=start_slam('slam')")
        source=replace_once(source,"manager.update(map_id=manifest['map_id'],diagnostic_fixture=True)", "manager.update(map_id=S14ATLAS['map_id'],diagnostic_fixture=True)")
        source=replace_once(source,"str(S08/'source_adapter.py')", "str(S14PKG/'scripts/regional_source14.py')")
        source=replace_once(source,"'--out',str(out/'sources'),'--case','normal'", "'--out',str(out/'sources'),'--case','false_fixed_and_loss' if case.startswith('regional_probe_fault') else 'normal'")
        source=replace_once(source,"str(common13.PKG/'scripts/prior_cost13.py')", "str(S14PKG/'scripts/regional_prior14.py')")
        source=replace_once(source,"common13.plan()['plan_revision']", "S14PLAN()['plan_revision']")
        source=replace_once(source,"        nav_config=BASE/f'config/nav2_{kind}.yaml'", "        from navigation_config14 import compose as compose_nav\n        nav_config=compose_nav(kind,out)")
    module = types.ModuleType('s14_pipeline')
    module.__file__ = str(NAV / 'scripts/runtime13.py')
    module.S14PKG=PKG
    if registered:module.S14ATLAS=atlas;module.S14PLAN=plan
    exec(compile(source, module.__file__, 'exec'), module.__dict__)
    return module

def run_case(*args, probe=False, survey=False, **kwargs):
    out=Path(args[0]);interrupted=[];case=args[2];old_region=os.environ.pop('S14_REGISTERED_REGION',None)
    registered=case.startswith('regional_probe_') or (not survey and (case.startswith('nominal_') or case in ['ackermann_navigation_probe','ackermann_registered_route_probe','spray_registered_docking_probe']))
    kind=kwargs.get('kind',args[7] if len(args)>7 else 'diff')
    if registered:os.environ['S14_REGISTERED_REGION']='A' if kind=='diff' else 'B'
    try:
        result=runtime(probe,survey,registered).run_case(*args, **kwargs)
        if registered:
            from binding14 import region
            a,_=region();result['regional_binding']=dict(region=os.environ['S14_REGISTERED_REGION'],map_id=a['map_id'],frame_epoch=a['frame_epoch'],atlas_revision=a['revision'],plan_revision=plan()['plan_revision']);dump(out/'runtime.json',result)
        return result
    finally:
        previous={s:signal.getsignal(s) for s in [signal.SIGINT,signal.SIGTERM,signal.SIGALRM]}
        for s in previous:signal.signal(s,lambda signum,frame:interrupted.append(signum))
        try:
            cleaned=cleanup(out)
            if cleaned.get('survivors'):raise RuntimeError('OWNED_PROCESS_CLEANUP_FAILED '+str(cleaned['survivors']))
        finally:
            for s,h in previous.items():signal.signal(s,h)
            os.environ.pop('S14_REGISTERED_REGION',None)
            if old_region is not None:os.environ['S14_REGISTERED_REGION']=old_region
        if interrupted:raise KeyboardInterrupt('STOP_DURING_CLEANUP')
