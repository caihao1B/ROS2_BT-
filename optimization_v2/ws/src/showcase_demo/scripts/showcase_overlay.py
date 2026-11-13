"""Extend a case-local S17 adapter for the independent demonstration only."""
from common_showcase import *
from overlay17 import build,replace

def compose(out,rtf=1.):
    scripts=build(out,BASE/'results/S17/qualification/stations.json',rtf,full=True)
    captured=out/'showcase_source/scripts';captured.mkdir(parents=True,exist_ok=True);bindings={}
    for origin in sorted((PKG/'scripts').glob('*.py')):
        source=origin.read_text();bindings[str(origin.relative_to(ROOT))]=sha(origin)
        if origin.name=='common_showcase.py':source=replace(source,'ROOT=Path(__file__).resolve().parents[5]','ROOT=Path('+repr(str(ROOT))+')')
        (captured/origin.name).write_text(source)
    dump(out/'showcase_sources.json',bindings)
    scene=scripts/'scene14.py';s=scene.read_text();needle="    root.write(out / 'world.sdf', encoding='unicode')"
    s=replace(s,needle,needle+'\n    sys.path.insert(0,'+repr(str(captured))+')\n    from interaction_scene import apply as interaction_scene\n    interaction_scene(out,kind)');scene.write_text(s)
    s=scene.read_text();s=replace(s,"    if os.environ.get('S17_OPERATION')=='inspection':","    if os.environ.get('S17_OPERATION')=='inspection' and not os.environ.get('S17_RESUME_MANIFEST'):");scene.write_text(s)
    helper=out/'s17_source/scripts/dynamic_envelope17.py';s=helper.read_text()
    old="count+=int(np.sum(mask&(block!=100)));block[mask]=100;cost[np.ix_(ys,xs)]=block"
    new="selected=mask&(block>=0)&(block<90);count+=int(np.sum(selected));block[selected]=np.maximum(block[selected],78);cost[np.ix_(ys,xs)]=block"
    s=replace(s,old,new);helper.write_text(s)
    mission=scripts/'mission14.py';text=mission.read_text()
    text=replace(text,"from common14 import *","from common14 import *\nsys.path.insert(0,"+repr(str(captured))+")\nfrom motion_monitor import snapshot")
    text=replace(text,"self.admission_recovery=AdmissionRecovery(policy['clear_hold_s'],policy['max_block_s'])","self.admission_recovery=AdmissionRecovery(policy['clear_hold_s'],policy['max_block_s']);self.startup_timeout=policy['startup_timeout_s']")
    text=replace(text,"if self.st['t']<2 and self.reason=='INPUT_STALE':continue","from startup_guard import pending_stale_context\n                if pending_stale_context(self.initialized,self.reason,self.st['t'],self.startup_timeout):continue")
    text=replace(text,"    def pulse(self):\n", "    def pulse(self):\n        snapshot(self)\n")
    text=replace(text,"stopped=self.halt();saved=self.save()", "stopped=self.halt();saved=self.save()\n        from motion_monitor import snapshot\n        snapshot(self,terminal=True)")
    request="dump(request,dict(plan_revision=self.c['plan_revision'],cluster=execution_cluster(self.a.vehicle),residual=mapping(self.area.difference(self.credit)),pose_map=[*self.api['xy'](),yaw(p.orientation)],out=str(directory),attempt=self.patches))"
    text=replace(text,request,"from residual_showcase import capture_grid\n        "+request.replace('dump(request,dict(','dump(request,capture_grid(self,dict(')[:-2]+'),directory))')
    old="for i,segment in enumerate(self.patch_candidate['segments']):self.pending.extend(self.segment_jobs(segment,f'patch_{self.patches}_{i}'))"
    new="for i,segment in enumerate(self.patch_candidate['segments']):\n                        if segment.get('showcase_measured_reference'):self.pending.append(self.transit_job(segment,f'patch_{self.patches}_{i}',reference=True))\n                        else:self.pending.extend(self.segment_jobs(segment,f'patch_{self.patches}_{i}'))"
    text=replace(text,old,new)
    text=replace(text,"        restore_mission(self)","        restore_mission(self)\n        from inspection_recovery import restore as restore_inspection\n        restore_inspection(self)")
    inspection=out/'s17_source/scripts/inspection17.py';ins=inspection.read_text()
    ins=replace(ins,"        m.log('inspection_part_complete',id=part['id'])","        m.log('inspection_part_complete',id=part['id'])\n        from inspection_recovery import completed_part\n        completed_part(m,part)")
    inspection.write_text(ins);compile(ins,str(inspection),'exec')
    (captured/'showcase_inspection17.py').write_text(ins)
    text=replace(text,"from inspection17 import enabled as inspecting, jobs as inspection_jobs, execute as inspection_execute","from showcase_inspection17 import enabled as inspecting, jobs as inspection_jobs, execute as inspection_execute")
    text=replace(text,"        restore_inspection(self)","        restore_inspection(self)\n        if inspecting():self.log('inspection_executor_binding',path=inspection_execute.__code__.co_filename)")
    mission.write_text(text);compile(text,str(mission),'exec')
    (scripts/'residual14.py').write_text('import sys\nsys.path.insert(0,'+repr(str(captured))+')\nfrom residual_showcase import main\nif __name__=="__main__":main()\n')
    nav=scripts/'navigation_config14.py';s=nav.read_text()
    needle="params['DWB']['xy_goal_tolerance']=.025"
    s=replace(s,needle,needle+";params['DWB']['vtheta_samples']=41;params['DWB']['angular_granularity']=.01")
    nav.write_text(s);compile(s,str(nav),'exec')
    if os.environ.get('SHOWCASE_PLAN'):
        binding=scripts/'binding14.py';(scripts/'binding_formal14.py').write_text(binding.read_text())
        binding.write_text('import sys\nsys.path.insert(0,'+repr(str(captured))+')\nfrom showcase_binding import *\n')
    prior=scripts/'regional_prior14.py';s=prior.read_text()
    # The original helper is snapshotted, then this independent demonstration
    # replaces only historical terrain storage. Raw lethal/frozen prior remain.
    helper_import='import sys\nsys.path.insert(0,'+repr(str(captured))+')\nfrom dynamic_terrain_memory import DynamicTerrainMemory as MapMemory'
    patch='    source=replace_once(source,'+repr('from map_memory17 import MapMemory')+','+repr(helper_import)+')\n'
    subscription=" n.create_subscription(String,'/s09/objects',lambda msg:st.update(moving_objects=json.loads(msg.data)),10)\n def grid(m):"
    patch+='    source=replace_once(source,'+repr(' def grid(m):')+','+repr(subscription)+')\n'
    patch+='    source=replace_once(source,'+repr('np.array([tx,ty]),t)')+','+repr("np.array([tx,ty]),t,st.get('moving_objects'))")+')\n'
    patch+='    source=replace_once(source,'+repr('retained_lethal=int(np.sum(remember==100)),unknown=')+','+repr('retained_lethal=int(np.sum(remember==100)),dynamic_history_filtered=stable_memory.removed_dynamic,unknown=')+')\n'
    s=replace(s,"    scope=dict(__name__='__main__'",patch+"    scope=dict(__name__='__main__'");prior.write_text(s);compile(s,str(prior),'exec')
    if os.environ.get('SHOWCASE_RECORD')=='1':
        s=scene.read_text();s=replace(s,'    interaction_scene(out,kind)','    interaction_scene(out,kind)\n    from record_scene import apply as record_scene\n    record_scene(out)');scene.write_text(s)
        runtime=scripts/'runtime14.py';s=runtime.read_text()
        injection="        launch('showcase_camera_bridge',[bridge,'/showcase/observer/image@sensor_msgs/msg/Image[gz.msgs.Image'],evaluation)\n        launch('showcase_recorder',[str(BASE/'venvs/scene/bin/python'),"+repr(str(captured/'record_frames.py'))+",'--out',str(out/'video')],evaluation)\n        drive=launch('drive'"
        hook='    source=replace_once(source,'+repr("        drive=launch('drive'")+','+repr(injection)+')\n'
        s=replace(s,"    module = types.ModuleType('s14_pipeline')",hook+"    module = types.ModuleType('s14_pipeline')");runtime.write_text(s)
    dump(out/'showcase_policy.json',dict(scope='SHOWCASE_ONLY',history='45s observed history is soft 78, live lethal and unknown untouched',terrain_memory='confirmed moving-track history excluded only from supplementary persistent terrain memory; live and frozen occupancy unchanged',motion_monitor='20s warning, 60s without position or heading progress triggers bounded manual handoff; stationary point service exempt',actor_policy='bounded_local_yield_v2_hysteresis',formal_credit=False))
    for p in [scene,helper]:compile(p.read_text(),str(p),'exec')
    return scripts
