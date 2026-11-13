"""Generate an isolated, exact-match S14 adapter; never edit upstream receipts."""
from common17 import *

def replace(s,a,b):
    if s.count(a)!=1:raise ValueError('ADAPTER_MISMATCH '+a[:100])
    return s.replace(a,b)

def build(destination,stations,rtf=.4,full=False):
    destination=Path(destination);scripts=destination/'scripts';scripts.mkdir(parents=True,exist_ok=True)
    upstream=BASE/'ws/src/mission_bt/scripts';bindings={}
    helpers=PKG/'scripts'
    if full:
        # Retain the executable helper bytes with each run. Editing a later
        # experiment must not silently change a running child's imported code.
        helpers=destination/'s17_source/scripts';helpers.mkdir(parents=True,exist_ok=True)
        for origin in sorted((PKG/'scripts').glob('*.py')):
            body=origin.read_text();bindings[str(origin.relative_to(ROOT))]=sha(origin)
            if origin.name=='common17.py':body=replace(body,"ROOT = Path(__file__).resolve().parents[5]",'ROOT = Path('+repr(str(ROOT))+')')
            (helpers/origin.name).write_text(body)
    if full:
        # The physical lidar already supplies 10Hz. Preserve all acquired
        # timestamps for the perception branch instead of discarding every
        # second scan before strict near-field pairing. Resolution/noise and
        # the 0.45s stale-data stop remain unchanged.
        original=BASE/'ws/src/sensor_sim_adapter/scripts/fanout.py';s=original.read_text();bindings[str(original.relative_to(ROOT))]=sha(original)
        s=replace(s,'from common import *','import sys\nsys.path.insert(0,'+repr(str(original.parent))+')\nfrom common import *')
        s=replace(s,'        if self.count%2:return\n','')
        (scripts/'fanout17.py').write_text(s)
        original=BASE/'ws/src/navigation_adapter/scripts/near_field13.py';s=original.read_text();bindings[str(original.relative_to(ROOT))]=sha(original)
        s=replace(s,'from common13 import *','import sys\nsys.path.insert(0,'+repr(str(original.parent))+')\nfrom common13 import *')
        s=replace(s,'from fanout import Fanout','from fanout17 import Fanout')
        (scripts/'near_field17.py').write_text(s)
        original=BASE/'ws/src/chassis_bridge/scripts/evaluator.py';s=original.read_text();bindings[str(original.relative_to(ROOT))]=sha(original)
        s=replace(s,"contacts={};stop=[False]","contacts={};stop=[False];contact_log=(a.out/'contact_events17.jsonl').open('w');last_contact={}")
        s=replace(s,"            key=c.collision1.name+'|'+c.collision2.name;contacts[key]=contacts.get(key,0)+1", """            key=c.collision1.name+'|'+c.collision2.name;contacts[key]=contacts.get(key,0)+1
            t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            ground=key.endswith('|ground::body::ground') or key.endswith('|site_terrain::terrain_link::terrain_collision')
            if not ground and t-last_contact.get(key,-100)>=.1:
                last_contact[key]=t;contact_log.write(json.dumps(dict(t=t,pair=key,cumulative_samples=contacts[key]))+'\\n');contact_log.flush()""")
        s=replace(s,"        f.close();(a.out/'contacts.json')","        contact_log.close();f.close();(a.out/'contacts.json')")
        (scripts/'contact_evaluator17.py').write_text(s)
        original=BASE/'ws/src/terrain_perception/scripts/costmap_adapter.py';s=original.read_text();bindings[str(original.relative_to(ROOT))]=sha(original)
        s=replace(s,'from common09 import *','import sys\nsys.path.insert(0,'+repr(str(original.parent))+')\nsys.path.insert(0,'+repr(str(helpers))+')\nfrom temporal_join17 import TerrainJoin\nfrom dynamic_envelope17 import Envelope,to_map\nfrom common09 import *')
        # Boundary validity still uses the original centre admission region.
        # Rasterize the physical authorized edge with its original margin:
        # Nav2 applies the vehicle footprint once. Previously rasterizing the
        # centre-eroded region and inflating again double-counted half-width.
        s=replace(s,"allowed=unary_union(features);regions=", "allowed=unary_union(features);raster_regions={kind:allowed.buffer(-pr['margin_m']) for kind,pr in cfg['profiles'].items()};regions=")
        s=replace(s,"outside=~contains_xy(regions[kind],wx,wy)","outside=~contains_xy(raster_regions[kind],wx,wy)")
        s=replace(s," def update(m,key):state[key]=m", " join=TerrainJoin();last_join=[-100.];dynamic=Envelope()\n def update(m,key):state[key]=m")
        s=replace(s,",('/s09/terrain/health','health')",'')
        s=replace(s,' def grid(m,kind):',' def grid(m,kind,h):')
        s=replace(s,"   h=state['health'];ok=h is not None and h['valid'] and 0<=t-h['t']<.45;", "   last_join[0]=t;ok=h is not None and h['valid'] and abs(t-h['t'])<1e-8 and 0<=n.get_clock().now().nanoseconds*1e-9-t<.45;")
        s=replace(s,"track_cells=marked);msg=String();", "track_cells=marked,grid_stamp=t,health_stamp=h['t'],health_valid=h['valid'],health_reason=h['reason'],processed_at=n.get_clock().now().nanoseconds*1e-9,join='EXACT_STAMP');msg=String();")
        s=replace(s," for kind in pubs:n.create_subscription(OccupancyGrid,'/s09/terrain/'+kind,lambda m,k=kind:grid(m,k),2)", """ def drain():
  now=n.get_clock().now().nanoseconds*1e-9
  for message,kind,health_packet in join.take(now):grid(message,kind,health_packet)
 def receive_grid(m,kind):join.grid(stamp(m),kind,m);drain()
 def receive_health(m):
  d=json.loads(m.data);state['health']=d;join.status(d);drain()
 n.create_subscription(String,'/s09/terrain/health',receive_health,20)
 # A case has exactly one chassis; the inactive profile has no consumers.
 n.create_subscription(OccupancyGrid,'/s09/terrain/'+a.vehicle,lambda m:receive_grid(m,a.vehicle),2)""")
        s=replace(s,' def watchdog():\n', ' def watchdog():\n  drain()\n')
        s=replace(s,"  if h is None or t-h['t']>.45:","  if h is None or t-h['t']>.45 or t-last_join[0]>.45:")
        s=replace(s,"status=state['status'];bound=False;boundary_count=0","status=state['status'];bound=False;boundary_count=0;memory_transform=None")
        s=replace(s,"tr=gp-R@op;wx=", "tr=gp-R@op;memory_transform=(R[:2,:2],tr[:2]);wx=")
        s=replace(s,"  out.data=v.astype('int8').reshape(-1).tolist();","  dynamic_cells=0\n  if memory_transform is not None:\n   rotation,translation=memory_transform;dynamic.update(to_map(state.get('objects'),rotation,translation),t);dynamic_cells=dynamic.apply(v,x,y,rotation,translation)\n  else:dynamic.update(None,t)\n  out.data=v.astype('int8').reshape(-1).tolist();")
        s=replace(s,'track_cells=marked,grid_stamp=t','track_cells=marked,dynamic_memory_cells=dynamic_cells,grid_stamp=t')
        if os.environ.get('S17_PREDICTION','disabled')!='disabled':
            s=replace(s,'from temporal_join17 import TerrainJoin','from temporal_join17 import TerrainJoin\nfrom prediction_cost17 import apply as future_cost')
            s=replace(s,"('/s09/objects','objects')","('/s09/objects','objects'),('/s15/forecasts','prediction')")
            s=replace(s,'  out.data=v.astype(\'int8\').reshape(-1).tolist();','  future_cells=future_cost(v,x,y,state.get(\'prediction\'),t)\n  out.data=v.astype(\'int8\').reshape(-1).tolist();')
            s=replace(s,'dynamic_memory_cells=dynamic_cells,grid_stamp=t','dynamic_memory_cells=dynamic_cells,future_soft_cells=future_cells,grid_stamp=t')
        (scripts/'cost_adapter17.py').write_text(s)
    for p in sorted(upstream.glob('*.py')):
        s=p.read_text();bindings[str(p.relative_to(ROOT))]=sha(p)
        if p.name=='common14.py':s=replace(s,"ROOT = Path(__file__).resolve().parents[5]",'ROOT = Path('+repr(str(ROOT))+')')
        if p.name=='scene14.py':
            s=replace(s,"BASE/'config/s14_survey_stations.json'",'Path('+repr(str(stations))+')')
            s=replace(s,"world.find('physics/real_time_factor').text = '0.4'", "world.find('physics/real_time_factor').text = "+repr(str(rtf)))
            s=replace(s,"    world = root.getroot().find('world')", """    world = root.getroot().find('world')
    if case.startswith('calibration_station_') and os.environ.get('S17_SURVEY_STATIC')=='1':
        removed=[]
        for plugin in list(world.findall('plugin')):
            if plugin.get('name')!='s02::ScriptedMotion':continue
            for actor in plugin.findall('actor'):
                name=actor.findtext('name');model=world.find(\"model[@name='\"+name+\"']\")
                if model is not None:world.remove(model);removed.append(name)
            world.remove(plugin)
        dump(out/'mapping_capture_mode.json',dict(mode='VACANT_FIELD_STATIC_SURVEY_ONLY',removed_dynamic_models=removed,formal_task_scene_unchanged=True))""")
        if p.name=='survey_client14.py':
            # A single noisy wheel sample at the final tick is not a stop
            # acknowledgement. Wait within the existing 8s shutdown reserve,
            # retaining exactly the original speed and tool thresholds.
            old="        publish();o=st['odom'];stopped=bool(o and abs(o.twist.twist.linear.x)<.01 and abs(o.twist.twist.angular.z)<.02 and st['tool'].get('stopped_confirmed'))"
            new="""        publish();stopped=False;hold=None;deadline=time.monotonic()+8;until=min(st['t']+2,a.seconds-6)
        while not st['stop'] and time.monotonic()<deadline and st['t']<until:
            rclpy.spin_once(n,timeout_sec=.01);o=st['odom']
            good=bool(o and abs(o.twist.twist.linear.x)<.01 and abs(o.twist.twist.angular.z)<.02 and st['tool'].get('stopped_confirmed'))
            if not good:hold=None
            elif hold is None:hold=st['t']
            elif st['t']-hold>=.2:stopped=True;break"""
            s=replace(s,old,new)
        if full:
            if p.name=='regional_prior14.py':
                original_prior=(BASE/'ws/src/navigation_adapter/scripts/prior_cost13.py').read_text()
                begin=original_prior.index('   # Vectorized rolling occupancy memory;')
                end=original_prior.index('  fused=fuse(',begin)
                old_memory=original_prior[begin:end]
                new_memory="""   origin=np.array([m.info.origin.position.x,m.info.origin.position.y]);confirmation=st['confirmation'];confirmed=np.zeros_like(v,dtype=bool)
   if confirmation is not None and confirmation.header.frame_id=='odom' and abs(t-stamp(confirmation))<.2 and abs(confirmation.info.resolution-m.info.resolution)<1e-6 and confirmation.info.width==m.info.width and confirmation.info.height==m.info.height and math.dist([confirmation.info.origin.position.x,confirmation.info.origin.position.y],origin)<.001:
    confirmed=np.asarray(confirmation.data).reshape(v.shape)==100
   remember=stable_memory.update(v,confirmed,origin,m.info.resolution,np.array([[co,-si],[si,co]]),np.array([tx,ty]),t)
"""
                imported='from common13 import *\nimport sys\nsys.path.insert(0,'+repr(str(helpers))+')\nfrom map_memory17 import MapMemory'
                memory_patch='    source=replace_once(source,'+repr('from common13 import *')+','+repr(imported)+')\n'
                memory_patch+="    source=replace_once(source,\"memo=dict(cost=None,origin=None,snapshot=-100.);\",\"memo=dict(cost=None,origin=None,snapshot=-100.);stable_memory=MapMemory();\")\n"
                memory_patch+="    source=replace_once(source,"+repr(old_memory)+","+repr(new_memory)+")\n"
                s=replace(s,"    scope=dict(__name__='__main__'",memory_patch+"    scope=dict(__name__='__main__'")
            if p.name=='navigation_config14.py':
                # NavigateToPose uses the navigator XML, whereas FollowPath
                # carries controller_id in its action. Both must select the
                # experiment controller, otherwise a comparison is mislabeled.
                s=replace(s,"    params=c['controller_server']['ros__parameters']", """    selected=os.environ.get('S17_CONTROLLER','DWB' if kind=='diff' else 'RPP')
    controller_parameters=c['controller_server']['ros__parameters']
    if selected not in controller_parameters or not controller_parameters[selected].get('plugin'):raise ValueError('CONTROLLER_NOT_CONFIGURED')
    if selected not in controller_parameters['controller_plugins']:controller_parameters['controller_plugins'].append(selected)
    if kind=='diff' and selected=='RPP':controller_parameters['RPP'].update(use_rotate_to_heading=True,rotate_to_heading_angular_vel=.35)
    tree=replace_once((NAV/'config/navigate.xml').read_text(),'controller_id=\"DWB\"','controller_id=\"'+selected+'\"')
    selected_tree=out/'navigate_selected17.xml';selected_tree.write_text(tree)
    c['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(selected_tree)
    if os.environ.get('S14_REGISTERED_REGION')=='C':
        # Every inspection action is bounded to 8m. Keep its entire detour
        # envelope in the existing observed 30m grid rather than publishing
        # the whole 100m road at 5Hz. Resolution, freshness and footprint stay.
        c['global_costmap']['global_costmap']['ros__parameters'].update(rolling_window=True,width=30,height=30)
    params=c['controller_server']['ros__parameters']""")
                s=replace(s,"'controller_id=\"RPP\"'","'controller_id=\"'+selected+'\"'")
            if p.name=='common14.py':
                s+='\nsys.path.insert(0,'+repr(str(helpers))+')\n'
            if p.name=='runtime14.py':
                shutdown="""        baseline.s04.s03.stop_all([p for p in children if p is not server])
        if server.poll() is None:
            request=gz(['service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','stop: true'])
            result['server_stop_request']=dict(exit_code=request.returncode,acknowledged='true' in request.stdout)
            try:server.wait(timeout=12)
            except subprocess.TimeoutExpired:
                result['server_stop_request']['timed_out']=True
                baseline.s04.s03.stop_all([server])"""
                s=replace(s,"    module = types.ModuleType('s14_pipeline')",'    source=replace_once(source,"        baseline.s04.s03.stop_all(children)",'+repr(shutdown)+')\n'+"    module = types.ModuleType('s14_pipeline')")
                s=replace(s,"    module = types.ModuleType('s14_pipeline')", "    source=replace_once(source,\"str(common13.PKG/'scripts/near_field13.py')\",\"str(S14PKG/'scripts/near_field17.py')\")\n    module = types.ModuleType('s14_pipeline')")
                s=replace(s,"    module = types.ModuleType('s14_pipeline')", "    source=replace_once(source,\"str(S09/'costmap_adapter.py')\",\"str(S14PKG/'scripts/cost_adapter17.py')\")\n    source=replace_once(source,\"str(BASE/'ws/install/gnss_supervisor/lib/gnss_supervisor/gnss_supervisor')\",\"str(BASE/'ws/install/acceptance_runner/lib/acceptance_runner/gnss_supervisor17')\")\n    module = types.ModuleType('s14_pipeline')")
                s=replace(s,"if registered:os.environ['S14_REGISTERED_REGION']='A' if kind=='diff' else 'B'","if registered:os.environ['S14_REGISTERED_REGION']='C' if os.environ.get('S17_OPERATION')=='inspection' else ('A' if kind=='diff' else 'B')")
                s=replace(s,"    module = types.ModuleType('s14_pipeline')", "    source=replace_once(source,\"str(BASE/'ws/src/chassis_bridge/scripts/evaluator.py')\",\"str(S14PKG/'scripts/contact_evaluator17.py')\")\n    module = types.ModuleType('s14_pipeline')")
                if os.environ.get('S17_PREDICTION','disabled')!='disabled':
                    s=replace(s,"    module = types.ModuleType('s14_pipeline')","    from risk_adapter17 import pipeline\n    source=pipeline(source)\n    module = types.ModuleType('s14_pipeline')")
            if p.name=='scene14.py':
                s=replace(s,"    root = E.parse(out / 'world.sdf')", "    if os.environ.get('S17_OPERATION')=='inspection':\n        from inspection17 import route\n        start=route(kind)['start']\n    root = E.parse(out / 'world.sdf')")
                s=replace(s,"    if case.startswith('calibration_station_'):","    if os.environ.get('S17_RESUME_MANIFEST'):start=read(Path(os.environ['S17_RESUME_MANIFEST']))['restart_pose_prior']\n    if case.startswith('calibration_station_'):")
            if p.name=='common14.py':
                s=replace(s,"['plans'][0]","['plans'][int(os.environ['S17_CLUSTER'][1])-1]")
            if p.name=='binding14.py':
                from inputs17 import active
                inputs=active()
                s=s.replace('maps/s14_regions',str(inputs['atlas'].parent.relative_to(BASE))).replace('plans/s14_registered',str(inputs['plan'].parent.relative_to(BASE)))
                s=replace(s,"d['runtime_eligible_clusters']!=['A1','B1']","d['runtime_eligible_clusters']!=['A1','A2','A3','B1','B2','B3']")
                s=replace(s,"if key!='B1':","if key not in ['B1','B2','B3']:")
                s=replace(s,"['plans'][0]","['plans'][int(os.environ['S17_CLUSTER'][1])-1]")
                s=replace(s,".get(zone+'1')",".get(os.environ['S17_CLUSTER'])")
            if p.name=='routing14.py':
                s=replace(s,"route['cluster']!='B1'","route['cluster'] not in ['B1','B2','B3']")
                s=replace(s,"base=original['zones']['B'];target=", "base=copy.deepcopy(original['zones']['B']);base['plans']=[base['plans'][int(route['cluster'][1])-1]];target=")
            if p.name=='mission14.py':
                s=replace(s,"class Suspended(RuntimeError):pass","from inspection17 import enabled as inspecting, jobs as inspection_jobs, execute as inspection_execute\n\nclass Suspended(RuntimeError):pass")
                s=replace(s,"self.scope='FULL_CLUSTER' if a.case.startswith('nominal_') else 'SMOKE'","self.scope='INSPECTION' if inspecting() else ('FULL_CLUSTER' if a.case.startswith('nominal_') else 'SMOKE')")
                s=replace(s,"        node.create_subscription(String,'/s14/directive',self.receive,5)","        node.create_subscription(String,'/s14/directive',self.receive,5)\n        node.create_subscription(String,'/s09/objects',lambda msg:st.update(inspection_objects=json.loads(msg.data)),10)")
                original_budget="budget_evidence=estimate(self.pending[0],self.api['xy']() if self.st.get('pose') is not None else None,t,self.busy,self.st.get('nav_plan'),live_replan=self.executing_action in ['REPLAN','DETOUR']);next_min=budget_evidence['seconds']"
                updated_budget=original_budget+"\n                if inspecting():\n                    from inspection_budget17 import estimate as inspection_eta\n                    budget_evidence=inspection_eta(self.pending[0],self.api['xy']() if self.st.get('pose') is not None else None,t,self.busy,self.st.get('nav_plan'),getattr(self,'inspection_completed_parts',[]),getattr(self,'inspection_executed_parts',{}));next_min=budget_evidence['seconds']"
                s=replace(s,original_budget,updated_budget)
                s=replace(s,'alternative=len(self.pending)>1','alternative=len(self.pending)>1 and not inspecting()')
                s=replace(s,"        self.credit=Polygon();self.reason=", "        if inspecting():self.area=Polygon()\n        self.credit=Polygon();self.reason=")
                s=replace(s,"    def make_jobs(self):\n", "    def make_jobs(self):\n        if inspecting():return inspection_jobs(self)\n")
                s=replace(s,"    def execute_job(self,j,replan=False):\n", "    def execute_job(self,j,replan=False):\n        if inspecting():return inspection_execute(self,j,replan)\n")
                s=replace(s,"len(self.done)==len(self.jobs) if self.a.case in", "len(self.done)==len(self.jobs) if inspecting() or self.a.case in")
                if s.count("self.p['zone']+'1'")!=2:raise ValueError('CLUSTER_ID_ADAPTER')
                s=s.replace("self.p['zone']+'1'","os.environ['S17_CLUSTER']")
                s=replace(s,"f'B1_{i}'","f'{os.environ[\"S17_CLUSTER\"]}_{i}'")
                s=replace(s,"f'{self.p[\"zone\"]}1_{i}'","f'{os.environ[\"S17_CLUSTER\"]}_{i}'")
                s=replace(s,"id='A1_return'","id=os.environ['S17_CLUSTER']+'_return'")
                s=replace(s,"    def make_jobs(self):", "        sys.path.insert(0,"+repr(str(helpers))+")\n        from resume17 import restore_mission\n        restore_mission(self)\n\n    def make_jobs(self):")
                if os.environ.get('S17_PREDICTION','disabled')!='disabled':
                    from risk_adapter17 import mission
                    s=mission(s)
        (scripts/p.name).write_text(s)
    dump(destination/'upstream.json',bindings)
    return scripts

if __name__=='__main__':build(RESULTS/'qualification/overlay',RESULTS/'qualification/stations.json')
