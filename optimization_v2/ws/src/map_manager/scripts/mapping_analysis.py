"""Offline scorer only. Evaluation domain data must never reach online nodes."""
import gzip,importlib.util,sys
from scipy.spatial import cKDTree
from s06_common import *
def rows(p):
    p=Path(p)
    with p.open() if p.exists() else gzip.open(str(p)+'.gz','rt') as f:return [json.loads(s) for s in f if s.strip()]
def ply(path):
    with Path(path).open() as f:
        header=[]
        while True:
            s=f.readline().strip();header.append(s)
            if s=='end_header':break
            if not s:raise ValueError('invalid PLY header')
        count=int(next(s.split()[-1] for s in header if s.startswith('element vertex ')))
        a=np.loadtxt(f,dtype=np.float32,max_rows=count,usecols=(0,1,2))
    return a[:,:3]
def score_poses(nodes,truth,key):
    tt=np.array([r['t'] for r in truth]);xyz=np.array([r['xyz'] for r in truth]);rp=np.array([r['rpy'] for r in truth]);rp[:,2]=np.unwrap(rp[:,2]);origin=np.array([np.interp(5.,tt,xyz[:,i]) for i in range(3)]);r0=Rotation.from_euler('z',np.interp(5.,tt,rp[:,2]))
    ts=np.array([n['stamp'] for n in nodes]);estimate=np.array([n[key][:3] for n in nodes]);ref=r0.inv().apply(np.array([np.interp(ts,tt,xyz[:,i]) for i in range(3)]).T-origin);err=np.linalg.norm(estimate-ref,axis=1)
    return dict(ate_rmse_m=float(np.sqrt(np.mean(err**2))),ate_max_m=float(err.max()),endpoint_m=float(err[-1])),dict(t=ts.tolist(),xyz=estimate.tolist(),truth=ref.tolist(),errors=err.tolist())
def ghosting(graph,key):
    ids=list(graph.get('loop_scans',{}))
    if len(ids)!=2:return dict(available=False)
    nodes={x['id']:x for x in graph['nodes']};clouds=[]
    for id in ids:
        a=np.array(graph['loop_scans'][id]);a=a[a[:,2]>.2];p=nodes[int(id)][key];clouds.append(transform(a,p[:3],p[3:]))
    if min(map(len,clouds))<50:return dict(available=False,reason='Insufficient above-ground points')
    d=cKDTree(clouds[0]).query(clouds[1],k=1)[0];overlap=d[d<1.5]
    if not len(overlap):return dict(available=False,points=len(d),overlap_points=0,overlap_fraction=0.,reason='No overlapping surface within 1.5 m; not zero error')
    return dict(available=len(overlap)>50,points=len(d),overlap_points=len(overlap),overlap_fraction=float(len(overlap)/len(d)),median_m=float(np.median(overlap)),p95_m=float(np.quantile(overlap,.95)),definition='Above-ground nearest-surface disagreement on the same revisited pair; gate 1.5 m; includes dynamic residue')
def revisit_alignment(out,graph,key):
    # A save/resume edge between nearby timestamps is not a route revisit.
    byid={x['id']:x for x in graph['nodes']}
    links=[x for x in graph['links'] if x['type'] in [1,2] and abs(byid[x['from']]['stamp']-byid[x['to']]['stamp'])>=30]
    if not links:return dict(available=False)
    link=min(links,key=lambda x:max(byid[x['from']]['stamp'],byid[x['to']]['stamp']))
    snapshots=sorted((Path(out)/'mapping_inputs').glob('cloud_*.npz'));clouds=[];chosen=[]
    for id in [link['from'],link['to']]:
        node=byid[id];f=min(snapshots,key=lambda f:abs(float(f.stem.split('_')[1])-node['stamp']));d=np.load(f);dt=abs(float(d['t'])-node['stamp'])
        if dt>3.1:return dict(available=False,reason='No retained snapshot sufficiently close to revisit keyframe')
        points=d['raw'][d['keep']];points=points[points[:,2]>.2];raw=transform(points,d['xyz'],d['q']);pose=node[key];origin=node['raw'];rot=Rotation.from_quat(pose[3:])*Rotation.from_quat(origin[3:]).inv()
        clouds.append(rot.apply(raw-np.array(origin[:3]))+pose[:3]);chosen.append(dict(node=id,node_stamp=node['stamp'],snapshot=f.name,snapshot_stamp=float(d['t']),time_difference_s=dt))
    if min(map(len,clouds))<50:return dict(available=False,reason='Insufficient above-ground points')
    distance=cKDTree(clouds[0]).query(clouds[1],k=1)[0];overlap=distance[distance<1.5]
    if not len(overlap):return dict(available=False,reason='No overlapping measured surface within 1.5 m',snapshots=chosen,overlap_fraction=0.)
    return dict(available=len(overlap)>50,points=len(distance),overlap_points=len(overlap),overlap_fraction=len(overlap)/len(distance),median_m=float(np.median(overlap)),p95_m=float(np.quantile(overlap,.95)),snapshots=chosen,definition='Measured stable above-ground snapshots near the first loop with >=30 s separation; nearest keyframe correction applied to each observation; 1.5 m overlap gate, snapshot offsets disclosed')
def dynamic_residue(out,truth):
    private=read(BASE/'evaluation/truth_manifest.json');actors=private['actors'];tt=np.array([r['t'] for r in truth]);pos=np.array([r['xyz'] for r in truth]);rp=np.array([r['rpy'] for r in truth]);rp[:,2]=np.unwrap(rp[:,2]);counts=dict(dynamic_points=0,dynamic_kept=0,other_points=0,other_kept=0,snapshots=0,terrain_kept_points={a['id']:0 for a in private['terrain']})
    for f in (out/'mapping_inputs').glob('cloud_*.npz'):
        d=np.load(f);t=float(d['t']);p=np.array([np.interp(t,tt,pos[:,i]) for i in range(3)]);q=Rotation.from_euler('xyz',[np.interp(t,tt,rp[:,i]) for i in range(3)]).as_quat();xyz=transform(d['raw'],p,q);mask=np.zeros(len(xyz),bool)
        for a in actors:
            phase=2*math.pi*t/a['period']+a['phase'];c=np.array(a['center'])+[a['amplitude'][0]*math.sin(phase),a['amplitude'][1]*math.sin(2*phase),0];mask|=(np.abs(xyz-c)<(np.array(a['size'])/2+.10)).all(axis=1)
        for terrain in private['terrain']:
            ox,oy,_=terrain['origin'];dx=xyz[:,0]-ox;height=np.minimum(dx,terrain['length']-dx)*math.tan(math.radians(terrain['slope_deg']));surface=(dx>0)&(dx<terrain['length'])&(xyz[:,1]>oy)&(xyz[:,1]<oy+terrain['width'])&(height>.06)&(abs(xyz[:,2]-height)<.06)
            counts['terrain_kept_points'][terrain['id']]+=int((surface&d['keep']).sum())
        counts['dynamic_points']+=int(mask.sum());counts['dynamic_kept']+=int((mask&d['keep']).sum());counts['other_points']+=int((~mask).sum());counts['other_kept']+=int((~mask&d['keep']).sum());counts['snapshots']+=1
    counts['dynamic_rejection_fraction']=1-counts['dynamic_kept']/max(1,counts['dynamic_points']);counts['other_retained_fraction']=counts['other_kept']/max(1,counts['other_points']);counts['label_policy']='Private actor bounds used ONLY after shutdown for scoring; tolerance 0.10m, not perfect semantic labels';return counts
def loop_event_comparison(out,graph,truth):
    snapshots=rows(out/'online/graphs.jsonl');before=None;after=None;byid={x['id']:x for x in graph['nodes']}
    for snapshot in snapshots:
        if any(x['type'] in [1,2] and x['source'] in byid and x['target'] in byid and abs(byid[x['source']]['stamp']-byid[x['target']]['stamp'])>=30 for x in snapshot['links']):after=snapshot;break
        before=snapshot
    if before is None or after is None:return dict(available=False)
    b=dict(zip(before['ids'],before['poses']));a=dict(zip(after['ids'],after['poses']));byid={x['id']:x for x in graph['nodes']};common=sorted(set(b)&set(a)&set(byid))
    if not common:return dict(available=False)
    nodes=[dict(stamp=byid[i]['stamp'],before=b[i],after=a[i]) for i in common]
    bscore,bplot=score_poses(nodes,truth,'before');ascore,aplot=score_poses(nodes,truth,'after');shift=np.linalg.norm(np.array([a[i][:3] for i in common])-np.array([b[i][:3] for i in common]),axis=1)
    return dict(available=True,before_t=before['t'],after_t=after['t'],common_nodes=len(common),before=bscore,after=ascore,max_historical_correction_m=float(shift.max()),before_path=bplot,after_path=aplot,definition='Same historical node IDs in the last retained pre-event graph and the first measured loop-event graph; neighbour refinement and gravity were already active before the event')
def analyze(out):
    out=Path(out);rt=read(out/'runtime.json');end=read(out/'online/result.json');graph=read(out/'db_graph.json');truth=rows(out/'evaluation/truth.jsonl');obs=rows(out/'online/observations.jsonl');frames=rows(out/'mapping_inputs/frames.jsonl');checks={};m=dict(case=rt['case'],checks=checks)
    checks['completed']=rt['success'] and end['completed'];checks['database_reopened']=graph['reopened_read_only'] and len(graph['nodes'])>=15;checks['real_sensor_payloads']=all(n['compressed_scan_bytes']>0 and n['compressed_image_bytes']>0 for n in graph['nodes'])
    cloud=ply(out/'map_optimized_cloud.ply');m['map_points']=len(cloud);m['height_range_m']=[float(np.quantile(cloud[:,2],.01)),float(np.quantile(cloud[:,2],.99))];checks['real_3d_height']=m['height_range_m'][1]-m['height_range_m'][0]>2
    m['nodes']=len(graph['nodes']);m['loops']=[l for l in graph['links'] if l['type'] in [1,2] and l['from']<l['to']]
    if not m['loops']:m['loops']=[l for l in graph['links'] if l['type'] in [1,2]]
    checks['loop_closure']=bool(m['loops']) and graph.get('accepted_loop_recheck',{}).get('accepted',False)
    m['loop_recheck']=graph.get('accepted_loop_recheck');m['false_candidate']=graph.get('false_candidate');checks['false_loop_rejected']=bool(m['false_candidate']) and not m['false_candidate'].get('visual_then_icp_accepted',True) and m['false_candidate']['raw_separation_m']>3
    for candidate in [m['loop_recheck'],m['false_candidate']]:
        if candidate and candidate.get('rms_m')==0:candidate['rms_m']=None;candidate['rms_status']='Not populated by this backend; use independently measured alignment below'
    infos=[x for x in obs if x['key']=='info'];m['online_rejected_hypotheses']=sum(x['stats'].get('Loop/RejectedHypothesis/',0)>0 for x in infos);m['visual_loops']=sum(x.get('loop',0)>0 for x in infos);m['proximity_loops']=sum(x.get('proximity',0)>0 for x in infos)
    plots={}
    for key in ['raw','optimized']:
        m[key+'_trajectory'],plots[key]=score_poses(graph['nodes'],truth,key)
        m[key+'_first_database_edge_alignment']=ghosting(graph,key)
        m[key+'_alignment']=revisit_alignment(out,graph,key)
    m['first_loop_event']=loop_event_comparison(out,graph,truth);checks['actual_before_after_event']=m['first_loop_event']['available']
    m['gravity_links']=sum(l['type']==9 for l in graph['links']);checks['gravity_observed']=m['gravity_links']>=len(graph['nodes'])//2 and graph.get('stored_optimizer_parameters',{}).get('odom_gravity')=='true'
    checks['alignment_measured']=m['optimized_alignment']['available'];checks['map_quality']=m['optimized_alignment'].get('p95_m',999)<.5 and m['optimized_trajectory']['ate_rmse_m']<1.
    m['dynamic_residue']=dynamic_residue(out,truth);checks['dynamic_measurement']=m['dynamic_residue']['dynamic_points']>0
    audit=[x for x in end['audits'] if x['t']>10];checks['unique_tf_and_actuator']=bool(audit) and all(sorted(x['tf'])==['s05_local_odometry','s06_slam'] and x['actuator']==['s03_drive_bridge'] for x in audit)
    checks['truth_isolation']=not any('/evaluation/' in name or name.startswith('/world/') or '/s04/raw/' in name for name in end['subscriptions'])
    f=[x for x in frames if 'raw' in x];m['sync_max_s']=max(x['max_sync_error_s'] for x in f);checks['sensor_sync']=m['sync_max_s']<=.04 and all(x['pair_error_s']==0 for x in f)
    lifecycle=[x for x in obs if x['key']=='lifecycle'];m['lifecycle_states']=list(dict.fromkeys(x['state'] for x in lifecycle));checks['pause_save_resume']=all(x in m['lifecycle_states'] for x in ['COLLECTING','WAIT_STOP','PAUSED','SAVED']) and not any(x['state']=='FAULT' for x in lifecycle)
    paused=[x for x in lifecycle if x['state']=='PAUSED'];saved=[x for x in lifecycle if x['state']=='SAVED'];m['physical_stops']={}
    sys.path.insert(0,str(BASE/'ws/src/chassis_bridge/scripts'));spec=importlib.util.spec_from_file_location('s06_s03_stop_score',BASE/'ws/src/chassis_bridge/scripts/analyze.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);physical=mod.physical(truth)
    waits=[x for i,x in enumerate(lifecycle) if x['state']=='WAIT_STOP' and (not i or lifecycle[i-1]['state']!='WAIT_STOP')]
    for i,x in enumerate(waits):m['physical_stops'][str(i)]=mod.stop_score(physical,x['t'],x['t']+3)
    checks['physical_stop_thresholds']=len(waits)>=2 and all(x['passed'] for x in m['physical_stops'].values())
    if paused:
        ts=[x['t'] for x in paused];during=[x for x in infos if min(ts)+.1<x.get('received_at_clock',x['t'])<max(ts)-.1];checks['pause_really_stops_collection']=not during
    else:checks['pause_really_stops_collection']=False
    checks['save_then_more_nodes']=bool(saved) and sum(n['stamp']>saved[-1]['t'] for n in graph['nodes'])>=5
    from mapping_export import verify_manifest
    checks['version_bound']=verify_manifest(out/'map_manifest.json')
    xyz=np.array([x['xyz'] for x in truth]);m['physical_distance_m']=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum());m['physical_bounds']=[xyz.min(axis=0).tolist(),xyz.max(axis=0).tolist()];contacts=read(out/'evaluation/contacts.json');m['obstacle_contacts']={k:v for k,v in contacts.items() if 'vehicle::' in k and not any(s in k for s in ['ground::','grade_'])};checks['no_obstacle_contact']=not m['obstacle_contacts']
    motion=read(out/'motion/result.json');checks['route_completed']=motion['goal'];m['motion']=motion
    if rt['case']=='mapping_full':
        checks['both_zones_and_connector']=m['physical_bounds'][0][0]<50 and m['physical_bounds'][1][0]>124
        checks['measured_terrain_surfaces']=sum(m['dynamic_residue']['terrain_kept_points'].values())>20
    res=read(out/'resources.json');m['peak_owned_rss_bytes']=max(x['owned_rss'] for x in res);m['wall_seconds']=rt['wall_seconds'];m['rtf']=rt['seconds']/rt['wall_seconds'];checks['resource_bound']=m['peak_owned_rss_bytes']<8589934592
    if rt['case']=='weak_texture':
        # The native low-texture scene is a diagnostic case, not a successful map.
        checks={k:v for k,v in checks.items() if k in ['completed','database_reopened','real_sensor_payloads','unique_tf_and_actuator','truth_isolation','sensor_sync','pause_save_resume','physical_stop_thresholds','version_bound','resource_bound','no_obstacle_contact']}
        checks['feature_poverty_diagnosed']=m['online_rejected_hypotheses']>0 or m['visual_loops']==0;m['checks']=checks;m['diagnostic']='Original untextured S02 retained; visual loop absence/rejection is measured. Geometric proximity loops, if any, do not prove visual place recognition.'
    m['passed']=all(checks.values());dump(out/'metrics.json',m);dump(out/'plot_data.json',plots);return m
