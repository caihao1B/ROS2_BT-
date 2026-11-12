"""Standalone browser view of actual offline planner output (never simulator truth)."""
from common12 import *
from motion12 import body_local

def render(out,data,mission=None,faults=None):
    if mission is None:
        (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><h1>S12 未完成</h1><pre>'+__import__('html').escape(json.dumps(data,ensure_ascii=False,indent=2))+'</pre>');return
    routes=[]
    for zone,d in mission['zones'].items():
        for p in d['plans']:
            region=union_all([shape(f['geometry']) for f in d['units']['features'] if f['properties']['cluster']==p['cluster']])
            routes.append(p|dict(id=f'{zone}{p["cluster"]+1}',title=f'{zone} 区 · 簇 {p["cluster"]+1}',region=mapping(region)))
    for i,p in enumerate(mission['connector_routes']):routes.append(p|dict(id=f'road{i}',title=f'连接路 · {p["vehicle"]} · {p["direction"]}',zone='road',region=p['motion_domain']))
    c=config();clean=[]
    for p in routes:
        route={k:p[k] for k in ['id','title','zone','vehicle','timing','region']};route['coverage']=p.get('coverage_check',{});route['comparison']=p.get('comparison',{});route['points']=p.get('points',[]);route['body']=body_local(p['vehicle'])[0].tolist();segments=[];cursor=c['cluster_setup_seconds']
        for s in p['segments']:
            work=s['kind'] in ['COVERAGE','HEADLAND_WORK'];before=c['startup_seconds'] if work else 0.;after=c['shutdown_seconds'] if work else s.get('action_seconds',0)
            slim={k:s[k] for k in ['kind','tool_on','poses','timing','validation']};slim.update(t0=cursor,motion_start=cursor+before,motion_end=cursor+before+s['timing']['motion_seconds'],t1=cursor+before+s['timing']['motion_seconds']+after,task_id=s.get('task_id'),target_xy=s.get('target_xy'))
            segments.append(slim);cursor=slim['t1']
        route.update(segments=segments,base_seconds=cursor,reserve_seconds=p['timing']['reserve_s'],min_clearance_m=min(s['validation']['min_clearance_m'] for s in p['segments']),max_curvature=max(s['validation']['max_curvature'] for s in p['segments']),steering_stops=sum(len(s['timing'].get('steering_events',[])) for s in p['segments']))
        clean.append(route)
    payload=dict(default_case='original' if data.get('case')=='infeasible_and_stale' else 'normal',status=data['status'],run_id=data['run_id'],routes=clean,zones={z:{k:d[k] for k in ['original_domain','task_domain','out_of_scope','original_area_m2','task_area_m2','scale','iteration','partition_verification']} for z,d in mission['zones'].items()},attempts=data.get('attempts',{}),faults=faults or {},revision=mission['plan_revision'],offline=True)
    text=(PKG/'scripts/report_template.html').read_text().replace('__DATA__',json.dumps(payload,ensure_ascii=False,separators=(',',':')).replace('</','<\\/'))
    plan_path=out/'validated/mission_plan.json' if (out/'validated/mission_plan.json').exists() else BASE/'plans/validated/mission_plan.json'
    text=text.replace('__PLAN_LINK__',os.path.relpath(plan_path,out))
    (out/'index.html').write_text(text)
