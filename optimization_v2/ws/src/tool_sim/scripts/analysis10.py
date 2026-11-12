"""Offline cross-domain scoring. Online runtime does not import this module."""
from common10 import *
from shapely.geometry import shape
import sqlite3,gzip

def rows(p):
 p=Path(p)
 if p.exists():return [json.loads(s) for s in p.read_text().splitlines() if s]
 if Path(str(p)+'.gz').exists():return [json.loads(s) for s in gzip.open(str(p)+'.gz','rt') if s.strip()]
 return []
def analyze(out):
 out=Path(out);rt=read(out/'runtime.json');client=read(out/'client/result.json') if (out/'client/result.json').exists() else {};ledger=read(out/'ledger/summary.json') if (out/'ledger/summary.json').exists() else {'jobs':[]};truth=read(out/'tool_evaluation/result.json') if (out/'tool_evaluation/result.json').exists() else {'jobs':{}};events=client.get('events',[]);rr={x['label']:x for x in events if x['event']=='result'};jobs={x['key']:x for x in ledger['jobs']};checks=dict(runtime_completed=rt['success'] and client.get('completed',False),frozen_map_unchanged=rt['database_unchanged']);vehicle=rt['vehicle']
 def state(label,wanted):return rr.get(label,{}).get('state')==wanted
 if vehicle=='diff':
  checks.update(success=state('success','SUCCEEDED'),partial=state('partial','PARTIAL') and 0<rr['partial']['quantity']<2,failure=state('not_docked','FAILED') and rr['not_docked']['quantity']==0,cancel=state('cancel','CANCELLED') and rr['cancel']['action_status']==5,lost=state('lost','LOST') and rr['lost']['uncertain'],duplicate=state('duplicate','SUCCEEDED') and rr['duplicate']['replayed'] and rr['duplicate']['quantity']==rr['success']['quantity'],retry=state('retry_partial','SUCCEEDED'),conflict=rr.get('conflict',{}).get('error')=='IDEMPOTENCY_CONFLICT',loss_requires_review=rr.get('lost_retry_hold',{}).get('error')=='RECOVERY_REQUIRED')
  for mode in ['LOCALIZATION','SPEED','TARGET']:checks['reject_'+mode]=state('invalid_'+mode,'FAILED') and rr['invalid_'+mode]['quantity']==0
 else:
  checks.update(cancel=state('mow_cancel','CANCELLED'),retry=state('mow_retry','SUCCEEDED'),duplicate=state('mow_duplicate','SUCCEEDED') and rr['mow_duplicate']['replayed'] and rr['mow_duplicate']['quantity']==rr['mow_retry']['quantity'],no_transit=state('transit_no_work','FAILED') and rr['transit_no_work']['quantity']==0,no_turn=state('turn_no_work','FAILED') and rr['turn_no_work']['quantity']==0)
 checks['stop_action']=state('stop_work','STOPPED');comparisons=[];physical_rows=rows(out/'tool_evaluation/physical.jsonl');db_goals=sqlite3.connect(out/'ledger/ledger.sqlite');goals={k:json.loads(g) for k,g in db_goals.execute('SELECT key,goal FROM jobs')};db_goals.close();actual_target_doses={}
 for previous,current in zip(physical_rows,physical_rows[1:]):
  key=current['key'];dt=current['t']-previous['t']
  if vehicle=='diff' and key in goals and previous['key']==key and 0<dt<.1:
   target=goals[key]['target'][:2];inside=all(math.dist([x['tool_xyz'][0]-28,x['tool_xyz'][1]-14],target)<=.20 for x in [previous,current])
   if inside:actual_target_doses[key]=actual_target_doses.get(key,0.)+max(0,min(previous['flow'],current['flow']))*dt
 for key,j in jobs.items():
  physical=truth['jobs'].get(key,dict(dose=0.,area_m2=0.,area={'type':'Polygon','coordinates':[]}));q=j['confirmed_quantity']
  if vehicle=='diff':measure=actual_target_doses.get(key,0.);error=max(0.,q-measure);conservative=error<=.06
  else:error=shape(j['credit_geojson']).difference(shape(physical['area'])).area;conservative=error<=.005;measure=physical['area_m2']
  comparisons.append(dict(key=key,confirmed=q,physical=measure,physical_total_dose=physical.get('dose',0.),overclaim=error,conservative=conservative,state=j['state'],remaining=j['remaining_quantity'],uncertain=j['uncertainty_pending']))
 checks['physical_effect_supports_ledger']=bool(comparisons) and all(x['conservative'] for x in comparisons)
 db=sqlite3.connect(out/'ledger/ledger.sqlite');accepted=db.execute('SELECT observation FROM events WHERE accepted=1').fetchall();rejects=dict(db.execute('SELECT reason,COUNT(*) FROM events WHERE accepted=0 GROUP BY reason'));db.close();checks['actual_feedback_credit']=len(accepted)>0;checks['no_nonwork_credit']=all(json.loads(x[0])['context']['segment']=='COVERAGE' for x in accepted) if vehicle=='ackermann' else True
 obs=read(out/'online/result.json');subs=obs.get('subscriptions',{});checks['truth_domain_isolation']=not any(k.startswith('/s10/evaluation') for k in subs);checks['evaluation_process_domain']=all(c['domain']==120 for c in rt['commands'] if c['role']=='tool_evaluator')
 audits=obs.get('audits',[]);active_audits=[x for x in audits if 5<x['t']<=rt.get('sim_seconds_observed',0)];checks['unique_chassis_writer']=len(active_audits)>5 and all(x['actuator']==['s03_drive_bridge'] for x in active_audits) and all(len(x['actuator'])<=1 for x in audits);checks['tool_command_ownership']=len(active_audits)>5 and all(x.get('tools')==['s10_tool_action_server'] for x in active_audits) and all(len(x.get('tools',[]))<=1 for x in audits)
 physical=rows(out/'tool_evaluation/physical.jsonl');duplicate_label='duplicate' if vehicle=='diff' else 'mow_duplicate';requests=[x for x in events if x['event']=='request' and x['label']==duplicate_label];end=rr.get(duplicate_label,{}).get('t',0);during=[x for x in physical if requests and requests[0]['t']-.02<=x['t']<=end+.15];checks['replay_no_physical_restart']=bool(during) and all(abs(x['rpm_rad_s'])<=.1 and not x['on'] for x in during)
 rejected=['not_docked','invalid_LOCALIZATION','invalid_SPEED','invalid_TARGET'] if vehicle=='diff' else ['transit','turn'];checks['rejected_guards_no_physical_effect']=all(truth['jobs'].get(k,{}).get('active_samples',0)==0 for k in rejected)
 checks['physical_stop_at_end']=bool(physical) and all(abs(x['rpm_rad_s'])<=.1 and not x['on'] for x in physical[-5:]);motion=read(out/'motion/result.json');checks['chassis_scope']=motion['distance']<.1 if vehicle=='diff' else .4<motion['distance']<2.2
 resources=read(out/'resources.json');m=dict(passed=all(checks.values()),checks=checks,comparisons=comparisons,results=rr,vehicle=vehicle,sim_seconds=round(max([x['t'] for x in physical]+[0.]),3),work_window_sim_seconds=rt.get('sim_seconds_observed'),wall_seconds=rt['wall_seconds'],peak_rss_gib=max([x['owned_rss_bytes'] for x in resources]+[0])/1024**3,credited_samples=len(accepted),rejections=rejects,distance_m=motion['distance']);dump(out/'metrics.json',m);return m
