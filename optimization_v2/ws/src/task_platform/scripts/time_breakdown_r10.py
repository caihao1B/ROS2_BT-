"""Offline action-level timing; waiting and retries remain part of task time."""
from common import *
from analyze_r02 import rows

def analyze_time():
 pointer=APP/'results/R10/spray_full_A1_latest.json';run=ROOT/read(pointer)['path'];events=rows(run/'mission_events.jsonl');result=read(run/'mission_result.json');nav_start=None;nav_time=0.;failure_at=None;replan_wait=0.;last_complete=0.;points=[];failures=[]
 for e in events:
  kind=e['event']
  if kind in ['NAV_GOAL','REDOCK_NAV_GOAL']:nav_start=e['t']
  if kind in ['NAV_SUCCESS','REDOCK_NAV_SUCCESS','NAV_CANCEL_ACK','NAV_TERMINAL_FAILURE'] and nav_start is not None:nav_time+=max(0.,e['t']-nav_start);nav_start=None
  if kind=='NAV_TERMINAL_FAILURE':failure_at=e['t']
  if kind=='RETRY_DECISION' and failure_at is not None:replan_wait+=max(0.,e['t']-failure_at);failure_at=None
  if kind=='TOOL_RESULT' and e.get('state')=='SUCCEEDED' and not e.get('replayed'):
   points.append(dict(point=e['point'],completion_t=e['t'],since_previous_completion_s=e['t']-last_complete));last_complete=e['t']
  if kind=='DOCKING_SETTLED_CHECK' and not e.get('stable'):
   q=e['stability'];why=[]
   for field,limit in [('median_distance_m',.035),('latest_distance_m',.045),('position_scatter_p95_m',.015),('max_gap_s',.15)]:
    if q.get(field,float('inf'))>limit:why.append(field)
   if q.get('samples',0)<20 or q.get('span_s',0)<1.8:why.append('insufficient_observation_window')
   failures.append(dict(point=e['point'],t=e['t'],attempt=e['attempt'],reasons=why,metrics=q))
 data=dict(run=str(run.relative_to(ROOT)),total_sim_s=result.get('t'),selected_points=result.get('selected_points'),completed_points=len(result.get('completed',[])),navigation_action_intervals_s=nav_time,failed_navigation_to_bt_retry_wait_s=replan_wait,near_goal_progress_replans=sum(e['event']=='NEAR_GOAL_PROGRESS_REPLAN' for e in events),redock_goals=sum(e['event']=='REDOCK_NAV_GOAL' for e in events),failed_docking_checks=failures,point_completion_intervals=points,terminal_error=result.get('error'),scope='Offline observed action intervals, not a causal ablation or pure driving-time measurement; original task clock includes all waiting')
 dump(APP/'results/R10/time_breakdown.json',data);print(json.dumps({k:v for k,v in data.items() if k not in ['failed_docking_checks','point_completion_intervals']}));return data
if __name__=='__main__':analyze_time()
