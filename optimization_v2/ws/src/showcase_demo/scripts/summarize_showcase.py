"""Explicitly selected real runs; one demonstration receipt, no formal credit."""
from common_showcase import *
import argparse

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--runs',type=Path,nargs='+',required=True);a=ap.parse_args();entries=[];keys=set();interactions=[];behavior=[];all_episodes=[]
    for p in a.runs:
        p=p.resolve();spec=read(p/'spec.json');r=read(p/'showcase_result.json');key=spec['cluster'] if spec['operation']=='work' else 'road_'+spec['vehicle']
        if key in keys:raise ValueError('DUPLICATE_SELECTED_OBLIGATION '+key)
        keys.add(key);entries.append(dict(key=key,run=str(p.relative_to(ROOT)),status=r['status'],complete=r['complete'],safety=r['safety'],localization_pass=r['localization_pass'],declared_demo_denominator=r['declared_demo_denominator'],plan_revision=r['plan_revision'],required_points=r['required_points'],confirmed_points=r['confirmed_points'],required_area_m2=r['required_area_m2'],confirmed_area_m2=r['confirmed_area_m2'],coverage_fraction=r['coverage_fraction'],sim_seconds=r['sim_seconds_including_stop'],wall_seconds=r['wall_seconds'],position_p95_m=r['position_error_m']['p95'],metrics_sha256=sha(p/'showcase_result.json')))
        episodes=r.get('execution_episodes',[str(p.relative_to(ROOT))]);episode_metrics=[read(ROOT/e/'showcase_result.json') for e in episodes]
        if set(episodes)&set(all_episodes):raise ValueError('SHARED_EPISODE_CANNOT_COUNT_TWICE')
        all_episodes.extend(episodes)
        entries[-1].update(execution_episodes=episodes,sim_seconds=sum(e['sim_seconds_including_stop'] for e in episode_metrics),wall_seconds=sum(e['wall_seconds'] for e in episode_metrics),resume_chain_verified=r.get('resume_chain_verified',len(episodes)==1),position_p95_m=max(e['position_error_m']['p95'] for e in episode_metrics))
        events=[json.loads(line) for episode in episodes for line in (ROOT/episode/'client/events.jsonl').read_text().splitlines()]
        behavior.append(dict(key=key,repair_probe=spec.get('repair_probe',False),events_sha256=sha(p/'client/events.jsonl'),bt_replans=sum(e['event']=='replan_attempt' for e in events),tool_confirmations=[{k:e.get(k) for k in ['key','state','reason','quantity','uncertain']} for e in events if e['event']=='tool_result'],robot_observed_conflicts=sum(bool(e.get('observed_conflicts')) for e in events),temporary_inspection_goals=sum(e['event']=='inspection_temporary_goal' for e in events),actor_behavior_separate=True))
        behavior[-1]['residual_patch_attempts']=sum(e['event']=='patch_planning_started' for e in events)
        behavior[-1]['source_event_hashes']={episode:sha(ROOT/episode/'client/events.jsonl') for episode in episodes}
        actor=p/'scene/actor_interactions.jsonl'
        if actor.exists():
            records=[json.loads(line) for line in actor.read_text().splitlines()];interactions.append(dict(run=str(p.relative_to(ROOT)),yield_events=sum(bool(e.get('yielding')) for e in records),transitions=len(records),sha256=sha(actor)))
    required={'A1','A2','A3','B1','B2','B3','road_diff','road_ackermann'};budget=read(OUT/'recording_budget.json');video=read(OUT/'media/manifest.json') if (OUT/'media/manifest.json').exists() else None
    checks=dict(all_eight_obligations=keys==required,all_runs_pass=all(e['status']=='PASS' for e in entries),single_declared_plan=len({e['plan_revision'] for e in entries})==1,observed_actor_yield=any(e['yield_events']>0 for e in interactions),wall_budget=budget['consumed_wall_s']<=budget['limit_wall_s'],sim_budget=sum(e['sim_seconds'] for e in entries)<=10800,full_and_short_video=bool(video and all((ROOT/video[k]['path']).exists() and sha(ROOT/video[k]['path'])==video[k]['sha256'] for k in ['full','short'])))
    checks['no_repair_probes_counted']=not any(read(ROOT/e/'spec.json').get('repair_probe',False) for e in all_episodes)
    actor_yield=checks.pop('observed_actor_yield')
    checks['observed_dynamic_interaction']=actor_yield or any(e['robot_observed_conflicts'] or e['temporary_inspection_goals'] for e in behavior)
    checks['restart_chains_verified']=all(e['resume_chain_verified'] for e in entries)
    checks['cluster_cumulative_sim_limit']=all(e['sim_seconds']<=1800 for e in entries if not e['key'].startswith('road_'))
    checks['video_contains_all_restart_episodes']=bool(video and [e['run'] for e in video['episodes']]==all_episodes)
    charged=[];cluster_clock={};unknown_charged=[]
    for episode in budget['episodes']:
        folder=ROOT/episode['run'];metric=folder/'showcase_result.json'
        if not metric.exists():unknown_charged.append(episode['run']);continue
        measured=read(metric);identity=read(folder/'spec.json');seconds=measured['sim_seconds_including_stop'];charged.append(dict(run=episode['run'],sim_seconds=seconds,status=measured['status']))
        if identity['operation']=='work':cluster_clock[identity['cluster']]=cluster_clock.get(identity['cluster'],0.)+seconds
    checks['all_charged_sim_clock_known']=not unknown_charged
    checks['all_charged_sim_budget']=sum(e['sim_seconds'] for e in charged)<=10800
    checks['all_charged_cluster_sim_limit']=all(seconds<=1800 for seconds in cluster_clock.values())
    receipt=dict(schema='INDEPENDENT-SHOWCASE-ACCEPTANCE/1',status='PASS' if all(checks.values()) else 'PARTIAL',scope='one independent demo obligation per row; explicit restart chains retained; not a reliability matrix',checks=checks,runs=entries,dynamic_interactions=interactions,robot_behavior=behavior,budget=budget,selected_sim_total_s=sum(e['sim_seconds'] for e in entries),media_manifest_sha256=sha(OUT/'media/manifest.json') if video else None,s17_status='PARTIAL',s17_formal_completed=0,s17_formal_required=60,s18_executed=False,not_run_obligations=sorted(required-keys),charged_episodes=charged,unknown_charged_episodes=unknown_charged,all_charged_sim_total_s=sum(e['sim_seconds'] for e in charged),charged_work_cluster_sim_s=cluster_clock)
    dump(OUT/'demo_acceptance.json',receipt);print(json.dumps(receipt,ensure_ascii=False,indent=2));return 0 if receipt['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
