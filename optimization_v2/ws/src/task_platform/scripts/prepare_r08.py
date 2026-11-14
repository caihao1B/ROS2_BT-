"""Independent inspection contract; R07 remains immutable."""
from common import *
import copy,math,yaml,shutil

def build():
 require('R07');out=edition('R08');out.mkdir(parents=True,exist_ok=True);cfg=copy.deepcopy(read(edition('R07')/'mission.json'));v=read(edition('R04')/'vehicles/inspection.json');rig=read(edition('R05')/'inspection/sensors.json');route=read(edition('R03')/'mission.json')['task_plans']['inspection'][0]
 poses=route['segments'][0]['poses'];length=sum(math.dist(a[:2],b[:2]) for a,b in zip(poses,poses[1:]));cfg.update(schema='R08-INSPECTION/1',task='inspection',vehicle=v,rig=rig,clusters=[],route_world=poses,route_map=[[x-28,y-14,h] for x,y,h in poses],original_route_length_m=length,tool_capability='ABSENT',startup_stability=dict(min_samples=5,min_span_s=.9,max_translation_spread_m=.06,max_yaw_spread_rad=.01,continuous_stable_s=1.,reject_prehint_capture=True),task_budget_s=1800.,speed_cap_mps=.4,navigation_controller='Native RPP / FollowPath',navigation_goal_xy_m=.12,point_path_policy='Follow original connector; cancel and await a fresh BT context before bounded replanning',current_unknown_clearance_policy='physical footprint only; padding is never treated as occupied',scope='Original complete connector retained; selected prefixes and leader fixture separately declared; no tool or work-credit publisher',qualification_required=['complete_original_connector','observed_leader_follow_stop_resume','observed_detour_or_bounded_manual_hold'])
 cfg['localization_surface_window']=dict(processing_range_m=48.,physical_sensor_range_m=50.,max_age_s=1.5,unique_voxel_m=.06,source_voxel_m=.08,motion_compensation='observed SE3 local odometry; no stale or duplicate rays')
 cfg['local_motion_integrity']=dict(reference='independent GNSS antenna displacement at matched epochs',nis_limit=9.21,local_pose_fusion=False,unknown_motion='reject ICP increment; no inferred wheel-slip authority')
 cfg['global_reference_checks']=dict(jump_anchor='current robot pose',gnss_comparison='measured antenna at its captured odometry epoch',planar_query='nearest eligible normal; held-out mapped landmark candidates; raw unmodeled objects retained by perception')
 cfg['detour_validation']='Current full-footprint executable prefix, live Nav2 collision checking, remaining prior candidate not certified'
 cfg['crossing_interlock']=dict(horizon_s=2.,model='current CV with propagated future acceleration covariance, not learned intent',acceleration_spectral_density_m2_s3=.8,motion_nis=9.21,observed_motion_memory_s=10.,ego_hypotheses=['stationary','bounded_next_motion'],independent_of_fixture=True)
 cfg['dynamic_activity']=dict(history_s=35.,margin_m=.25,lookahead_m=1.5,source='observed crossing tracks only',prior_clearance=False)
 cfg['ground_statistic']='Raw-ray weighted quantiles; confidence-separated spatial modes; strong voxel hazards retained'
 cfg['near_sensor']['vertical']=64;cfg['near_sensor']['vertical_min']=-1.3;cfg['terrain_self_mask_margin_m']=.01;cfg['near_sensor']['vertical_max']=.13
 cfg['near_sensor']['sampling_reason']='Boundary mounts at 0.20 m, calibrated 0.25 rad downward pitch and supported internal ray angles observe rear/side clearance cells previously shadowed by the 0.05 m padded self filter; no unknown clearance exemption'
 cfg['near_sensor']['units']=[dict(name='front',xyz=[.259,0,.20],pitch=.25,yaw=0.),dict(name='rear',xyz=[-.259,0,.20],pitch=.25,yaw=math.pi),dict(name='left',xyz=[0,.220,.20],pitch=.25,yaw=math.pi/2),dict(name='right',xyz=[0,-.220,.20],pitch=.25,yaw=-math.pi/2)]
 from rig_r08 import effective_rig
 cfg['rig']=effective_rig(rig,cfg['near_sensor'])
 cfg.pop('version',None);cfg['version']=canonical(cfg);dump(out/'mission.json',cfg)
 nav=yaml.safe_load((edition('R07')/'nav2.yaml').read_text());cc=nav['controller_server']['ros__parameters'];cc['controller_plugins']=['RPP'];cc['RPP'].update(desired_linear_vel=.4,lookahead_dist=.65,min_lookahead_dist=.35,max_lookahead_dist=.9);cc['goal'].update(xy_goal_tolerance=.12,stateful=False)
 for name in ['local_costmap','global_costmap']:nav[name][name]['ros__parameters']['footprint']=str(v['footprint'])
 nav['global_costmap']['global_costmap']['ros__parameters']['plugins']=['static','live_obstacles','inflation']
 nav['global_costmap']['global_costmap']['ros__parameters']['live_obstacles']=dict(plugin='r08::ObservedPriorLayer',topic='/r08/dynamic_risk',max_age=.6)
 nav['local_costmap']['local_costmap']['ros__parameters']['plugins'].append('dynamic_history')
 nav['local_costmap']['local_costmap']['ros__parameters']['dynamic_history']=dict(plugin='r08::ObservedPriorLayer',topic='/r08/dynamic_risk',max_age=.6)
 nav['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(edition('R07')/'navigate.xml');(out/'nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
 terrain=read(edition('R07')/'terrain.json');m=rig['self_filter'];terrain.update(lidar_xyz=rig['lidar']['xyz'],self_mask=[min(x for x,y in v['footprint'])-.01,max(x for x,y in v['footprint'])+.01,min(y for x,y in v['footprint'])-.01,max(y for x,y in v['footprint'])+.01],self_z_max=m['z_max']);terrain.update(protective_half_width_m=max(abs(y) for x,y in v['footprint'])+.1,protective_front_m=max(1.4,max(x for x,y in v['footprint'])+.4))
 terrain['profiles']['diff'].update(footprint=v['footprint'],clearance_m=.065,tool_width_m=0.,vehicle_config_sha256=v['config_hash']);dump(out/'terrain.json',terrain);dump(out/'manager.json',read(edition('R07')/'manager.json'));return out
