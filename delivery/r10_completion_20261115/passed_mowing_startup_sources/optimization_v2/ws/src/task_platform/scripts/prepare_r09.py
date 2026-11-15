"""Dino task binding; original three-cluster obligations are retained verbatim."""
from common import *
import copy,math,yaml
from shapely.geometry import LineString,mapping,shape
from shapely.affinity import translate

def build():
 require('R08');out=edition('R09');out.mkdir(parents=True,exist_ok=True);c=copy.deepcopy(read(edition('R08')/'mission.json'));v=read(edition('R04')/'vehicles/mowing.json');rig=read(edition('R05')/'mowing/sensors.json');plan=read(edition('R03')/'mission.json');atlas=read(BASE/'maps/s17_full/manifest.json');c.update(schema='R09-MOWING/1',task='mowing',vehicle=v,rig=rig,tool_capability='ABSTRACT_MOWER_FEEDBACK',speed_cap_mps=.4,reverse_cap_mps=.25,yaw_cap_rps=.14,task_budget_s=1800.,clusters=[],scope='Declared segment subset integrated with real dynamics and tool feedback; all original B1-B3 obligations and formal S17 status preserved')
 for k in ['route_world','route_map','original_route_length_m','point_path_policy','qualification_required','crossing_interlock','detour_validation','dynamic_activity']:c.pop(k,None)
 regions=read(edition('R03')/'B/partition/clusters.geojson')['features']
 for source in plan['task_plans']['mowing']:
  region=translate(shape(regions[int(source['cluster'][1:])-1]['geometry']),-28,-14);cluster=copy.deepcopy(source);cluster['area_map']=mapping(region);cluster['original_version']=cluster.pop('version');segments=[]
  for i,s in enumerate(cluster['segments']):
   s['id']=f"{cluster['cluster']}:segment:{i}";s['direction']=s.get('direction',1);s['poses_world']=s.pop('poses');s['poses']=[[x-28,y-14,h] for x,y,h in s['poses_world']]
   if s['kind']=='COVERAGE':
    # Requested band is the actual planned work strip. Pose erosion is applied
    # only by feedback ledger, never by shrinking the requested denominator.
    centers=[[x+v['tool']['mount_xyz'][0]*math.cos(h),y+v['tool']['mount_xyz'][0]*math.sin(h)] for x,y,h in s['poses']];area=LineString(centers).buffer(v['tool']['effective_width_m']/2,cap_style=2)
    area=area.intersection(region)
    if area.geom_type!='Polygon' or len(area.interiors):raise ValueError('WORK_STRIP_NEEDS_EXPLICIT_COMPONENT_SPLIT')
    s['target_area']=list(area.exterior.coords)[:-1];s['requested_area_m2']=area.area
   segments.append(s)
  cluster['segments']=segments;c['clusters'].append(cluster)
 c['near_sensor'].update(vertical=128,range_m=4.5,units=[dict(name='front',xyz=[2.214,0,.23],pitch=.25,yaw=0.),dict(name='rear',xyz=[-.576,0,.23],pitch=.25,yaw=math.pi),dict(name='left',xyz=[.819,1.390,.23],pitch=.25,yaw=math.pi/2),dict(name='right',xyz=[.819,-1.390,.23],pitch=.25,yaw=-math.pi/2)])
 c['terrain_self_mask_margin_m']=.05;c['near_sensor']['sampling_reason']='Four physical 128 x 128 low-mounted 0.23 m channels just beyond body edges; 0.05 m self-return uncertainty mask; unknown clearance remains blocked'
 c['startup_stability'].update(basis='robust map-to-odom corrections',min_samples=18,min_span_s=3.5,continuous_stable_s=3.,admission_window_s=7.,max_admission_gap_s=.6,correction_filter_window_s=4.2,correction_filter_max_samples=25)
 c['navigation_goal_xy_m']=.05;c['navigation_goal_yaw_rad']=.08;c['mowing_protocol']=dict(stop_at_every_segment=True,tool_off_for_reverse_and_turn=True,completion='measured segment arrival; coverage area separately feedback-confirmed',default_segment_limit=5,original_denominator_preserved=True)
 from rig_r08 import effective_rig
 c['rig']=effective_rig(rig,c['near_sensor'])
 c.pop('version',None);c['version']=canonical(c);dump(out/'mission.json',c)
 nav=yaml.safe_load((edition('R08')/'nav2.yaml').read_text());nav['global_costmap']['global_costmap']['ros__parameters']['live_obstacles']['topic']='/r07/risk_local';nav['local_costmap']['local_costmap']['ros__parameters']['plugins']=[x for x in nav['local_costmap']['local_costmap']['ros__parameters']['plugins'] if x!='dynamic_history'];nav['local_costmap']['local_costmap']['ros__parameters'].pop('dynamic_history',None);cc=nav['controller_server']['ros__parameters'];cc['RPP'].update(desired_linear_vel=.35,lookahead_dist=.8,min_lookahead_dist=.35,max_lookahead_dist=1.2,use_fixed_curvature_lookahead=True,curvature_lookahead_dist=.8,interpolate_curvature_after_goal=True,use_rotate_to_heading=False,allow_reversing=True,regulated_linear_scaling_min_radius=3.0983388881649963,regulated_linear_scaling_min_speed=.04,approach_velocity_scaling_dist=.5,min_approach_linear_velocity=.02);cc['goal']=dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=.05,yaw_goal_tolerance=.08,stateful=False)
 for name in ['local_costmap','global_costmap']:
  p=nav[name][name]['ros__parameters'];p['footprint']=str(v['footprint']);p['inflation'].update(inflation_radius=1.8,cost_scaling_factor=4.)
 nav['planner_server']['ros__parameters']['GridBased']=dict(plugin='nav2_smac_planner::SmacPlannerHybrid',tolerance=.1,downsample_costmap=False,allow_unknown=False,max_iterations=100000,max_on_approach_iterations=1000,max_planning_time=3.0,motion_model_for_search='REEDS_SHEPP',angle_quantization_bins=72,analytic_expansion_ratio=3.5,analytic_expansion_max_length=12.0,minimum_turning_radius=3.0983388881649963,reverse_penalty=2.,change_penalty=.2,non_straight_penalty=1.2,cost_penalty=2.,retrospective_penalty=.015,lookup_table_size=20.,cache_obstacle_heuristic=False,smooth_path=True)
 (out/'nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
 terrain=read(edition('R08')/'terrain.json');m=rig['self_filter'];terrain.update(lidar_xyz=rig['lidar']['xyz'],self_mask=[min(x for x,y in v['footprint'])-.05,max(x for x,y in v['footprint'])+.05,min(y for x,y in v['footprint'])-.05,max(y for x,y in v['footprint'])+.05],self_z_max=m['z_max'],collision_ceiling_m=1.2,qualified_speed_mps=.4,protective_half_width_m=1.485,protective_front_m=2.609)
 terrain['self_uncertainty_margin_m']=.05
 terrain['profiles']['ackermann'].update(footprint=v['footprint'],clearance_m=.10,tool_width_m=1.6,vehicle_config_sha256=v['config_hash']);dump(out/'terrain.json',terrain);dump(out/'manager.json',read(edition('R08')/'manager.json'))
 tool=yaml.safe_load((BASE/'config/tool_contract.yaml').read_text());profile=yaml.safe_load((BASE/'config/tool_profiles.yaml').read_text())['mower'];profile.update(mount_xyz=v['tool']['mount_xyz'],collision_size=v['tool']['collision_size'],mass_kg=v['tool']['mass_kg'],virtual_effective_width_m=v['tool']['effective_width_m'],virtual_effective_length_m=.70)
 tool.update(feedback_period_s=.02,vehicle='ackermann',tool=profile,map_id=c['map_id'],map_revision=c['map_revision'],frame_epoch=c['frame_epoch'],plan_revision=c['version'],work_max_speed=.4,scope='Actual simulated rotor feedback and conservative estimated footprint union; no cutting effectiveness claim; R04 qualified .5 m/s bounds .35 m/s command and .4 m/s observed speed limit')
 tool['tool_revision']=canonical(tool);dump(out/'tool.json',tool);return out
