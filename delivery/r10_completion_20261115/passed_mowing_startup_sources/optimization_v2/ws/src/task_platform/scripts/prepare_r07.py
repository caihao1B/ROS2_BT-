"""Independent point-service contracts; original formal map and missions immutable."""
from common import *
import copy, math, yaml
import numpy as np
from shapely.geometry import shape, mapping
from shapely import union_all, contains_xy
from PIL import Image

def build():
    require('R06')
    out = edition('R07'); out.mkdir(parents=True, exist_ok=True)
    vehicle = read(edition('R04')/'vehicles/spray.json')
    rig = read(edition('R05')/'spray/sensors.json')
    mission = read(edition('R03')/'mission.json')
    atlas = read(BASE/'maps/s17_full/manifest.json')
    allowed = union_all([shape(f['geometry']) for name in ['workspaces','connector']
        for f in read(BASE/f'maps/prior/{name}.geojson')['features']
        if f['geometry']['type'] in ['Polygon','MultiPolygon']])
    permanent = union_all([shape(f['geometry']) for f in read(BASE/'maps/prior/permanent.geojson')['features']])
    allowed = allowed.difference(permanent)
    cfg = dict(schema='R07-SPRAY/1', task='spray', vehicle=vehicle, rig=rig,
        map_id=atlas['map_id'], map_revision=atlas['revision'], frame_epoch=atlas['frame_epoch'],datum_llh=atlas['datum_llh'],site_origin=atlas['site_origin'],datum_translation_sigma_m=atlas['datum_translation_sigma_m'],
        source_plan_version=mission['version'], map_offset=[28.,14.],
        allowed_prior=mapping(allowed), speed_cap_mps=.4, yaw_cap_rps=.45,terrain_plane_neighborhood_cells=9,ground_observation_filter=dict(frames=3,max_gap_s=.25,max_age_s=.4,strong_hazard_immediate_m=.045),rpp_unknown_policy='reject; observed layer explicitly writes unknown',navigation_goal_xy_m=.05,navigation_goal_orientation_required=False,checked_direct_connection_max_m=4.,direct_path_spacing_m=.05,docking_realign_max_attempts=2,docking_settle_s=1.5,navigation_controller='RPP exact point arrival; DWB full-footprint alternative unqualified',point_path_policy='Compute once; outer BT explicitly cancels and replans on persistent obstruction',map_correction_filter=dict(window_s=1.5,max_samples=7,type='robust_se2_correction_median'),
        motion_observation_age_s=.45, localization_age_s=.6,local_icp_sampling=dict(max_ground_points=300,max_elevated_points=2200,scan_cloud_max_points=0,guess_frame='private_wheel_imu_odom',increment_timestamp='window_midpoint',lag_smoothing_s=2.,wheel_slip_nis=9.21,original_quality_gates=True),
        footprint_margin_m=.1, task_budget_s=1800.,
        scope='Independent point-service task; prior outlines are planning assumptions, live terrain gates motion; original formal obligations untouched',
        near_sensor=dict(horizontal=128,vertical=16,vertical_min=-1.30,vertical_max=-.12,range_m=3.6,units=[dict(name='front',xyz=[.525,0,.25],yaw=0.),dict(name='rear',xyz=[-.525,0,.25],yaw=math.pi),dict(name='left',xyz=[0,.365,.25],yaw=math.pi/2),dict(name='right',xyz=[0,-.365,.25],yaw=-math.pi/2)],noise_m=.01,dropout=.02,delay_s=.04,main_perception_temporal_stride=1,max_pair_skew_s=.12,pose_bracket_s=.08,motion_compensated=True,perception_epoch='front_near_capture',far_returns_optional_unknown_preserved=True),
        clusters=[])
    for cluster in mission['task_plans']['spray']:
        points = [dict(id=s['point_id'], target=[s['target'][0]-28,s['target'][1]-14],
                       yaw=s['poses'][0][2], quantity=s['action_seconds'])
                  for s in cluster['segments'] if s['kind']=='POINT_SERVICE']
        cfg['clusters'].append(dict(id=cluster['cluster'], start=cluster['start'], points=points,
            original_point_count=cluster['point_count'], source_version=cluster['version']))
    cfg['version'] = canonical(cfg); dump(out/'mission.json',cfg)
    # Prior-only global planning. Unknown small hazards are absent from this map;
    # current local terrain is a separate mandatory controller/command gate.
    resolution=.15; lo=np.floor(np.array(allowed.bounds[:2])-2); hi=np.ceil(np.array(allowed.bounds[2:])+2)
    width,height=np.ceil((hi-lo)/resolution).astype(int)
    x=lo[0]+(np.arange(width)+.5)*resolution; y=lo[1]+(np.arange(height)+.5)*resolution
    xx,yy=np.meshgrid(x,y); free=contains_xy(allowed.buffer(-.1),xx,yy)
    Image.fromarray(np.where(free,254,0).astype('uint8')[::-1]).save(out/'prior.pgm')
    (out/'prior.yaml').write_text(yaml.safe_dump(dict(image=str(out/'prior.pgm'),mode='trinary',resolution=resolution,
        origin=[float(lo[0]-28),float(lo[1]-14),0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
    nav=yaml.safe_load((BASE/'config/nav2_diff.yaml').read_text())
    cc=nav['controller_server']['ros__parameters']; cc['controller_plugins']=['RPP','DWB']; cc['DWB']['plugin']='dwb_core::DWBLocalPlanner'; cc['DWB'].pop('implementation',None)
    cc['DWB'].update(max_vel_x=.4,max_speed_xy=.4,max_vel_theta=.45,sim_time=2.,vx_samples=18,vtheta_samples=24,xy_goal_tolerance=.05)
    cc['RPP'].pop('implementation',None);cc['RPP'].update(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',desired_linear_vel=.3,lookahead_dist=.45,min_lookahead_dist=.25,max_lookahead_dist=.7,use_rotate_to_heading=True,rotate_to_heading_angular_vel=.35,rotate_to_heading_min_angle=.45,max_angular_accel=.5,regulated_linear_scaling_min_radius=.6,regulated_linear_scaling_min_speed=.03,approach_velocity_scaling_dist=.6,min_approach_linear_velocity=.02,allow_reversing=False)
    cc['DWB']['critics']=[('ObstacleFootprint' if x=='BaseObstacle' else x) for x in cc['DWB']['critics']];cc['DWB']['ObstacleFootprint']=cc['DWB'].pop('BaseObstacle')
    cc['goal'].update(plugin='nav2_controller::PositionGoalChecker',xy_goal_tolerance=.05,stateful=False);cc['goal'].pop('yaw_goal_tolerance',None); cc['failure_tolerance']=2.
    nav['planner_server']['ros__parameters']['GridBased'].update(plugin='r07::ExactGoalPlanner',tolerance=.04)
    nav['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(out/'navigate.xml')
    (out/'navigate.xml').write_text('<root BTCPP_format="4" main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree"><Sequence><ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" error_code_id="{compute_path_error_code}"/><FollowPath path="{path}" controller_id="RPP" goal_checker_id="goal" error_code_id="{follow_path_error_code}"/></Sequence></BehaviorTree></root>')

    for name in ['local_costmap','global_costmap']:
        p=nav[name][name]['ros__parameters']; p.update(footprint=str(vehicle['footprint']),footprint_padding=.1,resolution=.15,update_frequency=5.,publish_frequency=2.)
        p['inflation'].update(inflation_radius=.75,cost_scaling_factor=4.)
        if name=='local_costmap':p.update(width=12,height=12,track_unknown_space=False);p['observed'].update(topic='/r07/risk_local',max_age=.6)
        else:
            p.update(rolling_window=False,origin_x=float(lo[0]-28),origin_y=float(lo[1]-14),width=int(np.ceil(hi[0]-lo[0])),height=int(np.ceil(hi[1]-lo[1])))
            p['plugins']=['static','inflation'];p.pop('observed');p['static']=dict(plugin='nav2_costmap_2d::StaticLayer',map_subscribe_transient_local=True,map_topic='/map')
    nav['map_server']=dict(ros__parameters=dict(use_sim_time=True,yaml_filename=str(out/'prior.yaml')))
    nav['lifecycle_manager']=dict(ros__parameters=dict(use_sim_time=True,autostart=False,bond_timeout=5.,node_names=['map_server','controller_server','planner_server','bt_navigator']))
    (out/'nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
    terrain=yaml.safe_load((BASE/'config/traversability.yaml').read_text())
    m=rig['self_filter'];terrain.update(lidar_xyz=rig['lidar']['xyz'],self_mask=[m['x_min'],m['x_max'],m['y_min'],m['y_max']],self_z_max=max(m['z_max'],vehicle['tool']['mount_xyz'][2]+vehicle['tool']['collision_size'][2]/2+.05),collision_ceiling_m=1.,qualified_speed_mps=.4)
    terrain['profiles']['diff'].update(footprint=vehicle['footprint'],clearance_m=.13,tool_width_m=.2,vehicle_config_sha256=vehicle['config_hash'])
    dump(out/'terrain.json',terrain)
    manager=yaml.safe_load((BASE/'config/localization_supervisor.yaml').read_text());manager.update(map_id=atlas['map_id'],diagnostic_fixture=False,nominal_speed=.4);dump(out/'manager.json',manager)
    tool=yaml.safe_load((BASE/'config/tool_contract.yaml').read_text()); profile=yaml.safe_load((BASE/'config/tool_profiles.yaml').read_text())['spray']
    profile.update(mount_xyz=vehicle['tool']['mount_xyz'],collision_size=vehicle['tool']['collision_size'],mass_kg=vehicle['tool']['mass_kg'],service_duration_sim_s=3.)
    tool.update(vehicle='diff',tool=profile,map_id=cfg['map_id'],map_revision=cfg['map_revision'],frame_epoch=cfg['frame_epoch'],plan_revision=cfg['version'],scope='R07 abstract spray action with measured simulated motor/flow; no arm or real agronomic effect')
    tool['tool_revision']=canonical(tool);dump(out/'tool.json',tool)
    return out

def build_terrain():
    """Reuse reviewed S14 terrain mathematics with configuration-derived extrinsics."""
    out=APP/'build/r07_source';out.mkdir(parents=True,exist_ok=True)
    src=out/'terrain.cpp';subprocess.run([sys.executable,str(BASE/'ws/src/mission_bt/scripts/terrain_source14.py'),str(src)],check=True)
    s=src.read_text()
    for before,after in [('v+=V(.1,0,.78);','v+=V(cfg_["lidar_xyz"][0],cfg_["lidar_xyz"][1],cfg_["lidar_xyz"][2]);'),('if(s13::selfPoint(v.x(),v.y(),v.z(),self_box_))continue;','if(v.x()>self_box_[0]&&v.x()<self_box_[1]&&v.y()>self_box_[2]&&v.y()<self_box_[3]&&v.z()<val("self_z_max"))continue;'),('closest<1.4?0.:.12','closest<1.4?0.:val("qualified_speed_mps")')]:
        if s.count(before)!=1:raise ValueError('UPSTREAM_TERRAIN_CHANGED')
        s=s.replace(before,after)
    s=s.replace('if(t-last_details_>=.95)rows.push_back', 'if(t-last_details_>=.95 || (s14::riskAt(c.cost[0],c.risk_t[0],t,val("unknown_after_s"))==100 && (o.inverse()*pos).head<2>().norm()<2.))rows.push_back')
    s=s.replace('if(t-last_details_>=.95){last_details_=t;', 'if(t-last_details_>=.95 || closest<1.4){last_details_=t;')
    s=s.replace('dy=-3;dy<=3;dy++','dy=-4;dy<=4;dy++').replace('dx=-3;dx<=3;dx++','dx=-4;dx<=4;dx++')
    s='#include "'+str(PKG/'r07_nav/terrain_observations.hpp')+'"\n'+s
    s=s.replace('struct Cell {double t=', 'struct Cell {std::deque<std::array<double,3>> observations;double t=')
    before='std::map<int,double> h;for(auto&[i,z]:bins){std::sort(z.begin(),z.end());h[i]=z[size_t(.25*(z.size()-1))];}'
    after='std::map<int,double> h,observed_top;for(auto&[i,z]:bins){std::sort(z.begin(),z.end());auto pos=center(i);double f=floor(pos.x(),pos.y());auto obs=r07::groundObservation(cells_[i].observations,t,z[size_t(.25*(z.size()-1))]-f,z[size_t(.8*(z.size()-1))]-f,val("obstacle_min_height_m"));if(obs.qualified){h[i]=obs.ground+f;observed_top[i]=obs.top;}}'
    assert s.count(before)==1;s=s.replace(before,after)
    before='for(auto&[i,z]:bins){auto pos=center(i);double ground=floor(pos.x(),pos.y()),height=z[size_t(.8*(z.size()-1))]-ground;'
    after='for(auto&[i,z]:bins){if(!h.count(i)){auto& c=cells_[i];for(int k=0;k<2;++k)s14::observeRisk(-1,t,cfg_["clear_frames"].get<int>(),c.cost[k],c.clear[k],c.risk_t[k]);continue;}auto pos=center(i);double ground=floor(pos.x(),pos.y()),height=observed_top[i];'
    assert s.count(before)==1;s=s.replace(before,after)
    src.write_text(s)
    supervisor=(PKG/'r06_backend/supervisor.cpp').read_text();assert supervisor.count('||t<latest_stamp;')==1;supervisor=supervisor.replace('||t<latest_stamp;','||t+.03<latest_stamp;');supervisor=supervisor.replace('auto d=p.d;', 'auto d=p.d;if(d.value("reason_code",std::string())=="LOCALIZATION_ADMISSION_WAIT"&&e.value("localized",false)&&!e.value("sensors_healthy",false))d["reason_code"]="PERCEPTION_ADMISSION_WAIT";');(out/'supervisor.cpp').write_text(supervisor)
    cmake='''cmake_minimum_required(VERSION 3.16)
project(r07_terrain)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O2 -Wall -Wextra)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp 4.10.0 EXACT REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
remove_definitions(EIGEN_MPL2_ONLY)
add_compile_definitions(EIGEN_MPL2_ONLY)
add_executable(r07_terrain terrain.cpp)
target_include_directories(r07_terrain PRIVATE "@INCLUDE@" ${PCL_INCLUDE_DIRS})
ament_target_dependencies(r07_terrain rclcpp sensor_msgs nav_msgs std_msgs visualization_msgs pcl_conversions)
target_link_libraries(r07_terrain ${PCL_LIBRARIES} Eigen3::Eigen nlohmann_json::nlohmann_json)
add_executable(r07_supervisor supervisor.cpp)
target_include_directories(r07_supervisor PRIVATE "@BT@")
ament_target_dependencies(r07_supervisor rclcpp std_msgs std_srvs behaviortree_cpp)
target_link_libraries(r07_supervisor nlohmann_json::nlohmann_json)
'''.replace('@INCLUDE@',str(BASE/'ws/src/terrain_perception/include')).replace('@BT@',str(PKG/'r06_backend'))
    (out/'CMakeLists.txt').write_text(cmake)
    subprocess.run(['cmake','-S',str(out),'-B',str(APP/'build/r07')],check=True)
    subprocess.run(['cmake','--build',str(APP/'build/r07'),'--parallel','2'],check=True)

    subprocess.run(['cmake','-S',str(PKG/'r07_nav'),'-B',str(APP/'build/r07_nav'),'-DCMAKE_INSTALL_PREFIX='+str(APP/'build/r07_nav_install')],check=True)
    subprocess.run(['cmake','--build',str(APP/'build/r07_nav'),'--parallel','2'],check=True)
    subprocess.run(['cmake','--install',str(APP/'build/r07_nav')],check=True)
