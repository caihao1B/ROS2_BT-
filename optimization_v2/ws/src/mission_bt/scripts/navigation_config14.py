"""Vehicle-specific actual Nav2 plugin/BT selection, isolated from S13 config."""
from common14 import *

def compose(kind,out):
    c=yaml.safe_load((BASE/f'config/nav2_{kind}.yaml').read_text())
    if os.environ.get('S14_REGISTERED_REGION'):
        from binding14 import region
        _,reg=region();lo,hi=np.asarray(reg['site_extent_xy'])
        origin=np.floor(lo);size=np.ceil(hi)-origin
        # The sensor/prior layer already publishes a 30m grid. A 20m rolling
        # master cropped valid forward-only turns whenever the robot moved.
        # Keep the global master fixed on the qualified region; the existing
        # layer still refuses unknown, stale data, and unqualified boundaries.
        c['global_costmap']['global_costmap']['ros__parameters'].update(
            rolling_window=False,origin_x=float(origin[0]),origin_y=float(origin[1]),
            width=int(size[0]),height=int(size[1]))
    params=c['controller_server']['ros__parameters']
    if kind=='diff':
        params['goal']['xy_goal_tolerance']=.025;params['goal']['yaw_goal_tolerance']=.025;params['DWB']['xy_goal_tolerance']=.025
    else:
        if 'RPP' not in params['controller_plugins']:raise ValueError('ACKERMANN_RPP_NOT_LOADED')
        p=c['planner_server']['ros__parameters']['GridBased']
        p.update(plugin='s14::AckermannPlanner',tolerance=.04,motion_model_for_search='DUBIN',minimum_turning_radius=2.11,angle_quantization_bins=72,allow_unknown=False,smooth_path=False,analytic_expansion_max_length=10.55,lookup_table_size=20.)
        tree=replace_once((NAV/'config/navigate.xml').read_text(),'controller_id="DWB"','controller_id="RPP"')
        path=out/'navigate_ackermann.xml';path.write_text(tree)
        c['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(path)
    path=out/'registered_nav2.yaml';path.write_text(yaml.safe_dump(c,sort_keys=False));return path
