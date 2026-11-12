"""Only the simulator knows the declared cluster deployment position."""
from common14 import *
from scene13 import compose as compose13
from xml.etree import ElementTree as E

def compose(out, kind, case, session):
    compose13(out, kind, 'navigation_only', session)
    p = cluster(kind)
    start = [30.5,15.3,.15] if case in ['decision_story','mow_smoke','tool_loss','persistent_block','budget_exhaustion'] else p['start']
    if case.startswith('calibration_station_'):
        start=read(BASE/'config/s14_survey_stations.json')['stations'][case.removeprefix('calibration_station_')]['prior_pose']
    if case=='spray_registered_docking_probe':start=p['segments'][47]['poses'][0]
    if case=='ackermann_navigation_probe':start=p['segments'][4]['poses'][0]
    if case=='ackermann_registered_route_probe':
        from binding14 import execution_cluster
        start=max((s for s in execution_cluster(kind)['segments'] if not s['tool_on']),key=lambda s:s['timing']['motion_seconds'])['poses'][0]
    root = E.parse(out / 'world.sdf')
    world = root.getroot().find('world')
    world.find('physics/real_time_factor').text = '0.4'
    world.find("model[@name='vehicle']/pose").text = f'{start[0]} {start[1]} .025 0 0 {start[2]}'
    if case=='regional_probe_low_obstacle':
        x=start[0]+1.1*math.cos(start[2]);y=start[1]+1.1*math.sin(start[2])
        model=E.fromstring(f'<model name="s14_observed_low_obstacle"><static>true</static><pose>{x} {y} .03 0 0 {start[2]}</pose><link name="body"><collision name="collision"><geometry><box><size>.25 .35 .06</size></box></geometry></collision><visual name="visual"><geometry><box><size>.25 .35 .06</size></box></geometry><material><diffuse>.7 .25 .1 1</diffuse></material></visual></link></model>')
        world.append(model)
    if os.environ.get('S14_REGISTERED_REGION'):
        m=plan();c=read(out/'tool.json');c.update(plan_revision=m['plan_revision'],map_id=m['frame']['map_id'],map_revision=m['frame']['map_revision'],frame_epoch=m['frame']['frame_epoch']);dump(out/'tool.json',c)
        world.find("model[@name='vehicle']/plugin[@name='s10::ToolActuator']/config").text=json.dumps(c)
    root.write(out / 'world.sdf', encoding='unicode')
    dump(out / 'deployment.json', dict(source_plan=plan()['plan_revision'],
         zone=p['zone'], cluster=p['cluster'], prior_start=start,
         purpose='simulation deployment, never an online localization observation'))
    return [start[0] - 28 - .5, start[1] - 14 + .5, start[2] + .1]
