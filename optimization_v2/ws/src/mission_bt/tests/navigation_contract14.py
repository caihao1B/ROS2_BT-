"""Ackermann action/plugin consistency and unchanged work obligations/endpoints."""
import sys,tempfile,argparse
from pathlib import Path
from xml.etree import ElementTree as E
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
from navigation_config14 import compose
from mission14 import Mission

with tempfile.TemporaryDirectory() as directory:
    out=Path(directory);c=yaml.safe_load(compose('ackermann',out).read_text())
    tree=E.parse(c['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'])
    names={x.attrib['controller_id'] for x in tree.iter('FollowPath')}
    assert names=={'RPP'} and names<=set(c['controller_server']['ros__parameters']['controller_plugins'])
    p=c['planner_server']['ros__parameters']['GridBased']
    assert p['motion_model_for_search']=='DUBIN' and p['minimum_turning_radius']==2.11 and not p['allow_unknown']
    assert p['plugin']=='s14::AckermannPlanner'
    m=Mission.__new__(Mission);m.a=argparse.Namespace(case='nominal_mow_cluster',vehicle='ackermann');m.p=cluster('ackermann')
    jobs=m.make_jobs()
    for i,s in enumerate(m.p['segments']):
        matching=[j for j in jobs if j['id']==f'B1_{i}' or j['id'].startswith(f'B1_{i}_')]
        assert matching
        if s['kind'] in ['COVERAGE','HEADLAND_WORK']:
            expected=m.segment_jobs(s,f'B1_{i}');assert matching==expected
        else:
            assert len(matching)==1 and matching[0]['poses'][-1]==[s['poses'][-1][0]-28,s['poses'][-1][1]-14,s['poses'][-1][2]]
    # A missing work-entry connection must fail before tool activation.
    m.api={'revalidate':lambda:None,'xy':lambda:[-100.,-100.]}
    m.start_tool=lambda _:(_ for _ in ()).throw(AssertionError('TOOL_MUST_STAY_OFF'))
    try:m.execute_job(next(j for j in jobs if j['kind']=='COVERAGE'))
    except RuntimeError as e:assert str(e)=='WORK_ENTRY_NOT_REACHED_TOOL_REMAINS_OFF'
    else:raise AssertionError('Work started away from its required entry')
    from binding14 import execution_cluster
    os.environ['S14_REGISTERED_REGION']='B'
    try:
        registered=yaml.safe_load(compose('ackermann',out).read_text())
        global_grid=registered['global_costmap']['global_costmap']['ros__parameters']
        assert not global_grid['rolling_window'] and registered['local_costmap']['local_costmap']['ros__parameters']['rolling_window']
        m.p=cluster('ackermann');execution=execution_cluster('ackermann');jobs=m.make_jobs()
        from scipy.spatial import ConvexHull
        from scipy.spatial.distance import cdist
        from common12 import body_local
        route=np.vstack([s['poses'] for s in execution['segments']]);fp=body_local('ackermann')[0];radius=np.linalg.norm(fp,axis=1).max()
        # Every required footprint stays in the 30m upstream grid even at the
        # opposite end of the route. This does not certify unseen free space.
        co=np.cos(route[:,2,None]);si=np.sin(route[:,2,None]);body=np.stack([route[:,0,None]+co*fp[:,0]-si*fp[:,1],route[:,1,None]+si*fp[:,0]+co*fp[:,1]],axis=-1).reshape(-1,2)
        # Exact convex-hull diameter to the entire swept polygon, plus 1m for
        # raster alignment and tracking deviation (not sampled center radius).
        assert cdist(route[ConvexHull(route[:,:2]).vertices,:2],body[ConvexHull(body).vertices]).max()+1 < 15
        assert np.all(route[:,:2]-[28,14]-radius>[global_grid['origin_x'],global_grid['origin_y']])
        assert np.all(route[:,:2]-[28,14]+radius<np.array([global_grid['origin_x']+global_grid['width'],global_grid['origin_y']+global_grid['height']]))
        assert len(execution['segments'])!=len(m.p['segments'])
        for i,s in enumerate(execution['segments']):
            expected=m.segment_jobs(s,f'B1_{i}') if s['tool_on'] else [m.transit_job(s,f'B1_{i}',reference=True)]
            assert [j for j in jobs if j['id']==f'B1_{i}' or j['id'].startswith(f'B1_{i}_')]==expected
        try:m.execute_job(next(j for j in jobs if j['kind'] in ['COVERAGE','HEADLAND_WORK']))
        except RuntimeError as e:assert str(e)=='WORK_ENTRY_NOT_REACHED_TOOL_REMAINS_OFF'
        else:raise AssertionError('Registered work started away from its required entry')
        m.a.case='ackermann_registered_route_probe';probe=m.make_jobs()
        assert len(probe)==1 and probe[0]['kind'] not in ['COVERAGE','HEADLAND_WORK']
        calls=[];steering=[]
        m.api.update(log=lambda *a,**kw:None,align=lambda k:steering.append(k),follow=lambda label,poses,nav=False:(calls.append((label,poses,nav)) or dict(status=4)))
        m.execute_job(probe[0]);assert calls and all(not c[2] for c in calls) and len(calls)==len(steering)
        assert calls[-1][1][-1]==probe[0]['poses'][-1] and all(abs(k)<=1/2.11 for k in steering)
        calls.clear();m.execute_job(probe[0],replan=True);assert len(calls)==1 and calls[0][2] and calls[0][1][-1]==probe[0]['poses'][-1]
    finally:os.environ.pop('S14_REGISTERED_REGION',None)
print('ACKERMANN_BT_PLUGINS_MATCH; DUBINS_2_11M; ORIGINAL_AND_REGISTERED_JOBS_MATCH; TOOL_OFF_UNTIL_ENTRY')
