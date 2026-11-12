"""Public curved connector, smooth interpolation, explicit parking approaches."""
from scipy.interpolate import CubicSpline
from common12 import *
from motion12 import *
from mowing12 import connection
from spray12 import visibility

def road_geometry(c):
    fs=read(BASE/'maps/prior/connector.geojson')['features'];raw=shape(next(f['geometry'] for f in fs if f['properties']['id']=='centerline'));xy=[]
    for p in raw.coords:
        if not xy or math.dist(xy[-1],p)>1e-8:xy.append(p)
    p=np.asarray(xy);length=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))];spline=CubicSpline(length,p,axis=0);s=np.linspace(0,length[-1],math.ceil(length[-1]/c['path_step_m'])+1);q=spline(s);d=spline(s,1);poses=np.c_[q,np.arctan2(d[:,1],d[:,0])].tolist()
    if LineString(q).hausdorff_distance(raw)>.1:raise ValueError('CONNECTOR_INTERPOLATION_DEVIATION')
    work=[shape(f['geometry']) for f in read(BASE/'maps/prior/workspaces.geojson')['features']];road=shape(next(f['geometry'] for f in fs if f['properties']['id']=='road'));obstacles=union_all([shape(f['geometry']) for f in read(BASE/'maps/prior/permanent.geojson')['features']]);motion=union_all(work+[road]).difference(obstacles)
    return poses,motion,float(LineString(q).hausdorff_distance(raw))

def approach(start,goal,vehicle,motion,c,out,name):
    if vehicle=='diff':
        cost,paths,_=visibility([start[:2],goal[:2]],motion,c);xy=paths[0,1];current=start;segments=[]
        for a,b in zip(xy,xy[1:]):
            if math.dist(a,b)<1e-8:continue
            heading=math.atan2(b[1]-a[1],b[0]-a[0]);segments.append(rotate_at(current,heading,motion,c));s=segment(straight(a,b,c['path_step_m']),'TRANSIT',vehicle,motion,c);segments.append(s);current=s['poses'][-1]
        segments.append(rotate_at(current,goal[2],motion,c));return segments
    candidates=connection(start,[dict(start=goal)],motion,c,out,name)
    if candidates:return [min(candidates,key=lambda x:x[0])[2]]
    for path in dubins.dubins(start,goal,2.11,c['path_step_m']/2):
        s=segment(path['poses'],'TRANSIT',vehicle,motion,c,stop_curvature=True,connector_source='existing analytic forward Dubins alternatives')
        if s['validation']['passed']:return [s]
    raise ValueError('NO_COLLISION_FREE_APPROACH '+name)

def plan_connector(selected,c,out):
    path,motion,deviation=road_geometry(c);result=[]
    for vehicle in ['diff','ackermann']:
        dock_a=selected['A']['plans'][0]['start'] if vehicle=='diff' else [34.,47.,0.]
        dock_b=selected['B']['plans'][0]['start'] if vehicle=='ackermann' else [128.,58.,0.]
        for direction in ['A_to_B','B_to_A']:
            poses=path if direction=='A_to_B' else [[p[0],p[1],wrap(p[2]+math.pi)] for p in reversed(path)]
            start,goal=(dock_a,dock_b) if direction=='A_to_B' else (dock_b,dock_a)
            ss=approach(start,poses[0],vehicle,motion,c,out,vehicle+direction+'_entry')
            ss.append(segment(poses,'CONNECTOR',vehicle,motion,c));ss+=approach(poses[-1],goal,vehicle,motion,c,out,vehicle+direction+'_exit')
            timing=summarize(ss,c);connected=all(math.dist(a['poses'][-1][:2],b['poses'][0][:2])<.002 and abs(wrap(a['poses'][-1][2]-b['poses'][0][2]))<.002 for a,b in zip(ss,ss[1:]))
            result.append(dict(vehicle=vehicle,direction=direction,segments=ss,start=start,end=goal,timing=timing,passed=timing['all_segments_valid'] and connected and timing['total_s']<c['hard_budget_s'],centerline_deviation_m=deviation,motion_domain=mapping(motion),work_credited=False))
    return result
