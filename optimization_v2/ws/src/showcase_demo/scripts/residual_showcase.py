"""Bounded analytic detours checked against observed occupancy and full body.

The task polygon and work lanes remain fixed. A valid reference still requires
the existing live Nav2 collision checks, localization and tool admission.
"""
from common14 import *
from common12 import config,dubins
from routing14 import CollisionMap
from motion12 import segment,effect_polygon
from shapely.geometry import shape
from shapely.affinity import translate
from scipy.ndimage import distance_transform_edt
import yaml

def capture_grid(m,request,directory):
    grid=m.st.get('global_map')
    if grid is None:raise ValueError('RESIDUAL_OBSERVATION_MISSING')
    stamp=grid.header.stamp.sec+grid.header.stamp.nanosec*1e-9
    if grid.header.frame_id!='map' or not 0<=m.st['t']-stamp<.6 or not m.platform_ok:raise ValueError('RESIDUAL_OBSERVATION_NOT_ADMITTED')
    data=np.asarray(grid.data,np.int16).reshape(grid.info.height,grid.info.width).copy()
    data[data>=90]=100  # Conservative: do not turn high observed costs into free cells.
    path=directory/f'observed_{m.patches}.npz'
    np.savez_compressed(path,data=data,resolution=grid.info.resolution,origin=[grid.info.origin.position.x,grid.info.origin.position.y],frame='map')
    request.update(observed_grid=str(path),observed_grid_sha256=sha(path),observed_t=stamp)
    return request

def candidate(request):
    path=Path(request['observed_grid'])
    if sha(path)!=request['observed_grid_sha256']:raise ValueError('RESIDUAL_GRID_CHANGED')
    grid=np.load(path);collision=CollisionMap(path,[[-1000,-1000],[1000,1000]])
    c=config();cluster=request['cluster'];residual=shape(request['residual'])
    target=translate(shape(cluster['coverage_check']['covered']),xoff=-28,yoff=-14)
    tc=yaml.safe_load((BASE/'config/tool_contract.yaml').read_text())
    # Score only footprints the unchanged conservative ledger could accept.
    # The actual required-area denominator remains the whole original polygon.
    recoverable=residual.intersection(target.buffer(-tc['ledger_precision_m']))
    options=[]
    for work in cluster['segments']:
        if work['kind'] not in ['COVERAGE','HEADLAND_WORK']:continue
        poses=np.array(work['poses']);poses[:,:2]-=[28,14]
        gain=effect_polygon(poses,c).intersection(recoverable).area
        if gain>.005 and collision.valid(poses):options.append(dict(work=work,goal=poses[0].tolist(),gain=gain))
    if not options:return None
    start=request['pose_map'];begin=time.monotonic();tested=0;chosen=None
    def edge(a,b):
        for p in dubins.dubins(a,b,2.11,c['path_step_m']/2):
            if collision.valid(p['poses']):return p
        return None
    for option in sorted(options,key=lambda o:-o['gain']):
        direct=edge(start,option['goal'])
        if direct:chosen=(direct,option,None);break
    if chosen is None:
        a=grid['data'];origin=grid['origin'];res=float(grid['resolution']);clearance=distance_transform_edt((a>=0)&(a<90))*res;nodes=[]
        for x in np.arange(origin[0]+.5,origin[0]+a.shape[1]*res,1.):
            for y in np.arange(origin[1]+.5,origin[1]+a.shape[0]*res,1.):
                i,j=np.floor((np.array([x,y])-origin)/res).astype(int)
                if not (0<=i<a.shape[1] and 0<=j<a.shape[0]) or clearance[j,i]<1.:continue
                for yaw in np.arange(8)*math.pi/4:
                    via=[x,y,yaw]
                    for option in options:
                        proxy=(math.dist(start[:2],via[:2])+math.dist(option['goal'][:2],via[:2]))/option['gain'];nodes.append((proxy,via,option))
        for _,via,option in sorted(nodes,key=lambda row:row[0]):
            if time.monotonic()-begin>30:break
            if not collision.valid([via,via]):continue
            first=edge(start,via);tested+=1
            if first is None:continue
            second=edge(via,option['goal'])
            if second is not None:
                joined=dict(poses=first['poses']+second['poses'][1:],length_m=first['length_m']+second['length_m'])
                if collision.valid(joined['poses']):chosen=(joined,option,via);break
    if chosen is None:return None
    route,option,via=chosen;poses=np.asarray(route['poses']);poses[:,:2]+=[28,14]
    motion=shape(plan()['zones']['B']['motion_domain'])
    turn=segment(poses.tolist(),'TURN','ackermann',motion,c,stop_curvature=True,connector_source='bounded observed-grid analytic detour')
    if not turn['validation']['passed']:return None
    turn['showcase_measured_reference']=True
    certificate=dict(observed_grid_sha256=request['observed_grid_sha256'],observed_t=request['observed_t'],whole_padded_body_valid=True,unknown_forbidden=True,occupancy_cutoff=90,minimum_radius_m=2.11,via_map=via,tested_nodes=tested,planning_wall_s=time.monotonic()-begin,length_m=route['length_m'],live_controller_collision_check_required=True)
    return dict(segments=[turn,option['work']],cost_s=turn['timing']['motion_seconds']+option['work']['timing']['motion_seconds']+8.,expected_gain_m2=option['gain'],basis='confirmed residual; fixed work lane; bounded analytic detour on observed occupancy',certificate=certificate)

def main():
    request=read(Path(sys.argv[1]));current=plan()
    if current['plan_revision']!=request['plan_revision']:raise ValueError('PATCH_PLAN_VERSION_CHANGED')
    result=candidate(request)
    dump(Path(sys.argv[2]),dict(plan_revision=current['plan_revision'],candidate=result))

if __name__=='__main__':main()
