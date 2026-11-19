"""Compare read-only Nav2 private-map responses with the measured terrain at the same positions."""
import argparse
from array import array
from bisect import bisect_left
import base64
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import statistics
from types import SimpleNamespace as NS
import zlib

from collision_observer import perimeter_cells, cell_at


def grid(data):
    values=array('b')
    values.frombytes(zlib.decompress(base64.b64decode(data['int8_zlib_base64'])))
    return NS(data=values,info=NS(width=data['size'],height=data['size'],resolution=data['resolution'],
                                  origin=NS(position=NS(x=data['origin'][0],y=data['origin'][1]))))


def body(pose,x,y):
    dx,dy=x-pose[0],y-pose[1]
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return c*dx+s*dy,-s*dx+c*dy


def area(x,y,xlim,ylim):
    if abs(x)/xlim>=abs(y)/ylim:
        return 'FRONT' if x>=0. else 'REAR'
    return 'LEFT' if y>=0. else 'RIGHT'


def cost_group(cost):
    if cost is None:
        return 'OUTSIDE_CROP'
    if cost<0:
        return 'UNKNOWN'
    if cost>=100:
        return 'LETHAL'
    if cost==0:
        return 'ZERO_COST'
    return 'NONZERO_TRAVERSABLE'


def read_rows(path):
    with path.open() as f:
        return [json.loads(line) for line in f if line.endswith('\n')]


def digest(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def ideal_ground_returns(config,terrain,lowest):
    import numpy as np
    horizontal,vertical=np.meshgrid(np.linspace(-math.pi/2,math.pi/2,config['horizontal']),
                                    np.linspace(lowest,config['vertical_max'],config['vertical']))
    azimuth=horizontal.ravel();elevation=vertical.ravel()
    parts=[]
    for mount in config['units']:
        if mount.get('pitch',0.)!=0.:
            raise ValueError('This ideal-flat projection is scoped to the original unpitched spray rig')
        sx,sy,sz=mount['xyz'];h=mount['yaw']
        ray_length=-sz/np.sin(elevation);radial=ray_length*np.cos(elevation)
        valid=(ray_length>=.03)&(ray_length<=config['range_m'])
        parts.append(np.c_[sx+radial[valid]*np.cos(azimuth[valid]+h),sy+radial[valid]*np.sin(azimuth[valid]+h)])
    pts=np.concatenate(parts)
    left,right,down,up=terrain['self_mask']
    return pts[~((pts[:,0]>left)&(pts[:,0]<right)&(pts[:,1]>down)&(pts[:,1]<up))]


def projected_counts(rays,pose,raw_grid,locations):
    import numpy as np
    x0,y0=raw_grid.info.origin.position.x,raw_grid.info.origin.position.y
    res=raw_grid.info.resolution;h=pose[2];c,s=math.cos(h),math.sin(h)
    world=rays@np.array([[c,s],[-s,c]])+np.array(pose[:2])
    offsets=np.floor((world-np.array([x0,y0]))/res).astype(int)
    # The 3.6 m range cannot cross more than 48 terrain cells on either axis.
    by_index,count=np.unique(1000*offsets[:,1]+offsets[:,0],return_counts=True)
    counts=dict(zip(by_index,count))
    return [int(counts.get(1000*math.floor((y-y0)/res)+math.floor((x-x0)/res),0)) for x,y in locations]


def analyze(observer,run):
    spec=json.loads((run/'spec.json').read_text())
    mission=json.loads((run/'mission.json').read_text())
    footprint=mission['vehicle']['footprint']
    # The bound local_nav2.yaml for this deployment uses .10 m independent edge padding.
    import yaml
    root=Path(__file__).resolve().parents[3]
    import re
    candidates=[root/p for p in spec['implementation_bindings'] if re.search(r'/spray/nav2\.yaml$',p)]
    if len(candidates)!=1 or digest(candidates[0])!=spec['implementation_bindings'][str(candidates[0].relative_to(root))]:
        raise ValueError('Need the session-bound, unchanged spray Nav2 configuration')
    params=yaml.safe_load(candidates[0].read_text())['local_costmap']['local_costmap']['ros__parameters']
    padded=[[x+math.copysign(params['footprint_padding'],x),y+math.copysign(params['footprint_padding'],y)] for x,y in footprint]
    xlim=max(abs(x) for x,y in padded);ylim=max(abs(y) for x,y in padded)
    if spec['session']!=observer.name:
        raise ValueError('Mismatched run and observer')
    source_terrain=candidates[0].with_name('terrain.json')
    if digest(source_terrain)!=spec['implementation_bindings'][str(source_terrain.relative_to(root))]:
        raise ValueError('Terrain projection configuration differs from the run')
    terrain=json.loads(source_terrain.read_text())
    near=mission['near_sensor']
    floor_rays={angle:ideal_ground_returns(near,terrain,angle) for angle in (near['vertical_min'],-1.48)}
    floor_counts={angle:[] for angle in floor_rays}
    pub=read_rows(observer/'footprint_samples.jsonl')
    private={r['trigger_t']:r for r in read_rows(observer/'private_map_samples.jsonl')}
    body_half_x=max(abs(x) for x,y in footprint);body_half_y=max(abs(y) for x,y in footprint)
    categories=Counter();regions=Counter();upstream=Counter();previous=Counter();region_rows=Counter();centers=Counter()
    recent=deque()
    map_lag=[];raw_lag=[];examples=[];multi=0
    for item in pub:
        risk=grid(item['risk_patch']);t=item['t']
        while recent and t-recent[0][0]>2.:
            recent.popleft()
        packet=private.get(t)
        if packet is not None:
            native=grid(packet['private_patch']);pose=packet['pose']
            counts=Counter();sides=set();unknown_xy=[]
            for x,y in perimeter_cells(native,pose,padded):
                cost=cell_at(native,x,y)
                state=cost_group(cost);counts[state]+=1
                if state=='UNKNOWN':
                    unknown_xy.append((x,y))
                    dx,dy=body(pose,x,y);side=area(dx,dy,xlim,ylim)
                    regions[side]+=1;sides.add(side)
                    belt='INSIDE_BODY' if abs(dx)<=body_half_x and abs(dy)<=body_half_y else (
                        'BETWEEN_BODY_AND_PADDED_OUTLINE' if abs(dx)<=xlim and abs(dy)<=ylim else 'BEYOND_PADDED_OUTLINE')
                    centers[belt]+=1
                    rawcost=cell_at(risk,x,y);upstream[cost_group(rawcost)]+=1
                    prior='NO_RECENT_OUTSIDE_BODY_MEASUREMENT'
                    # This is counterfactual visibility evidence only. An earlier
                    # body interior may have been cleared by the footprint layer.
                    for rt,rg,rp in reversed(recent):
                        bx,by=body(rp,x,y)
                        if abs(bx)<=xlim+.10 and abs(by)<=ylim+.10:
                            continue
                        history=cell_at(rg,x,y)
                        if history is not None and history>=0:
                            prior=cost_group(history);break
                    previous[prior]+=1
            if unknown_xy:
                for angle,rays in floor_rays.items():floor_counts[angle].extend(projected_counts(rays,pose,risk,unknown_xy))
            key='LETHAL' if counts['LETHAL'] else 'UNKNOWN' if counts['UNKNOWN'] else 'NO_BLOCKING_EDGE'
            categories[key]+=1
            region_rows.update(sides)
            multi+=len(sides)>1
            map_lag.append(packet['map_header_t']-t);raw_lag.append(t-item['risk_t'])
            if len(examples)<8 and key!='NO_BLOCKING_EDGE':
                examples.append(dict(t=t,current_edge=key,sides=sorted(sides),private_time_minus_pose=map_lag[-1],
                                     observed_grid_age=raw_lag[-1],raw_map_origin=item['risk_patch']['origin']))
        recent.append((t,risk,item['pose']))
    summary=json.loads((observer/'summary.json').read_text())
    terrain_file=observer/'terrain_window.jsonl'
    terrain_comparison=None
    if terrain_file.is_file():
        frames=read_rows(terrain_file)
        stamps=[f['t'] for f in frames]
        cells_by_frame=[{(round(c[0],3),round(c[1],3)):dict(zip(f['columns'],c)) for c in f['cells']} for f in frames]
        terrain_states=Counter()
        terrain_groups=Counter()
        details=[]
        observed_examples=[]
        for item in pub:
            t=item['t'];packet=private.get(t)
            if not stamps or packet is None:continue
            i=bisect_left(stamps,item['risk_t'])
            nearest=min([j for j in (i,i-1) if 0<=j<len(stamps)],key=lambda j:abs(stamps[j]-item['risk_t']))
            if abs(stamps[nearest]-item['risk_t'])>.16:continue
            native=grid(packet['private_patch']);raw=grid(item['risk_patch'])
            for x,y in perimeter_cells(native,packet['pose'],padded):
                if cell_at(native,x,y)!=-1:continue
                x0,y0=raw.info.origin.position.x,raw.info.origin.position.y
                res=raw.info.resolution
                xcell=x0+(math.floor((x-x0)/res)+.5)*res
                ycell=y0+(math.floor((y-y0)/res)+.5)*res
                row=cells_by_frame[nearest].get((round(xcell,3),round(ycell,3)))
                sample_time=round(item['risk_t']-stamps[nearest],3)
                if row is None:
                    terrain_states['NO_QUALIFIED_TERRAIN_ROW']+=1
                    continue
                kind=cost_group(row['diff_cost'])
                terrain_states[kind]+=1
                details.append(row['points'])
                if row['diff_cost']==-1:
                    terrain_groups['SLOPE_VARIANCE_EXCEEDED' if not row['slope_observed'] else 'SLOPE_OBSERVED']+=1
                if len(observed_examples)<8:
                    observed_examples.append(dict(warning_t=t,terrain_t=stamps[nearest],raw_grid_minus_terrain_s=sample_time,
                                                  private_xy=[x,y],terrain_cell=row))
        terrain_comparison=dict(window_sim_s=[stamps[0],stamps[-1]] if stamps else [],terrain_frames=len(frames),
                                tolerance_s=.16,private_unknown_cell_by_reported_terrain=dict(terrain_states),
                                unknown_diff_class_breakdown=dict(terrain_groups),
                                median_reported_cell_points=statistics.median(details) if details else None,
                                example=observed_examples,source_sha256=digest(terrain_file),
                                scope='A missing terrain detail can mean no input points or insufficient qualified ground. '
                                      'Different pipeline times are possible even within the reported time tolerance.')
    def lag(a):
        return None if not a else dict(median=statistics.median(a),min=min(a),max=max(a))
    return dict(scope='Private Nav2 snapshots around sampled controller warnings. Responses are asynchronous. '
                      'An edge observation cannot reconstruct the exact triggering control iteration or prove the rest of a command arc is clear.',
                session=spec['session'],plan_revision=summary['plan_revision'],observer_status=summary['exit_reason'],
                warning_lines_seen=summary['observed_warning_lines'],public_samples=len(pub),paired_private_samples=len(map_lag),
                private_current_edge_by_sample=dict(categories),unknown_cells_by_vehicle_side=dict(regions),
                samples_with_unknown_by_side=dict(region_rows),samples_with_unknown_on_multiple_sides=multi,
                private_unknown_cell_centers_relative_to_footprint=dict(centers),
                raw_observation_at_private_unknown=dict(upstream),preceding_2s_observed_same_cell_outside_padded_body_plus_10cm=dict(previous),
                private_header_minus_warning_clock_s=lag(map_lag),raw_grid_age_at_warning_s=lag(raw_lag),
                ideal_floor_projection=dict(scope='Synthetic flat level ground, original beam count, declared mounts and self-mask. '
                       'No occlusion, dropout, vehicle motion, temporal qualification or surface classification. Never authorizes motion.',
                       terrain_config_sha256=digest(source_terrain),by_vertical_min_rad={
                             angle:dict(sampled_private_unknown_cells=len(n),fewer_than_three_expected_hits=sum(v<3 for v in n),
                                        median_expected_hits=statistics.median(n) if n else None)
                             for angle,n in floor_counts.items()}),
                examples=examples,terrain_window=terrain_comparison,
                inputs={p.name:digest(p) for p in (observer/'footprint_samples.jsonl',observer/'private_map_samples.jsonl',
                                                    observer/'summary.json',run/'spec.json',run/'mission.json',candidates[0])})


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--observer',required=True,type=Path);ap.add_argument('--run',required=True,type=Path)
    ap.add_argument('--out',type=Path)
    args=ap.parse_args();data=json.dumps(analyze(args.observer,args.run),indent=2)+'\n'
    if args.out:args.out.write_text(data)
    print(data,end='')
