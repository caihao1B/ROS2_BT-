"""Measured ground-ray occupancy; no simulator geometry or free-space oracle.

Free and obstacle votes are distinct per sensor frame. Three obstacle votes
win over free rays; unresolved hits stay unknown. Runtime S09 remains veto.
"""
from common14 import *
from regional_math14 import rotation
from PIL import Image
from collision_volume14 import envelope, hazardous

def ground_plane(points):
    candidates=points[(points[:,2]<.15)&(np.linalg.norm(points[:,:2],axis=1)<12)]
    if len(candidates)<150:raise ValueError('REGIONAL_OCCUPANCY_NO_GROUND')
    rng=np.random.default_rng(41);best=None;count=0
    for _ in range(100):
        p=candidates[rng.choice(len(candidates),3,replace=False)];n=np.cross(p[1]-p[0],p[2]-p[0]);length=np.linalg.norm(n)
        if length<1e-8:continue
        n/=length
        if n[2]<0:n=-n
        if n[2]<math.cos(math.radians(15)):continue
        d=-n@p[0];mask=np.abs(candidates@n+d)<.025
        if mask.sum()>count:count=int(mask.sum());best=mask
    if count<150:raise ValueError('REGIONAL_OCCUPANCY_GROUND_UNOBSERVABLE')
    p=candidates[best];center=p.mean(0);_,_,vt=np.linalg.svd(p-center,full_matrices=False);n=vt[-1]
    if n[2]<0:n=-n
    if n[2]<math.cos(math.radians(5-.7)):raise ValueError('REGIONAL_OCCUPANCY_UNQUALIFIED_SLOPE')
    return n,-n@center,count

def build(stations,poses,out,zone):
    # Origin/extent is a raster allocation from measured endpoints, not a free mask.
    transformed=[]
    for s in stations:
        p=poses[s['name']]
        for raw in s['clouds']:
            a=raw[np.isfinite(raw).all(1)&(np.linalg.norm(raw,axis=1)<30)];transformed.append(a[:,:2]@rotation(p[2]).T+p[:2])
    xy=np.vstack(transformed);resolution=.1;origin=np.floor((xy.min(0)-1)/resolution)*resolution;size=np.ceil((xy.max(0)+1-origin)/resolution).astype(int);width,height=map(int,size)
    free=np.zeros((height,width),np.uint16);obstacle=np.zeros_like(free);planes=[];volume=envelope()
    def ids(xy):
        cells=np.floor((xy-origin)/resolution).astype(int);inside=np.all(cells>=0,axis=1)&np.all(cells<size,axis=1);cells=cells[inside];return cells[:,1]*width+cells[:,0]
    for s in stations:
        p=poses[s['name']];R=rotation(p[2]);sensor=np.array(p[:2])+R@[.1,0.]
        for index,raw in enumerate(s['clouds']):
            a=raw[np.isfinite(raw).all(1)&(np.linalg.norm(raw,axis=1)<30)]
            n,d,count=ground_plane(a);h=a@n+d;ground=a[np.abs(h)<=.025];hazard=a[hazardous(h,n[2],volume['ceiling_m'])]
            targets=ground[:,:2]@R.T+p[:2];observed=hazard[:,:2]@R.T+p[:2]
            ray_ids=[]
            for target in targets:
                samples=np.linspace(sensor,target,max(2,int(math.ceil(np.linalg.norm(target-sensor)/(.5*resolution)))+1));ray_ids.append(ids(samples))
            free.flat[np.unique(np.concatenate(ray_ids))]+=1
            obstacle.flat[np.unique(ids(observed))]+=1
            planes.append(dict(station=s['name'],frame=index,normal=n.tolist(),offset=float(d),support=count,ground_points=len(ground),obstacle_points=len(hazard),overhead_points=int((h/n[2]>volume['ceiling_m']).sum())))
    pixels=np.full((height,width),205,np.uint8);pixels[(free>=3)&(obstacle==0)]=254;pixels[obstacle>=3]=0
    pgm=out/(zone+'_navigation.pgm');Image.fromarray(pixels[::-1]).save(pgm)
    yml=out/(zone+'_navigation.yaml');yml.write_text(yaml.safe_dump(dict(image=pgm.name,resolution=resolution,origin=[*origin.tolist(),0.],negate=0,occupied_thresh=.5,free_thresh=.196)))
    evidence=out/(zone+'_occupancy_evidence.npz');np.savez_compressed(evidence,free_frames=free,obstacle_frames=obstacle,origin=origin,resolution=resolution)
    summary=out/(zone+'_occupancy.json');dump(summary,dict(collision_volume=volume,planes=planes,free_cells=int((pixels==254).sum()),occupied_cells=int((pixels==0).sum()),unknown_cells=int((pixels==205).sum()),policy='Three observed frames; occupied wins; uncertain hits remain unknown; only measured ground rays clear; overhead exclusion never clears ground'))
    return dict(navigation_yaml=str(yml.relative_to(ROOT)),navigation_pgm=str(pgm.relative_to(ROOT)),occupancy_evidence=str(evidence.relative_to(ROOT)),occupancy_summary=str(summary.relative_to(ROOT)))
