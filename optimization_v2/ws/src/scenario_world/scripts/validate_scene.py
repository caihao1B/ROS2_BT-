"""OFFLINE evaluator: only this side compares observations with simulator truth."""
import copy,json,math,hashlib
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
from PIL import Image,ImageDraw
from shapely.geometry import shape,Polygon,Point,LineString,MultiPolygon,box
from generate_scene import geometries,generate,ROOT,BASE
def read(p):return json.loads(Path(p).read_text())
def require_workspace(p):
    if p.geom_type!='Polygon' or not p.is_valid or p.is_empty or p.area<=0:raise ValueError('workspace must be a valid single connected Polygon')
def validate_pair(link):
    def structural(e):return (e.tag,e.attrib,(e.text or '').strip(),[structural(c) for c in e])
    for c in link.findall('collision'):
        v=link.find(f"visual[@name='{c.get('name')}_visual']")
        if v is None or structural(c.find('geometry'))!=structural(v.find('geometry')) or c.findtext('pose')!=v.findtext('pose'):raise ValueError('collision/visual geometry or transform mismatch')
def mesh_xy(path):
    vertices=[];area=0
    for line in path.read_text().splitlines():
        a=line.split()
        if not a:continue
        if a[0]=='v':vertices.append(list(map(float,a[1:4])))
        if a[0]=='f':
            xy=np.array([vertices[int(v.split('/')[0])-1][:2] for v in a[1:4]])
            area+=abs(np.linalg.det(np.array([xy[1]-xy[0],xy[2]-xy[0]])))/2
    return area
def signed_mesh_volume(path):
    vertices=[];volume=0.
    for line in path.read_text().splitlines():
        a=line.split()
        if not a:continue
        if a[0]=='v':vertices.append(np.array(list(map(float,a[1:4]))))
        if a[0]=='f':
            x,y,z=[vertices[int(v.split('/')[0])-1] for v in a[1:4]]
            volume+=float(np.dot(x,np.cross(y,z)))/6
    return volume
def offline(out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);m=dict(variants={},negative_checks={},alignment_pairs=0)
    for variant in ['base','notch','holes']:
        d=out/variant;generate(d,41,variant);fs=read(d/'maps/prior/workspaces.geojson')['features'];zones=[shape(f['geometry']) for f in fs]
        for z in zones:require_workspace(z)
        c=read(d/'maps/prior/connector.geojson')['features'];road,line=[shape(f['geometry']) for f in c]
        endpoint=[zones[0].covers(Point(line.coords[0])),zones[1].covers(Point(line.coords[-1]))]
        assert all(endpoint) and not zones[0].intersects(zones[1])
        assert all(z.convex_hull.area>z.area for z in zones) and len(fs)==2
        assert 1500<zones[0].area<1800 and 1050<zones[1].area<1200
        assert all(box(0,0,160,112).covers(z) for z in zones)
        mesh_error=[abs(mesh_xy(d/f'assets/meshes/{n}_workspace.obj')-z.area) for n,z in zip(['spray','mowing'],zones)]
        assert max(mesh_error)<1e-5
        tree=ET.parse(d/'assets/worlds/irregular_farm.sdf');models=tree.findall('.//world/model')
        assert not any('cluster' in x.get('name') for x in models)
        for model in models:
            for l in model.findall('link'):validate_pair(l);m['alignment_pairs']+=len(l.findall('collision'))
        for uri in tree.findall('.//mesh/uri'):
            p=Path(uri.text.removeprefix('file://'));assert p.is_file()
            if p.name.startswith('grade'):assert 'vn ' in p.read_text() and signed_mesh_volume(p)>0
        probe=tree.find(".//model[@name='validation_probe']")
        assert list(map(float,probe.findtext('pose').split()[:3]))==read(d/'config/sensor_config.json')['probe']['position']
        widths=[2*Point(line.interpolate(f,normalized=True)).distance(road.boundary) for f in np.linspace(.05,.95,31)]
        assert min(widths)>5.9
        m['variants'][variant]=dict(areas_m2=[z.area for z in zones],holes=[len(z.interiors) for z in zones],bounds=[list(z.bounds) for z in zones],road_length_m=line.length,width_sample_min_m=min(widths),endpoint_inside=endpoint,mesh_area_errors_m2=mesh_error)
    # Same seed reproduces hidden object specification; changing seed must NOT alter prior.
    generate(out/'same',41);generate(out/'different',42)
    assert read(out/'base/evaluation/truth_manifest.json')==read(out/'same/evaluation/truth_manifest.json')
    assert read(out/'base/evaluation/truth_manifest.json')['actors']!=read(out/'different/evaluation/truth_manifest.json')['actors']
    for p in (out/'base/maps/prior').iterdir():assert p.read_bytes()==(out/'different/maps/prior'/p.name).read_bytes()
    m['prior_independent_of_unknown_seed']=True
    def rejection(name,fn):
        try:fn()
        except (ValueError,FileNotFoundError):m['negative_checks'][name]='PASS';return
        raise AssertionError('negative input accepted: '+name)
    rejection('self_intersection',lambda:require_workspace(Polygon([(0,0),(2,2),(2,0),(0,2)])))
    rejection('disconnected_workspace',lambda:require_workspace(MultiPolygon([box(0,0,1,1),box(3,0,4,1)])))
    l=copy.deepcopy(ET.parse(out/'base/assets/worlds/irregular_farm.sdf').find(".//model[@name='ped_0']/link"));l.find('collision/pose').text='1 0 0 0 0 0'
    rejection('offset_collision',lambda:validate_pair(l))
    from online_probe import validate_config
    cfg=read(out/'base/config/sensor_config.json');bad=copy.deepcopy(cfg);bad['topics'].append('/s02/evaluation/dynamic_poses')
    rejection('truth_topic_in_online',lambda:validate_config(bad))
    bad=copy.deepcopy(cfg);bad['ros_domain_id']=120;rejection('wrong_domain',lambda:validate_config(bad))
    bad=copy.deepcopy(cfg);bad['prior_path']=str(out/'base/evaluation');rejection('private_directory',lambda:validate_config(bad))
    (out/'metrics.json').write_text(json.dumps(m,indent=2)+'\n');return m
def trace_at(trace,name,t):
    valid=[r for r in trace if name in r['poses']]
    times=np.array([r['t'] for r in valid]);xyz=np.array([r['poses'][name] for r in valid])
    return np.array([np.interp(t,times,xyz[:,i]) for i in range(3)])
def image_roi(center,size,probe):
    corners=np.array([[center[0]+sx*size[0]/2,center[1]+sy*size[1]/2,center[2]+sz*size[2]/2] for sx in [-1,1] for sy in [-1,1] for sz in [-1,1]])-probe['position']
    pitch=probe['camera_pitch'];depth=np.cos(pitch)*corners[:,0]-np.sin(pitch)*corners[:,2];up=np.sin(pitch)*corners[:,0]+np.cos(pitch)*corners[:,2]
    f=probe['width']/2/math.tan(probe['camera_hfov']/2);u=probe['width']/2-f*corners[:,1]/depth;v=probe['height']/2-f*up/depth
    return [max(0,int(min(u))-2),max(0,int(min(v))-2),min(probe['width'],int(max(u))+3),min(probe['height'],int(max(v))+3)]
def observed(case):
    case=Path(case);o=read(case/'online/result.json');e=read(case/'evaluation/result.json');truth=read(case/'scene/evaluation/truth_manifest.json');cfg=read(case/'scene/config/sensor_config.json');rt=read(case/'runtime.json')
    m=dict(completed=rt['success'] and o['completed'] and e['completed'],counts=o['counts'],sim_seconds=o['clock'],wall_seconds=rt['wall_seconds'],active_rtf=o['clock']/o['wall_seconds'],lidar_max_hits={},camera_color_pixels={},actor_displacement_m={})
    for a in truth['actors']:
        pos=np.array([r['poses'][a['id']] for r in e['trajectory'] if a['id'] in r['poses']]);m['actor_displacement_m'][a['id']]=float(np.linalg.norm(pos.max(axis=0)-pos.min(axis=0)))
    for snap in o['snapshots']:
        cloud=np.load(case/'online'/snap['file'])['xyz']
        for a in truth['low_obstacles']+truth['actors']:
            center=np.array(a['center']) if a in truth['low_obstacles'] else trace_at(e['trajectory'],a['id'],snap['t'])
            half=np.array(a['size'])/2+.025
            hits=int(np.sum((np.abs(cloud-center)<=half).all(axis=1)&(cloud[:,2]>.03)))
            m['lidar_max_hits'][a['id']]=max(hits,m['lidar_max_hits'].get(a['id'],0))
    img=Image.open(case/'online/camera.png');rgb=np.array(img).astype(float);draw=ImageDraw.Draw(img)
    for a in truth['low_obstacles']+[truth['actors'][0],truth['actors'][-1]]:
        center=a['center'] if a in truth['low_obstacles'] else trace_at(e['trajectory'],a['id'],o['image_t'])
        roi=image_roi(center,a['size'],cfg['probe']);x0,y0,x1,y1=roi
        if x1<=x0 or y1<=y0:count=0
        else:
            r,g,b=np.moveaxis(rgb[y0:y1,x0:x1],-1,0)
            if a['id'].startswith('unknown'):mask=(r>120)&(r>1.4*g)&(g>1.8*b)
            elif a['id'].startswith('ped'):mask=(b>70)&(b>1.5*r)&(b>1.2*g)
            else:mask=(r>140)&(g>90)&(g>.45*r)&(b<65)
            count=int(mask.sum());draw.rectangle(roi,outline='red',width=1);draw.text((x0,max(0,y0-12)),a['id'],fill='red')
        m['camera_color_pixels'][a['id']]=count
    img.save(case/'camera_evaluated.png')
    m['online_isolation']=o['ros_domain_id']==119 and e['ros_domain_id']==120 and not any('/evaluation/' in t for t in o['graph']) and not any('/evaluation/' in p for p in o['read_files'])
    m['contact_pairs']=e['contact_pairs']
    if truth['fixture']:
        m['fixture_displacement_m']=float(np.linalg.norm(trace_at(e['trajectory'],'contact_fixture',e['trajectory'][-1]['t'])-truth['fixture_start']))
        m['actor_fixture_contact_count']=sum(n for pair,n in e['contact_pairs'].items() if 'contact_fixture' in pair and 'ped_0' in pair)
    (case/'metrics.json').write_text(json.dumps(m,indent=2)+'\n');return m
def reproducibility(first,second):
    a=read(Path(first)/'evaluation/result.json')['trajectory'];b=read(Path(second)/'evaluation/result.json')['trajectory']
    lo=max(a[0]['t'],b[0]['t']);hi=min(a[-1]['t'],b[-1]['t']);ts=np.arange(lo,hi,.2);errors={}
    for name in a[0]['poses']:
        errors[name]=max(float(np.linalg.norm(trace_at(a,name,t)-trace_at(b,name,t))) for t in ts)
    return dict(common_interval_seconds=[lo,hi],sample_count=len(ts),max_position_error_m=errors,tolerance_m=.03,passed=max(errors.values())<.03)
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--offline',type=Path);p.add_argument('--case',type=Path);a=p.parse_args();print(json.dumps(offline(a.offline) if a.offline else observed(a.case),indent=2))
