"""Offline simulator composer. Preserve S02/S04; add sparse natural survey landmarks.
Generated bark/brick assets are original procedural textures, not fiducial IDs.
This module is NEVER imported by the online SLAM/preprocessor/controller.
"""
import sys,importlib.util
from xml.etree import ElementTree as E
from PIL import Image,ImageDraw
from s06_common import *
def compose(out,kind,case,session):
    # Frozen orchestration's scene/config constructor is reused unchanged.
    sys.path.insert(0,str(BASE/'ws/src/localization_guard/scripts'))
    spec=importlib.util.spec_from_file_location('s05_mapping_base',BASE/'ws/src/localization_guard/scripts/runtime.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    mod.compose(out,kind,'loop',session)
    if case=='weak_texture':return
    root=E.parse(out/'world.sdf').getroot();w=root.find('world');rng=np.random.default_rng(41)
    texture=out/'brick.png';bark=out/'bark.png'
    a=np.clip(rng.normal([125,103,82],15,(256,256,3)),0,255).astype('uint8');im=Image.fromarray(a);d=ImageDraw.Draw(im)
    for row,y in enumerate(range(0,256,32)):
        d.line([(0,y),(256,y)],fill=(180,171,151),width=3)
        for x in range(-32 if row%2 else 0,256,64):d.line([(x,y),(x,y+32)],fill=(177,169,150),width=3)
    im.save(texture)
    a=np.clip(rng.normal([88,67,44],18,(256,256,3)),0,255).astype('uint8');a[:,::11]//=2;Image.fromarray(a).save(bark)
    def el(p,k,t=None,**kw):
        e=E.SubElement(p,k,kw)
        if t is not None:e.text=str(t)
        return e
    def mat(v,tex):
        old=v.find('material')
        if old is not None:v.remove(old)
        m=el(v,'material');el(m,'ambient','0.8 0.8 0.8 1');el(m,'diffuse','1 1 1 1');metal=el(el(m,'pbr'),'metal');el(metal,'albedo_map','file://'+str(tex));el(metal,'roughness',.95);el(metal,'metalness',0)
    for name in ['pump_house','shed','utility']:
        m=w.find(f"model[@name='{name}']")
        if m is not None:
            for v in m.findall('link/visual'):mat(v,texture)
    # Believable perimeter trees plus roadside trees outside the six-metre corridor.
    positions=[(22,12),(34,10),(37,20),(22,21),(40,27),(23,33),(36,43),(32,39),(45,47),(53,32),
               (119,51),(130,49),(134,56),(126,68),(115,70),(139,63),(114,57),(123,73),
               (35,59),(37,69),(50,69)]
    road=read(BASE/'maps/prior/connector.geojson')['features'][1]['geometry']['coordinates']
    for j in range(10,len(road)-8,18):
        xy=np.array(road[j]);tangent=np.array(road[j+2])-road[j-2];normal=np.array([-tangent[1],tangent[0]])/np.linalg.norm(tangent)
        positions.append(tuple(xy+normal*(5.5 if j%36==10 else -5.8)))
    cfg=yaml.safe_load((BASE/'config/routes/mapping_routes.yaml').read_text())
    segments=np.array([cfg['spawn_survey'][:2]]+cfg['mapping_full']['outbound'])
    for xy in positions:
        xy=np.array(xy);delta=segments[1:]-segments[:-1];ratio=np.clip(np.sum((xy-segments[:-1])*delta,axis=1)/np.sum(delta*delta,axis=1),0,1)
        if np.min(np.linalg.norm(xy-(segments[:-1]+ratio[:,None]*delta),axis=1))<1.8:raise ValueError('new tree violates survey route clearance')
    for i,(x,y) in enumerate(positions):
        m=el(w,'model',name=f's06_tree_{i}');el(m,'static','true');el(m,'pose',f'{x} {y} 0 0 0 0');link=el(m,'link',name='trunk');h=float(rng.uniform(3.0,4.5));r=float(rng.uniform(.22,.35))
        for tag in ['collision','visual']:
            v=el(link,tag,name='stem');el(v,'pose',f'0 0 {h/2} 0 0 0');g=el(el(v,'geometry'),'cylinder');el(g,'radius',r);el(g,'length',h)
            if tag=='visual':mat(v,bark)
        # Low-poly sphere crowns cast lidar returns; same visual/collision geometry.
        for j,(dx,dy,dz,rr) in enumerate([(.3,0,h,.95),(-.4,.3,h-.5,.65)]):
            for tag in ['collision','visual']:
                v=el(link,tag,name=f'crown{j}');el(v,'pose',f'{dx} {dy} {dz} 0 0 0');el(el(el(v,'geometry'),'sphere'),'radius',rr)
                if tag=='visual':mt=el(v,'material');el(mt,'ambient','0.22 0.36 0.11 1');el(mt,'diffuse','0.28 0.42 0.12 1')
    E.indent(root);E.ElementTree(root).write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
    dump(out/'mapping_scene.json',dict(version='s06.natural.3',base='S02+s03+s04 unchanged',trees=len(positions),natural_textures=True,artificial_fiducials=False,license='Original project-generated geometry/textures; same project use rights',sha256=digest(out/'world.sdf')))
