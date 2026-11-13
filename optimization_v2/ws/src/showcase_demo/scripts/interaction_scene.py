"""Explicit, versioned simulator actor behavior; no actor state feeds robot nodes."""
from common_showcase import *
from xml.etree import ElementTree as E
import yaml,math

def apply(out,kind):
    path=out/'world.sdf';root=E.parse(path);world=root.getroot().find('world');plugin=world.find("plugin[@name='s02::ScriptedMotion']")
    if plugin is None:raise ValueError('ORIGINAL_ACTORS_MISSING')
    binary=BASE/'ws/install/showcase_demo/lib/libshowcase_actors.so'
    if not binary.exists():raise ValueError('BUILD_SHOWCASE_ACTORS_FIRST')
    built=read(OUT/'actor_build.json')
    if built!={'source_sha256':sha(PKG/'src/ReactiveActors.cc'),'binary_sha256':sha(binary)}:raise ValueError('SHOWCASE_ACTOR_BUILD_STALE')
    plugin.set('filename',str(binary));plugin.set('name','showcase::ReactiveActors')
    cfg=yaml.safe_load((BASE/f'config/vehicles/{kind}.yaml').read_text());points=cfg['footprint'];tool=yaml.safe_load((BASE/'config/tool_profiles.yaml').read_text())['spray' if kind=='diff' else 'mower'];x,y=tool['mount_xyz'][:2];a,b=[v/2 for v in tool['collision_size'][:2]];points=points+[[x-a,y-b],[x+a,y+b]]
    for key,value in dict(vehicle_rear=min(p[0] for p in points),vehicle_front=max(p[0] for p in points),vehicle_half_width=max(abs(p[1]) for p in points),events=str(out/'actor_interactions.jsonl')).items():E.SubElement(plugin,key).text=str(value)
    actors=[]
    for actor in plugin.findall('actor'):
        name=actor.findtext('name');model=world.find("model[@name='"+name+"']");radius=.45
        for collision in model.findall('.//collision'):
            box=collision.findtext('geometry/box/size');cylinder=collision.findtext('geometry/cylinder/radius')
            if box:radius=max(radius,math.hypot(*[float(v)/2 for v in box.split()[:2]]))
            if cylinder:radius=max(radius,float(cylinder))
        E.SubElement(actor,'radius').text=str(radius);actors.append(dict(name=name,radius=radius))
    root.write(path,encoding='unicode');(out/'reactive_actors_used.cc').write_bytes((PKG/'src/ReactiveActors.cc').read_bytes())
    dump(out/'interaction_scene.json',dict(scope='SHOWCASE_ONLY',version='bounded_local_yield_v2_hysteresis',original_actors_retained=actors,static_geometry_unchanged=True,periodic_intent_time_frozen_while_yielding=True,clear_hold_sim_s=.5,robot_truth_input=False,formal_original_scenario_pass=False,binary_sha256=sha(binary),source_sha256=sha(PKG/'src/ReactiveActors.cc')))
