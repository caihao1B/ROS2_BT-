"""Simulator-only motion fixtures. Never imported by online prediction nodes."""
from common15 import *
import math,os
from xml.etree import ElementTree as E
add14()
from scene14 import compose as compose14
from common14 import cluster

def scenario():
    return os.environ.get('S15_SCENARIO','turning'),os.environ.get('S15_SPLIT','development')

def compose(out,kind,case,session):
    if kind!='diff':raise ValueError('S15 current closed-loop qualification is diff only')
    hint=compose14(out,kind,'nominal_spray_cluster',session)
    root=E.parse(out/'world.sdf');w=root.getroot().find('world');start=cluster(kind)['start']
    name,split=scenario();variant={'train':-.25,'val':.25,'test':0.,'development':0.}[split]
    # All methods receive identical scene, deployment and task within each episode.
    x,y,h=start
    definitions={
        'turning':[[0,2.,-2.],[10,2.,-2.],[22,1.5,0.],[29,2.7,.1],[39,4.,1.7],[65,4.,1.7]],
        'stop_go':[[0,1.7,-2.],[10,1.7,-2.],[19,1.7,-.7],[25,1.7,-.7],[40,1.7,2.2],[65,1.7,2.2]],
        'yield':[[0,2.,-2.],[10,2.,-2.],[22,1.6,-.8],[28,1.6,-.8],[40,2.1,2.1],[65,2.1,2.1]],
        'occlusion':[[0,2.,-2.],[8,2.,-2.],[38,2.,2.],[65,2.,2.]]}
    scale={'train':.9,'val':1.1,'test':1.,'development':1.}[split]
    knots=[[t*scale,px+variant,py*(1.+variant*.3)] for t,px,py in definitions[name]]
    plugin=E.SubElement(w,'plugin',filename='libs15_actor.so',name='s15sim::ScriptedMotion')
    actors=[]
    def person(label,points):
        m=E.parse(BASE/'assets/perception/person/model.sdf').getroot().find('model');m.set('name',label);l=m.find('link');l.set('name','body');E.SubElement(l,'gravity').text='false'
        E.SubElement(m,'pose').text=f'{x+points[0][1]} {y+points[0][2]} .02 0 0 {math.pi}'
        for uri in m.findall('.//uri'):uri.text='file://'+str(BASE/'assets/perception/person/meshes/walking.dae')
        w.append(m);actor=E.SubElement(plugin,'actor');E.SubElement(actor,'name').text=label
        for t,px,py in points:E.SubElement(actor,'waypoint').text=f'{t} {x+px} {y+py} .02'
        actors.append(dict(name=label,knots=points))
    person('s15_person',knots)
    if name=='yield':
        person('s15_neighbor',[[0,3.3+variant,1.8],[8,3.3+variant,1.8],[25,2.5+variant,-.2],[40,3.8+variant,-2.],[65,3.8+variant,-2.]])
    if name=='occlusion':
        model=E.fromstring(f'<model name="s15_occluder"><static>true</static><pose>{x+1.3} {y-1.05} 1.05 0 0 0</pose><link name="body"><collision name="collision"><geometry><box><size>.2 .35 2.1</size></box></geometry></collision><visual name="visual"><geometry><box><size>.2 .35 2.1</size></box></geometry><material><diffuse>.3 .35 .4 1</diffuse></material></visual></link></model>');w.append(model)
    root.write(out/'world.sdf',encoding='unicode')
    dump(out/'fixture_evaluation_only.json',dict(scenario=name,split=split,actors=actors,source='SIMULATOR_ONLY_NOT_PREDICTOR_INPUT',articulated_gait=False,interaction='scripted yield; no social response dynamics'))
    dump(out/'benchmark_task.json',dict(scope='S15_TWO_POINT_BENCHMARK_NOT_FULL_CLUSTER',start_map=[x-28,y-14,h],distances=[0.,.6],required_points=2,quantity_each=2.,original_s14_plan=cluster(kind)['cluster']))
    return hint
