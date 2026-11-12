"""Offline geometry exporter; uses frozen S02 assets without changing them."""
import argparse,copy,hashlib,json,math
from pathlib import Path
from xml.etree import ElementTree as E
import yaml
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';PKG=Path(__file__).resolve().parents[1]
def el(p,k,v=None,**a):
    x=E.SubElement(p,k,a)
    if v is not None:x.text=str(v)
    return x
def values(v):return ' '.join(map(str,v))
def write(path,root):path.parent.mkdir(parents=True,exist_ok=True);E.indent(root);E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
def config(kind):
    path=BASE/f'config/vehicles/{kind}.yaml';tpath=BASE/'config/tool_profiles.yaml';c=yaml.safe_load(path.read_text());tools=yaml.safe_load(tpath.read_text())
    c['tool']=tools['spray' if kind=='diff' else 'mower'];c['config_hash']=hashlib.sha256(path.read_bytes()+tpath.read_bytes()).hexdigest();return c
def inertial(link,mass,xyz,size):
    i=el(link,'inertial');el(i,'pose',values([*xyz,0,0,0]));el(i,'mass',mass);a,b,c=size;j=el(i,'inertia')
    for k,v in [('ixx',mass*(b*b+c*c)/12),('iyy',mass*(a*a+c*c)/12),('izz',mass*(a*a+b*b)/12),('ixy',0),('ixz',0),('iyz',0)]:el(j,k,v)
def shape(link,name,kind,dims,pose,color,friction=1):
    for tag in ['visual','collision']:
        v=el(link,tag,name=name if tag=='collision' else name+'_visual');el(v,'pose',values(pose));g=el(v,'geometry');s=el(g,kind)
        if kind=='box':el(s,'size',values(dims))
        elif kind=='sphere':el(s,'radius',dims[0])
        else:el(s,'radius',dims[0]);el(s,'length',dims[1])
        if tag=='visual':m=el(v,'material');el(m,'ambient',color);el(m,'diffuse',color)
        else:
            surface=el(v,'surface');f=el(el(surface,'friction'),'ode');el(f,'mu',friction);el(f,'mu2',friction)
    s=el(link,'sensor',name=name+'_contact',type='contact');ct=el(s,'contact');el(ct,'collision',name);el(ct,'topic','/s03/evaluation/contacts');el(s,'always_on','true');el(s,'update_rate',50)
def joint(m,name,parent,child,axis=None,limit=None,effort=40):
    j=el(m,'joint',name=name,type='revolute' if axis else 'fixed');el(j,'parent',parent);el(j,'child',child)
    if axis:
        a=el(j,'axis');el(a,'xyz',values(axis));l=el(a,'limit');el(l,'lower',-limit if limit else -1e16);el(l,'upper',limit if limit else 1e16);el(l,'effort',effort);el(l,'velocity',30 if not limit else .6);d=el(a,'dynamics');el(d,'damping',.01)
def vehicle(c,session='preview'):
    m=E.Element('model',name='vehicle');el(m,'self_collide','false')
    body=el(m,'link',name='base_link');b=c['body'];inertial(body,b['mass'],b['center'],b['size']);shape(body,'body','box',b['size'],[*b['center'],0,0,0],'.1 .38 .80 1' if c['id']=='diff' else '.08 .65 .27 1')
    tool=c['tool'];link=el(m,'link',name='tool_link');el(link,'pose',values([*tool['mount_xyz'],0,0,0]));inertial(link,tool['mass_kg'],[0,0,0],tool['collision_size']);shape(link,'tool','box',tool['collision_size'],[0]*6,'.95 .65 .10 1');joint(m,'tool_mount','base_link','tool_link')
    wheel=c['wheel'];r=wheel['radius'];track=wheel['track'];L=c['wheelbase']
    axles=[('rear',0)]+([('front',L)] if c['id']=='ackermann' else [])
    total=b['mass']+tool['mass_kg']
    for axle,x in axles:
        for side,sgn in [('left',1),('right',-1)]:
            parent='base_link';pos=[x,sgn*track/2,r]
            if axle=='front':
                parent='knuckle_'+side;kn=el(m,'link',name=parent);el(kn,'pose',values([*pos,0,0,0]));inertial(kn,.3,[0,0,0],[.05,.05,.05]);joint(m,'steer_'+side+'_joint','base_link',parent,[0,0,1],c['steering']['joint_limit_rad'],c['steering']['effort_Nm']);total+=.3
            name=axle+'_'+side;w=el(m,'link',name=name+'_wheel');el(w,'pose',values([*pos,0,0,0]));mass=wheel['mass']
            i=el(w,'inertial');el(i,'mass',mass);it=el(i,'inertia')
            for k,v in [('ixx',mass*(3*r*r+wheel['width']**2)/12),('iyy',mass*r*r/2),('izz',mass*(3*r*r+wheel['width']**2)/12),('ixy',0),('ixz',0),('iyz',0)]:el(it,k,v)
            shape(w,name,'cylinder',[r,wheel['width']],[0,0,0,math.pi/2,0,0],'.10 .11 .13 1',wheel['friction']);joint(m,name+'_joint',parent,name+'_wheel',[0,1,0],effort=wheel['effort_Nm']);total+=mass
    if c['id']=='diff':
        support=c['support'];name='caster_front';sr=support['radius'];sm=support['mass']
        l=el(m,'link',name=name);el(l,'pose',values([support['caster_x'],0,sr,0,0,0]));inertial(l,sm,[0,0,0],[2*sr]*3);shape(l,name,'sphere',[sr],[0]*6,'.25 .27 .3 1',support['friction']);joint(m,name+'_fixed','base_link',name);total+=sm
    p=el(m,'plugin',filename='libs03_drive.so',name='s03::WheelDrive');el(p,'config',json.dumps(c,separators=(',',':')));el(p,'session',session)
    c['total_mass_kg']=total
    # Temporary front-range stop probe for safe physical passage, not the S04 suite.
    s=el(body,'sensor',name='front_probe',type='gpu_lidar');el(s,'pose',values([.54 if c['id']=='diff' else 1.28,0,.55,0,0,0]));el(s,'topic','/s03/front_scan');el(s,'update_rate',20);el(s,'always_on','true')
    lidar=el(s,'lidar');scan=el(lidar,'scan');h=el(scan,'horizontal')
    for k,v in [('samples',181),('resolution',1),('min_angle',-1.0),('max_angle',1.0)]:el(h,k,v)
    ran=el(lidar,'range')
    for k,v in [('min',.08),('max',8),('resolution',.01)]:el(ran,k,v)
    noise=el(lidar,'noise');el(noise,'type','gaussian');el(noise,'mean',0);el(noise,'stddev',.01)
    return m
def canonical():
    out=PKG/'generated';out.mkdir(parents=True,exist_ok=True)
    for kind in ['diff','ackermann']:
        c=config(kind);s=E.Element('sdf',version='1.10');s.append(vehicle(c));write(out/(kind+'.sdf'),s);(out/(kind+'.json')).write_text(json.dumps(c,indent=2)+'\n')
def world(out,kind,session,start,rtf=2):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);root=E.parse(BASE/'assets/worlds/irregular_farm.sdf').getroot();w=root.find('world')
    # Isolated chassis qualification: preserve terrain/static obstacles, park actors.
    # Dynamic interaction is a later perception/prediction acceptance, not C02.
    probe=w.find("model[@name='validation_probe']");w.remove(probe)
    motion=w.find("plugin[@name='s02::ScriptedMotion']")
    parked=[]
    for i,actor in enumerate(motion.findall('actor')):
        name=actor.findtext('name');model=w.find(f"model[@name='{name}']")
        z=float(model.findtext('pose').split()[2]);pose=[10+2*i,100,z,0,0,0]
        model.find('static').text='true';model.find('pose').text=values(pose)
        parked.append(dict(name=name,pose=pose))
    w.remove(motion)
    w.find('physics/real_time_factor').text=str(rtf)
    c=config(kind);v=vehicle(c,session);el(v,'pose',values([start[0],start[1],.025,0,0,start[2]]));w.append(v)
    write(out/'world.sdf',root);c.update(session=session,initial_pose=start)
    c['test_fixture']=dict(dynamic_agents='parked outside driving trials',parked=parked,dynamic_avoidance='NOT_QUALIFIED')
    (out/'vehicle.json').write_text(json.dumps(c,indent=2)+'\n');return c
if __name__=='__main__':canonical();print('Generated both lightweight physical vehicles')
