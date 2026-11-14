from common import *
import math,copy,yaml
from xml.etree import ElementTree as E
sys.path.insert(0,str(BASE/'ws/src/robot_description/scripts'))
from generate import el,values,write,inertial,shape,joint

def profiles():
 old=yaml.safe_load((BASE/'config/vehicles/ackermann.yaml').read_text())
 defs=dict(spray=dict(label='Husky A200 dimension reference',kind='differential',L=.512,T=.555,r=.1651,w=.1143,body=[.9874,.5709,.26],center=[0,0,.26],mass=42.,wm=2.,length=.990,width=.670,ground=.130,toolwidth=0.,maxv=1.),
 inspection=dict(label='Jackal dimension reference',kind='differential',L=.262,T=.37559,r=.098,w=.040,body=[.508,.310,.185],center=[0,0,.1575],mass=15.092,wm=.477,length=.508,width=.430,ground=.065,toolwidth=0.,maxv=1.),
 mowing=dict(label='Project Dino front-Ackermann primitive adaptation',kind='ackermann',L=1.638,T=1.8,r=.33150052146263664,w=.167956,body=[2.78,2.77,.32],center=[.819,0,.978218021463],mass=974.9,wm=49.0043,length=2.78,width=2.77,ground=.818218021463,toolwidth=1.6,maxv=1.))
 result={}
 for task,d in defs.items():
  ack=d['kind']=='ackermann';c=copy.deepcopy(old);c.update(schema='R02-VEHICLE/1',task_id=task,id='ackermann' if ack else 'diff',kind=d['kind'],revision='r02.1',label=d['label'])
  c['body']=dict(size=d['body'],center=d['center'],mass=d['mass']);c['wheelbase']=d['L']
  c['wheel']=dict(radius=d['r'],width=d['w'],mass=d['wm'],track=d['T'],friction=1.0 if ack else .5,lateral_friction=1.0 if ack else .5,effort_Nm=600 if ack else 40)
  c['calibration']=dict(effective_track_m=d['T'],drive_track_m=d['T'],scope='initial geometric values; physics calibration evaluated separately')
  c['steering'].update(joint_limit_rad=math.pi/4 if ack else 0.,max_curvature=math.tan(.5)/d['L'] if ack else 0.,effort_Nm=600 if ack else 40,rate_rad_s=.6)
  c['limits'].update(max_speed=d['maxv'],reverse=.3,yaw_rate=.8,acceleration=.5,deceleration=.8)
  c['qualification_candidates']=dict(work_mps=.5 if ack else .4,transit_mps=[.3,.6,.8],connector_mps=.6,slope_mps=.3,slope_deg=5)
  cx=d['L']/2 if ack else 0.;a,b=d['length']/2,d['width']/2;c['footprint']=[[cx-a,-b],[cx+a,-b],[cx+a,b],[cx-a,b]]
  c['tool']=dict(kind='mower' if ack else 'spray' if task=='spray' else 'none',mount_xyz=[cx,0,.16 if ack else .45],mass_kg=10 if ack else 2 if task=='spray' else 0,collision_size=[.85,1.6,.12] if ack else [.2,.2,.12],effective_width_m=d['toolwidth'],dwell_seconds=3. if task=='spray' else 0.)
  c['physical_contract']=dict(reference_ground_clearance_m=d['ground'],body_frame='rear axle ground projection' if ack else 'axle-midpoint ground projection',declared_outer_length_m=d['length'],declared_outer_width_m=d['width'],nominal_bicycle_radius_m=d['L']/math.tan(.5) if ack else 0,spin_in_place_allowed=not ack,hardware_equivalence=False,geometry_source='local Dino SDF' if ack else 'manufacturer dimensions and official ROS description',dynamics_scope='Gazebo rigid contact + bounded joint velocity actuator; not hardware torque/power validation')
  c['config_hash']=canonical(c);result[task]=c
 return result

def vehicle(c,session):
 m=E.Element('model',name='vehicle');el(m,'self_collide','false');body=el(m,'link',name='base_link');b=c['body'];inertial(body,b['mass'],b['center'],b['size']);shape(body,'body','box',b['size'],[*b['center'],0,0,0],'.7 .6 .1 1' if c['task_id']!='mowing' else '.1 .55 .25 1')
 tool=c['tool']
 if tool['mass_kg']:
  l=el(m,'link',name='tool_link');el(l,'pose',values([*tool['mount_xyz'],0,0,0]));inertial(l,tool['mass_kg'],[0,0,0],tool['collision_size']);shape(l,'tool','box',tool['collision_size'],[0]*6,'.15 .2 .3 1');joint(m,'tool_mount','base_link','tool_link')
 ack=c['kind']=='ackermann';wheel=c['wheel'];L=c['wheelbase'];r=wheel['radius'];T=wheel['track']
 for axle,x in [('rear',0 if ack else -L/2),('front',L if ack else L/2)]:
  for side,sign in [('left',1),('right',-1)]:
   parent='base_link';pos=[x,sign*T/2,r]
   if ack and axle=='front':
    parent='knuckle_'+side;l=el(m,'link',name=parent);el(l,'pose',values([*pos,0,0,0]));inertial(l,.3,[0,0,0],[.05]*3);joint(m,'steer_'+side+'_joint','base_link',parent,[0,0,1],c['steering']['joint_limit_rad'],c['steering']['effort_Nm'])
   name=axle+'_'+side;l=el(m,'link',name=name+'_wheel');el(l,'pose',values([*pos,0,0,0]));mass=wheel['mass'];i=el(l,'inertial');el(i,'mass',mass);it=el(i,'inertia')
   for k,v in [('ixx',mass*(3*r*r+wheel['width']**2)/12),('iyy',mass*r*r/2),('izz',mass*(3*r*r+wheel['width']**2)/12),('ixy',0),('ixz',0),('iyz',0)]:el(it,k,v)
   shape(l,name,'cylinder',[r,wheel['width']],[0,0,0,math.pi/2,0,0],'.08 .09 .10 1',wheel['friction'])
   f=l.find('collision/surface/friction/ode');f.find('mu2').text=str(wheel['lateral_friction']);el(f,'fdir1','1 0 0')
   joint(m,name+'_joint',parent,name+'_wheel',[0,1,0],effort=wheel['effort_Nm'])
 p=el(m,'plugin',filename='libr02_drive.so',name='r02::WheelDrive');el(p,'config',json.dumps(c,separators=(',',':')));el(p,'session',session)
 return m

def world(out,c,session,full=False,start=(0,0,0),rtf=1.):
 out.mkdir(parents=True,exist_ok=True)
 if full:
  root=E.parse(BASE/'assets/worlds/irregular_farm.sdf').getroot();w=root.find('world')
  probe=w.find("model[@name='validation_probe']")
  if probe is not None:w.remove(probe)
 else:
  root=E.Element('sdf',version='1.10');w=el(root,'world',name='irregular_farm');el(w,'gravity','0 0 -9.8');p=el(w,'physics',name='default',type='ignored');el(p,'max_step_size',.001);el(p,'real_time_factor',rtf)
  for fn,n in [('gz-sim-physics-system','Physics'),('gz-sim-user-commands-system','UserCommands'),('gz-sim-scene-broadcaster-system','SceneBroadcaster'),('gz-sim-contact-system','Contact')]:el(w,'plugin',filename=fn,name='gz::sim::systems::'+n)
  ground=el(w,'model',name='ground');el(ground,'static','true');l=el(ground,'link',name='ground');shape(l,'ground','box',[400,400,.1],[0,0,-.05,0,0,0],'.25 .4 .25 1')
  light=el(w,'light',name='sun',type='directional');el(light,'pose','0 0 50 0 0 0');el(light,'diffuse','1 1 1 1');el(light,'direction','-.3 -.3 -1')
 w.find('physics/real_time_factor').text=str(rtf);v=vehicle(c,session);el(v,'pose',values([start[0],start[1],.02,0,0,start[2]]));w.append(v);write(out/'world.sdf',root);dump(out/'vehicle.json',c)
 return out/'world.sdf'

def generate():
 out=edition('R02');out.mkdir(parents=True,exist_ok=True)
 for task,c in profiles().items():
  dump(out/'vehicles'/f'{task}.json',c);s=E.Element('sdf',version='1.10');s.append(vehicle(c,'preview'));write(out/'models'/f'{task}.sdf',s)
 dump(out/'references.json',dict(husky=['https://www.clearpathrobotics.com/wp-content/uploads/2013/08/HUSKY_DATA_SHEET-v2_editeda.pdf','https://github.com/husky/husky/blob/noetic-devel/husky_description/urdf/husky.urdf.xacro'],jackal=['https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/jackal/user_manual_jackal/','https://github.com/jackal/jackal/blob/noetic-devel/jackal_description/urdf/jackal.urdf.xacro'],dino=dict(path='ros2_ws/src/dino_description/models/dino/model.sdf',sha256=sha(ROOT/'ros2_ws/src/dino_description/models/dino/model.sdf')),scope='Numerical geometry references only, not vendor hardware validation'))
 return out
