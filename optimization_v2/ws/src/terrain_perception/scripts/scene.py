"""Simulator-only S09 fixtures. Never imported by online perception nodes."""
from common09 import *
from xml.etree import ElementTree as E
import mapping_runtime as baseline
from generate import el,values,write

def compose(out,kind,case,session):
 baseline.compose(out,kind,'mapping_full',session);root=E.parse(out/'world.sdf').getroot();w=root.find('world');spawn=[30.5,15.3,.15];vehicle=w.find("model[@name='vehicle']");vehicle.find('pose').text=values([*spawn[:2],.025,0,0,spawn[2]])
 R=Rotation.from_euler('z',spawn[2]).as_matrix();shift=np.array([*spawn[:2],0.]);fixtures=[]
 def world(p):return R@np.array(p)+shift
 def box(name,pos,size,color='.6 .35 .15 1',yaw=0):
  m=el(w,'model',name=name);el(m,'static','true');xyz=world(pos);el(m,'pose',values([*xyz,0,0,spawn[2]+yaw]));l=el(m,'link',name='body')
  for tag in ['collision','visual']:
   v=el(l,tag,name=tag);el(el(el(v,'geometry'),'box'),'size',values(size))
   if tag=='visual':mat=el(v,'material');el(mat,'ambient',color);el(mat,'diffuse',color)
  fixtures.append(dict(name=name,local_position=pos,size=size,kind='box'));return m
 for h,y in [(.08,-1.5),(.15,0.),(.30,1.5)]:box('s09_low_'+str(int(h*100)),[3.6,y,h/2],[1.2,.8,h])
 box('s09_tall',[7,-3.,1.1],[1.2,1.,2.2],'.3 .35 .45 1')
 def ramp(name,pos,length,width,degrees):
  h=math.tan(math.radians(degrees))*length;verts=[(0,-width/2,0),(length,-width/2,0),(length,width/2,0),(0,width/2,0),(length,-width/2,h),(length,width/2,h)];faces=[(1,2,3,4),(1,5,2),(4,3,6),(2,5,6,3),(1,4,6,5)];obj=out/(name+'.obj');lines=['v '+' '.join(map(str,p)) for p in verts]
  for fi,f in enumerate(faces,1):
   a,b,c=[np.array(verts[i-1]) for i in f[:3]];normal=np.cross(b-a,c-a);normal=normal/np.linalg.norm(normal);lines.append('vn '+' '.join(map(str,normal)))
   for j in range(1,len(f)-1):lines.append('f '+' '.join(f'{i}//{fi}' for i in [f[0],f[j],f[j+1]]))
  obj.write_text('\n'.join(lines)+'\n');m=el(w,'model',name=name);el(m,'static','true');el(m,'pose',values([*world(pos),0,0,spawn[2]]));l=el(m,'link',name='body')
  for tag in ['collision','visual']:
   v=el(l,tag,name=tag);el(el(el(v,'geometry'),'mesh'),'uri','file://'+str(obj.resolve()))
   if tag=='visual':mat=el(v,'material');el(mat,'ambient','.35 .5 .2 1');el(mat,'diffuse','.4 .55 .23 1')
  fixtures.append(dict(name=name,local_position=pos,size=[length,width,h],degrees=degrees,kind='ramp'))
 ramp('s09_gentle',[1.5,3.3,0],3,1.8,3);ramp('s09_steep',[1.5,-3.3,0],3,1.8,9)
 motion=w.find("plugin[@name='s02::ScriptedMotion']")
 if motion is None:raise RuntimeError('expected existing scripted simulator actor system')
 if case in ['occluded_pedestrian','camera_loss']:
  box('s09_occluder',[3.1,-.1,1.1],[.5,.8,2.2],'.25 .25 .3 1')
  # A licensed human mesh in a scripted rigid-body walking pose. No gait animation claim.
  m=E.parse(BASE/'assets/perception/person/model.sdf').getroot().find('model');m.set('name','s09_person');l=m.find('link');l.set('name','body');el(l,'gravity','false');xyz=world([5.,1.6,.02]);el(m,'pose',values([*xyz,0,0,spawn[2]+math.pi]))
  for uri in m.findall('.//uri'):uri.text='file://'+str(BASE/'assets/perception/person/meshes/walking.dae')
  w.append(m);motion9=el(w,'plugin',filename='libs09_actor.so',name='s09sim::ScriptedMotion');actor=el(motion9,'actor');el(actor,'name','s09_person');el(actor,'center',values(world([5.,0.,.02])));el(actor,'amplitude','0 2.0 0');el(actor,'period',60);el(actor,'phase',.45)
  fixtures.append(dict(name='s09_person',kind='moving_person',source='CC0 OpenRobotics Walking person v3',size=[.6,.7,1.8]))
 # Online prior is still the frozen S02 map; these temporary objects are NOT exported to prior.
 write(out/'world.sdf',root);dump(out/'fixtures.json',dict(spawn=spawn,objects=fixtures,evaluation_only=True,low_removal_sim_s=35.,case=case));return [2.,2.,0.]
