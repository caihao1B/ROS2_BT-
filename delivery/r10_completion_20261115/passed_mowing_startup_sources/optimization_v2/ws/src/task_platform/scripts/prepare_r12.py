"""Explicit isolated recording runtimes derived from qualified task orchestration."""
from common import *

def recording_source(text,scene_name,stage):
 text=text.replace('from '+scene_name+' import compose','from '+scene_name+' import compose as original_compose')
 wrapper='''
def compose(out,task,session,**kwargs):
 from visual_r11 import apply,camera
 c=original_compose(out,task,session,**kwargs)
 original=out/'untextured.sdf';(out/'world.sdf').replace(original)
 apply(original,out/'world.sdf',task);camera(out/'world.sdf',task)
 dump(out/'recording_camera.json',dict(topic='/r12/observer/image',domain=120,autonomy_input=False,source='actual renderer',material_geometry_contract=str(out/'world.visual.json')))
 return c

'''
 text=text.replace('\ndef run(',wrapper+'\ndef run(',1).replace("APP/'results/"+stage+"'","APP/'results/R12'")
 anchor="spec['implementation_bindings']={"
 assert anchor in text
 text=text.replace(anchor,"bound += [Path(__file__),PKG/'scripts/record_r12.py',PKG/'scripts/visual_r11.py']+list((edition('R11')/'assets').glob('*'));"+anchor)
 insertion="""  launch('video_bridge',[bridge,'/r12/observer/image@sensor_msgs/msg/Image[gz.msgs.Image'],ev)
  launch('video_recorder',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/record_r12.py'),'--out',str(out/'video')],ev)
"""
 anchor="  for mode in ['motion','vision']:"
 assert anchor in text;text=text.replace(anchor,insertion+anchor,1).replace("ready=['generation_motion/ready'","ready=['video/ready','generation_motion/ready'",1)
 anchor='  time.sleep(.5)\n  from shutdown_r08 import stop_ros'
 assert anchor in text
 text=text.replace(anchor,"  gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'])\n  time.sleep(.5)\n  from shutdown_r08 import stop_ros")
 return text

def build():
 out=edition('R12');out.mkdir(parents=True,exist_ok=True);records=[]
 for task,source_name,scene_name,stage in [('spray','runtime_spray_r10.py','scene_spray_r10','R10'),('inspection','runtime_r08.py','scene_r08','R08'),('mowing','runtime_mowing_r10.py','scene_mowing_r10','R10')]:
  source=PKG/'scripts'/source_name;text=recording_source(source.read_text(),scene_name,stage)
  target=out/('runtime_'+task+'.py');target.write_text(text);compile(text,str(target),'exec');records.append(dict(task=task,source=str(source.relative_to(ROOT)),source_sha256=sha(source),derived_runtime=str(target.relative_to(ROOT)),derived_sha256=sha(target),changes=['isolated R12 output','material-only scene','observer camera in evaluation domain','actual video recorder','pause after physical stop before ROS teardown']))
 dump(out/'runtime_derivation.json',dict(records=records,no_task_or_gate_changes=True));return records
