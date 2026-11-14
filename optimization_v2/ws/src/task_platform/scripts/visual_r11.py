"""Material-only presentation. Collision, sensor poses and visual geometry stay exact."""
from common import *
import copy
from xml.etree import ElementTree as E

ASSETS=edition('R11')/'assets'

def signature(root):
 root=copy.deepcopy(root)
 for visual in root.findall('.//visual'):
  material=visual.find('material')
  if material is not None:visual.remove(material)
 return __import__('hashlib').sha256(E.canonicalize(E.tostring(root,encoding='unicode'),strip_text=True).encode()).hexdigest()

def assets():
 from PIL import Image,ImageDraw,ImageFont
 ASSETS.mkdir(parents=True,exist_ok=True)
 for task,color in [('spray',(29,91,135)),('inspection',(201,139,37)),('mowing',(26,102,66))]:
  im=Image.new('RGB',(512,512),color);d=ImageDraw.Draw(im)
  d.rounded_rectangle((16,16,496,496),radius=24,outline=(190,199,198),width=6)
  d.rectangle((32,44,480,120),fill=(33,41,47));font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',34);d.text((45,60),task.upper(),font=font,fill=(237,243,239))
  for y in range(180,390,24):d.rounded_rectangle((100,y,412,y+8),radius=3,fill=(21,31,34))
  for x in [34,474]:
   for y in [34,474]:d.ellipse((x-4,y-4,x+4,y+4),fill=(207,216,214))
  im.save(ASSETS/(task+'.png'))
 (ASSETS/'PROVENANCE.txt').write_text('Original deterministic Python/Pillow panel textures. No third-party logo or downloaded artwork. Geometry and non-material SDF nodes are unchanged. Camera pixels change and require recorded replay qualification.\n')

def material(visual,color,texture=None,metal=.0,rough=.8):
 old=visual.find('material')
 if old is not None:visual.remove(old)
 m=E.SubElement(visual,'material')
 for name in ['ambient','diffuse']:E.SubElement(m,name).text=color
 pbr=E.SubElement(E.SubElement(m,'pbr'),'metal')
 E.SubElement(pbr,'metalness').text=str(metal);E.SubElement(pbr,'roughness').text=str(rough)
 if texture:E.SubElement(pbr,'albedo_map').text=str(texture)

def apply(source,destination,task):
 source=Path(source);destination=Path(destination)
 if source.resolve()==destination.resolve():raise ValueError('ISOLATED_MATERIAL_OUTPUT_REQUIRED')
 root=E.parse(source).getroot();before=signature(root);changed=0
 for model in root.findall('world/model'):
  name=model.get('name','')
  for link in model.findall('link'):
   for v in link.findall('visual'):
    if name=='vehicle':
     if link.get('name')=='base_link':material(v,'1 1 1 1',ASSETS/(task+'.png'),.22,.42)
     elif 'wheel' in link.get('name',''):material(v,'.035 .041 .048 1',rough=.93)
     elif 'tool' in link.get('name',''):material(v,'.80 .55 .12 1',metal=.18,rough=.54)
     else:material(v,'.22 .27 .31 1',metal=.4,rough=.4)
    elif name=='ground':material(v,'.18 .25 .14 1',rough=1.)
    elif name=='mowing_workspace' or name.startswith('grade_'):material(v,'.31 .43 .21 1',rough=1.)
    elif name=='spray_workspace':material(v,'.40 .32 .22 1',rough=1.)
    elif name=='connector':material(v,'.38 .41 .40 1',rough=.98)
    elif name.startswith(('ped_','r08_leader')):material(v,'.92 .52 .12 1',rough=.82)
    elif 'low' in name or 'barrier' in name:material(v,'.53 .49 .39 1',rough=.96)
    elif name in ['shed','pump_house','utility']:material(v,'.57 .61 .58 1',rough=.85)
    else:continue
    changed+=1
 if signature(root)!=before:raise ValueError('GEOMETRY_OR_PHYSICS_CHANGED')
 destination.parent.mkdir(parents=True,exist_ok=True);E.indent(root);E.ElementTree(root).write(destination,encoding='utf-8',xml_declaration=True)
 if signature(E.parse(destination).getroot())!=before:raise ValueError('SERIALIZED_CONTRACT_CHANGED')
 result=dict(task=task,changed_materials=changed,geometry_sensor_physics_signature=before,geometry_and_physics_unchanged=True,source_sha256=sha(source),destination_sha256=sha(destination),sensor_pixels='CHANGED_REQUIRES_SEPARATE_QUALIFICATION',assets={str(p.relative_to(ROOT)):sha(p) for p in ASSETS.iterdir() if p.is_file()})
 dump(destination.with_suffix('.visual.json'),result);return result

def camera(path,task):
 root=E.parse(path);body=root.find("world/model[@name='vehicle']/link[@name='base_link']")
 # Dedicated observer only; physical robot sensors and online data domains stay unchanged.
 offset='-4.8 -5.2 5.5 0 .65 .83' if task=='mowing' else '-2.6 -3.0 3.5 0 .66 .83'
 body.append(E.fromstring(f'<sensor name="r12_observer" type="camera"><pose>{offset}</pose><update_rate>8</update_rate><always_on>true</always_on><topic>/r12/observer/image</topic><camera><horizontal_fov>1.1</horizontal_fov><image><width>960</width><height>540</height><format>R8G8B8</format></image><clip><near>.1</near><far>180</far></clip></camera></sensor>'))
 root.write(path,encoding='utf-8',xml_declaration=True)
