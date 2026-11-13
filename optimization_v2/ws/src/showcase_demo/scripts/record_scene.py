"""Observer camera on the simulated vehicle; its images never enter autonomy."""
from common_showcase import *
from xml.etree import ElementTree as E

def apply(out):
    path=out/'world.sdf';root=E.parse(path);world=root.find('world');body=world.find("model[@name='vehicle']/link[@name='base_link']")
    if body is None:raise ValueError('OBSERVER_CAMERA_PARENT_MISSING')
    # Camera forward axis is +X. Positive pitch points down; the offset sees
    # the vehicle, its immediate road and nearby actors from behind its left.
    body.append(E.fromstring('''<sensor name="showcase_observer" type="camera"><pose>-4 -6 7 0 0.71 0.83</pose><update_rate>10</update_rate><always_on>true</always_on><topic>/showcase/observer/image</topic><camera><horizontal_fov>1.25</horizontal_fov><image><width>960</width><height>540</height><format>R8G8B8</format></image><clip><near>0.2</near><far>180</far></clip></camera></sensor>'''))
    root.write(path,encoding='unicode');dump(out/'recording_camera.json',dict(topic='/showcase/observer/image',ros_domain=120,resolution=[960,540],sim_rate_hz=10,role='visual evidence only',autonomy_input=False,geometry_and_physics_unchanged=True))
