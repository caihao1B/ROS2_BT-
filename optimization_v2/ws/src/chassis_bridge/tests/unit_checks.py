"""Geometry, actuator policy and legacy threshold regression tests."""
import json,math,sys
from pathlib import Path
from xml.etree import ElementTree as E
import numpy as np
from shapely.geometry import Polygon,Point,box
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2'
sys.path.insert(0,str(BASE/'ws/src/chassis_bridge/scripts'));from kinematics import limit,steering,curvature,encoder_delta
sys.path.insert(0,str(BASE/'ws/src/robot_description/scripts'));from generate import config,canonical
def run():
    canonical();result=dict(checks={},geometry={});legacy=json.loads((ROOT/'native_20261108/ros2_ws/src/dino_control/config/control.json').read_text())
    for kind in ['diff','ackermann']:
        c=config(kind);f=Polygon(c['footprint']);assert f.is_valid
        b=c['body'];x,y,z=b['center'];sx,sy,sz=b['size'];assert f.covers(box(x-sx/2,y-sy/2,x+sx/2,y+sy/2))
        t=c['tool'];x,y,z=t['mount_xyz'];sx,sy,sz=t['collision_size'];assert f.covers(box(x-sx/2,y-sy/2,x+sx/2,y+sy/2))
        wheel=c['wheel'];rad=wheel['radius'];width=wheel['width'];track=wheel['track'];L=c['wheelbase']
        for angle in np.linspace(-c['steering']['joint_limit_rad'],c['steering']['joint_limit_rad'],41) if kind=='ackermann' else [0]:
            for x in ([0,L] if kind=='ackermann' else [0]):
                for y in [-track/2,track/2]:
                    for dx in [-rad,rad]:
                        for dy in [-width/2,width/2]:
                            a=angle if x==L and kind=='ackermann' else 0
                            assert f.covers(Point(x+math.cos(a)*dx-math.sin(a)*dy,y+math.sin(a)*dx+math.cos(a)*dy))
        sdf=E.parse(BASE/f'ws/src/robot_description/generated/{kind}.sdf');m=sdf.find('model');mass=0
        for link in m.findall('link'):
            i=link.find('inertial');mass+=float(i.findtext('mass'));inertia=i.find('inertia');d=[float(inertia.findtext(k)) for k in ['ixx','iyy','izz']];assert min(d)>0 and max(d)<=sum(d)-max(d)+1e-8
        assert m.find("joint[@name='rear_left_joint']") is not None
        if kind=='diff':
            # Three-point support must contain the combined COM, including slope/accel shift.
            support=Polygon([(0,-track/2),(0,track/2),(c['support']['caster_x'],0)])
            weighted=np.zeros(3)
            for link in m.findall('link'):
                i=link.find('inertial');offset=np.array(list(map(float,link.findtext('pose','0 0 0').split()[:3])))
                offset+=np.array(list(map(float,i.findtext('pose','0 0 0').split()[:3])))
                weighted+=float(i.findtext('mass'))*offset
            com=weighted/mass;shift=com[2]*(math.tan(math.radians(5))+.5/9.81)
            assert support.contains(Point(com[:2])) and support.contains(Point(com[0]-shift,com[1]))
        if kind=='ackermann':
            for side in ['left','right']:assert m.find(f"joint[@name='steer_{side}_joint']").get('type')=='revolute'
            for k in [-.5,-.2,0,.2,.5]:assert abs(curvature(*steering(k,c),c)-k)<1e-12
            assert limit(0,1,c)[:2]==(0.,0.)
        assert limit(float('nan'),0,c)[:2]==(0.,0.)
        assert c['safety']['command_wall_s']<=legacy['command_timeout_wall_s'];assert c['safety']['feedback_wall_s']<=legacy['feedback_timeout_wall_s'];assert c['safety']['actuator_wall_s']<=legacy['actuator_timeout_wall_s'];assert c['safety']['actuator_sim_s']<=legacy['actuator_timeout_sim_s']
        assert c['safety']['stopped_v_mps']<=.01 and c['safety']['stopped_w_rad_s']<=.02 and c['safety']['stop_delay_s']<=2
        result['geometry'][kind]=dict(mass_kg=mass,footprint=c['footprint'],tool=t,physical_track_m=track,calibration=c.get('calibration'),config_hash=c['config_hash'])
        result['checks'][kind+'_geometry_kinematics_safety']='PASS'
    assert config('ackermann')['tool']['virtual_effective_width_m']<=config('ackermann')['tool']['collision_size'][1]
    result['passed']=True;return result
if __name__=='__main__':print(json.dumps(run(),indent=2))
