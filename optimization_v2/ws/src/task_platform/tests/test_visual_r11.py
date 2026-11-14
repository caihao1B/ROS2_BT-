import sys,unittest,copy
from pathlib import Path
from xml.etree import ElementTree as E
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from visual_r11 import signature

class MaterialContract(unittest.TestCase):
 def setUp(self):self.root=E.fromstring('<sdf><world><model><link><collision><geometry><box><size>1 2 3</size></box></geometry></collision><sensor type="camera"><pose>0 0 1 0 0 0</pose></sensor><visual><geometry><box><size>1 2 3</size></box></geometry><material><diffuse>1 0 0 1</diffuse></material></visual></link></model></world></sdf>')
 def test_material_is_only_ignored_change(self):
  before=signature(self.root);self.root.find('.//diffuse').text='0 0 1 1';self.assertEqual(before,signature(self.root))
 def test_collision_change_is_detected(self):
  before=signature(self.root);self.root.find('.//collision/geometry/box/size').text='2 2 3';self.assertNotEqual(before,signature(self.root))
 def test_sensor_extrinsic_is_detected(self):
  before=signature(self.root);self.root.find('.//sensor/pose').text='0 0 2 0 0 0';self.assertNotEqual(before,signature(self.root))
 def test_visual_occlusion_geometry_is_detected(self):
  before=signature(self.root);self.root.find('.//visual/geometry/box/size').text='1 2 4';self.assertNotEqual(before,signature(self.root))
