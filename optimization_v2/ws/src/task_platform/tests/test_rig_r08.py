import sys,unittest,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import read,edition,canonical
from rig_r08 import effective_rig,validate

class SamplingContract(unittest.TestCase):
 def setUp(self):self.base=read(edition('R05')/'inspection/sensors.json')
 def test_actual_density_is_separate_revision(self):
  old=copy.deepcopy(self.base);rig=effective_rig(self.base,{'units':[]})
  self.assertEqual(validate(self.base,rig),rig);self.assertEqual(self.base,old)
  self.assertEqual(rig['lidar']['horizontal']*rig['lidar']['vertical'],262144)
  self.assertEqual(rig['lidar']['sigma_m'],old['lidar']['sigma_m']);self.assertNotEqual(rig['revision'],old['revision'])
 def test_tampered_extrinsic_rejected_even_with_new_hash(self):
  rig=effective_rig(self.base,{'units':[]});rig['gnss']['xyz'][0]+=.1;rig['revision']=canonical({k:v for k,v in rig.items() if k!='revision'})
  with self.assertRaisesRegex(ValueError,'CALIBRATION'):validate(self.base,rig)
 def test_unbound_change_rejected(self):
  rig=effective_rig(self.base,{'units':[]});rig['lidar']['horizontal']=1024
  with self.assertRaisesRegex(ValueError,'HASH'):validate(self.base,rig)
