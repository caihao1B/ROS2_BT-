import sys,unittest,os,copy,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import *
from contract_r05 import verify_runtime_contract
from rig_r05 import overlay
class Rig(unittest.TestCase):
 def test_01_sensor_geometry_and_error_models(self):
  import yaml
  original=yaml.safe_load((BASE/'config/sensors.yaml').read_text())
  for t in TASKS:
   c=read(edition('R04')/'vehicles'/f'{t}.json');s=read(edition('R05')/t/'sensors.json');top=c['body']['center'][2]+c['body']['size'][2]/2
   self.assertGreater(s['lidar']['xyz'][2],top+.1);self.assertEqual(s['lidar']['horizontal'],1024);self.assertEqual(s['lidar']['vertical'],64);self.assertEqual(s['stereo']['baseline_m'],.12)
   for name in ['lidar','stereo','imu','gnss']:
    for key,value in original[name].items():
     if key!='xyz' and not (name=='lidar' and key=='vertical'):self.assertEqual(s[name][key],value)
 def test_02_self_mask_encloses_chassis(self):
  for t in TASKS:
   c=read(edition('R04')/'vehicles'/f'{t}.json');mask=read(edition('R05')/t/'sensors.json')['self_filter']
   for x,y in c['footprint']:self.assertTrue(mask['x_min']<x<mask['x_max'] and mask['y_min']<y<mask['y_max'])
 def test_03_frozen_source_and_new_rig_contract(self):
  for t in TASKS:
   b=read(edition('R05')/t/'binding.json');os.environ['R05_BINDING']=str(edition('R05')/t/'binding.json');r=dict(map_id=b['source_map_id'],rig_revision=b['rig_revision'],rig_binding_revision=b['revision']);m=verify_runtime_contract(r);self.assertEqual(m['source_manifest_revision'],b['source_revision']);r['rig_revision']='wrong'
   with self.assertRaises(ValueError):verify_runtime_contract(r)
 def test_04_exact_adapter_generation(self):
  for t in TASKS:
   with tempfile.TemporaryDirectory() as d:
    p=overlay(t,Path(d));self.assertIn('verify_runtime_contract',(p/'quality.py').read_text());self.assertNotIn('+[.1,0,.78]',(p/'preprocessor.py').read_text());self.assertEqual(len(read(p/'adaptation.json')['generated']),8);self.assertIn('(np.linalg.norm(raw,axis=1)<35.0)',(p/'regional_math.py').read_text())
 def test_05_online_probe_no_truth_access(self):
  s=(PKG/'scripts/observer_r05.py').read_text();self.assertNotIn('/s03/evaluation',s);self.assertNotIn('/s04/raw',s);self.assertNotIn('fixture.json',s);self.assertIn("state['probe_phase']='GEAR_STOP'",s)
if __name__=='__main__':unittest.main()
