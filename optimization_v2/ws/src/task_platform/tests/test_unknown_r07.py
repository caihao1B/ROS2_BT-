"""The installed observed-risk plugin must retain unknowns with strict RPP policy."""
import unittest,subprocess,sys,json
from pathlib import Path
class UnknownGround(unittest.TestCase):
 def test_production_layer_unknown_contract(self):
  root=Path(__file__).resolve().parents[5];base=root/'optimization_v2';source=base/'ws/src/terrain_perception/tests/nav2_layer.py';out=base/'three_tasks/results/R07/unknown_policy_test';out.mkdir(parents=True,exist_ok=True)
  original=source.read_text();assert "track_unknown_space=True" in original
  script=original.replace("'123'","'126'").replace('track_unknown_space=True','track_unknown_space=False').replace("Path(__file__).resolve().parents[1]/'scripts'",repr(str(source.parents[1]/'scripts')))
  p=out/'run.py';p.write_text(script)
  with (out/'test.log').open('w') as f:r=subprocess.run([sys.executable,str(p),str(out)],stdout=f,stderr=subprocess.STDOUT,timeout=25)
  self.assertEqual(r.returncode,0,(out/'test.log').read_text());result=json.loads((out/'result.json').read_text());self.assertTrue(result['passed']);self.assertEqual(result['exit_code'],0)
if __name__=='__main__':unittest.main()
