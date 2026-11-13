import sys,unittest,tempfile
from pathlib import Path
from xml.etree import ElementTree as E
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from visual18 import apply,physical_signature,BASE
sys.path.insert(0,str(BASE/'ws/src/robot_description/scripts'))
from generate import world

class VisualContract(unittest.TestCase):
    def test_both_chassis_keep_all_nonvisual_elements(self):
        for kind in ['diff','ackermann']:
            with tempfile.TemporaryDirectory(dir=BASE/'runtime') as tmp:
                d=Path(tmp);world(d,kind,'offline_contract',[70,12,0],1)
                before=physical_signature(E.parse(d/'world.sdf').getroot())
                r=apply(d/'world.sdf',d/'skin.sdf')
                self.assertTrue(r['nonvisual_unchanged'])
                self.assertEqual(before,physical_signature(E.parse(d/'skin.sdf').getroot()))
                self.assertIn('vehicle shell, wheels, vents, reflectors, tool',r['changed'])
                self.assertEqual(r['sensor_equivalence'],'NOT_QUALIFIED')
    def test_source_never_overwritten(self):
        p=BASE/'assets/worlds/irregular_farm.sdf'
        with self.assertRaises(ValueError):apply(p,p)

if __name__=='__main__':unittest.main()
