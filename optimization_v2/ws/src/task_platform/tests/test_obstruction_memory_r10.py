import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from obstruction_memory_r10 import ObstructionMemory

class ObservedObstruction(unittest.TestCase):
 def test_unknown_does_not_fabricate_or_clear_obstruction(self):
  m=ObstructionMemory();self.assertFalse(m.update(0,False,False))
  self.assertTrue(m.update(1,True,True));self.assertTrue(m.update(20,False,False))
 def test_only_continuous_fresh_clearance_releases(self):
  m=ObstructionMemory();m.update(0,True,True)
  self.assertTrue(m.update(1,True,False));self.assertTrue(m.update(1.9,True,False));self.assertTrue(m.update(2,False,False))
  self.assertTrue(m.update(10,True,False));self.assertTrue(m.update(10.99,True,False));self.assertFalse(m.update(11,True,False))
 def test_clock_rewind_does_not_credit_elapsed_clearance(self):
  m=ObstructionMemory();m.update(5,True,True);m.update(6,True,False)
  self.assertTrue(m.update(1,True,False));self.assertFalse(m.update(2,True,False))
 def test_new_obstacle_restarts_clearance(self):
  m=ObstructionMemory();m.update(0,True,True);m.update(1,True,False);m.update(1.9,True,True)
  self.assertTrue(m.update(2,True,False));self.assertTrue(m.update(2.9,True,False));self.assertFalse(m.update(3,True,False))

if __name__=='__main__':unittest.main()
