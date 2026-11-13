"""Regression for different DDS callback orders and stale/future geometry."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from temporal_join17 import TerrainJoin

class TemporalJoin(unittest.TestCase):
    def test_both_arrival_orders_have_same_result(self):
        for order in ['gh','hg']:
            j=TerrainJoin()
            for x in order:
                if x=='g':j.grid(10.,'diff','observed_grid')
                else:j.status(dict(t=10.,valid=True))
            self.assertEqual(j.take(10.2),[('observed_grid','diff',dict(t=10.,valid=True))]);self.assertEqual(j.take(10.2),[])
    def test_new_health_does_not_validate_old_grid(self):
        j=TerrainJoin();j.grid(10.,'diff','grid');j.status(dict(t=10.1,valid=True));self.assertEqual(j.take(10.2),[])
    def test_invalid_matching_health_is_preserved(self):
        j=TerrainJoin();j.grid(10.,'diff','grid');j.status(dict(t=10.,valid=False));self.assertFalse(j.take(10.2)[0][2]['valid'])
    def test_expired_and_future_frames_cannot_be_admitted(self):
        j=TerrainJoin();j.grid(10.,'diff','grid');j.status(dict(t=10.,valid=True));self.assertEqual(j.take(9.9),[]);self.assertEqual(j.take(10.451),[])
    def test_bounded_buffers(self):
        j=TerrainJoin()
        for t in range(100):j.grid(float(t),'diff',object());j.status(dict(t=float(t),valid=True))
        self.assertLessEqual(len(j.grids),4);self.assertLessEqual(len(j.health),4)
if __name__=='__main__':unittest.main()
