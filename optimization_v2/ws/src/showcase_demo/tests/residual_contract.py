import unittest,sys,json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import numpy as np
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/acceptance_runner/scripts'))
from common17 import add14
add14()
import common14
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from residual_showcase import capture_grid,candidate,sha

class ResidualContracts(unittest.TestCase):
    def mission(self,stamp=1.,valid=True):
        grid=NS(header=NS(frame_id='map',stamp=NS(sec=int(stamp),nanosec=round((stamp%1)*1e9))),data=[-1,78,95,100],info=NS(height=2,width=2,resolution=.1,origin=NS(position=NS(x=0.,y=0.))))
        return NS(st={'t':1.,'global_map':grid},platform_ok=valid,patches=0)
    def test_stale_or_untrusted_observation_cannot_authorize_detour(self):
        with TemporaryDirectory() as d:
            for m in [self.mission(.2),self.mission(valid=False)]:
                with self.assertRaisesRegex(ValueError,'NOT_ADMITTED'):capture_grid(m,{},Path(d))
    def test_unknown_and_live_high_cost_are_never_cleared(self):
        with TemporaryDirectory() as d:
            request=capture_grid(self.mission(),{},Path(d));grid=np.load(request['observed_grid'])
            self.assertEqual(grid['data'].tolist(),[[-1,78],[100,100]])
            self.assertEqual(request['observed_grid_sha256'],sha(Path(request['observed_grid'])))
    def test_changed_snapshot_is_rejected(self):
        with TemporaryDirectory() as d:
            request=capture_grid(self.mission(),{},Path(d));Path(request['observed_grid']).write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'GRID_CHANGED'):candidate(request)

if __name__=='__main__':unittest.main()
