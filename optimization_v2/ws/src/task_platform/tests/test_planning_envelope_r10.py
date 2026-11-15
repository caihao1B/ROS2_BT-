import unittest,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from planning_envelope_r10 import envelope

class PlanningEnvelope(unittest.TestCase):
    def test_nav_padding_raster_and_pose_perturbations_fit(self):
        fp=[[-.571,-1.385],[2.209,-1.385],[2.209,1.385],[-.571,1.385]]
        p,d=envelope(fp,.1,.15);low=p[0][0];high=p[1][0];half=p[2][1]
        for yaw in [i*.001 for i in range(-120,121)]:
            for x,y in [(-.671,-1.485),(2.309,-1.485),(2.309,1.485),(-.671,1.485)]:
                for direction in range(0,360,5):
                    theta=math.radians(direction);radius=.25+.15*math.sqrt(2)/2
                    xx=math.cos(yaw)*x-math.sin(yaw)*y+radius*math.cos(theta)
                    yy=math.sin(yaw)*x+math.cos(yaw)*y+radius*math.sin(theta)
                    self.assertLessEqual(low,xx+1e-10);self.assertGreaterEqual(high,xx-1e-10);self.assertLessEqual(abs(yy),half+1e-10)
        self.assertAlmostEqual(d['lateral_extra_m'],.75)
    def test_old_point_two_margin_does_not_cover_grid_and_heading(self):
        padded_y=math.sin(.12)*2.309+math.cos(.12)*1.485+.25+.15*math.sqrt(2)/2
        self.assertGreater(padded_y,1.385+.2)
    def test_invalid_design_rejected(self):
        with self.assertRaises(ValueError):envelope([[-1,-1],[1,1]],.1,float('nan'))

if __name__=='__main__':unittest.main()
