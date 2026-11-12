"""Height projection preserves low obstacles, trunk and unobserved ground."""
import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
from collision_volume14 import envelope,hazardous
from regional_occupancy14 import build
from PIL import Image

def main():
    e=envelope();assert e['ceiling_m']==1.1,e
    h=np.array([.06,.5,1.1,2.4,-.2]);assert hazardous(h,1.,e['ceiling_m']).tolist()==[True,True,True,False,True]
    x,y=np.meshgrid(np.linspace(-2,2,30),np.linspace(-2,2,30));ground=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    # Isolated above-ceiling observations must not turn their ground cell free.
    probes=np.array([[8.05,8.05,2.4],[3.05,4.05,.06],[4.05,4.05,.6],[5.05,4.05,-.2]])
    cloud=np.vstack([ground,probes]);before=cloud.copy()
    with tempfile.TemporaryDirectory(dir=BASE/'results/S14') as directory:
        out=Path(directory);build([dict(name='synthetic',clouds=[cloud]*3)],{'synthetic':[0.,0.,0.]},out,'test')
        cfg=yaml.safe_load((out/'test_navigation.yaml').read_text());a=np.asarray(Image.open(out/'test_navigation.pgm'))[::-1]
        def cell(x,y):
            col,row=np.floor((np.array([x,y])-cfg['origin'][:2])/cfg['resolution']).astype(int);return int(a[row,col])
        assert cell(8.05,8.05)==205,'Overhead-only cell was falsely cleared'
        assert all(cell(x,y)==0 for x,y in probes[1:,:2]),'Low/trunk/negative obstacles lost'
        assert cell(0.,0.)==254
        assert np.array_equal(cloud,before),'Full point cloud changed'
    print(json.dumps(dict(passed=True,envelope=e,low_obstacle_retained=True,overhead_only_unknown=True,full_cloud_unchanged=True)))
if __name__=='__main__':main()
