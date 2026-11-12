"""Run the real residual planner off the callback thread, then cancel a new job."""
import sys,os,tempfile,time,argparse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
os.environ['S14_REGISTERED_REGION']='B'
from common14 import *
from mission14 import Mission
from geometry_msgs.msg import PoseWithCovarianceStamped
from shapely.geometry import shape,Polygon
from shapely.affinity import translate

with tempfile.TemporaryDirectory(prefix='s14_residual_') as directory:
    m=Mission.__new__(Mission);m.a=argparse.Namespace(out=Path(directory),vehicle='ackermann');m.p=cluster('ackermann')
    m.c={'plan_revision':plan()['plan_revision']};m.area=translate(shape(m.p['coverage_check']['covered']),xoff=-28,yoff=-14);m.credit=Polygon();m.patches=0
    pose=PoseWithCovarianceStamped();start=m.p['start'];pose.pose.pose.orientation.z=math.sin(start[2]/2);pose.pose.pose.orientation.w=math.cos(start[2]/2)
    m.st={'pose':pose};events=[];m.api={'xy':lambda:[start[0]-28,start[1]-14],'log':lambda event,**kw:events.append(dict(event=event,**kw))}
    m.patch_process=None;m.patch_log=None;m.patch_output=None;m.patch_candidate=None
    t=time.monotonic();m.start_patch();dispatch=time.monotonic()-t;assert dispatch<.5
    cycles=0
    while m.patch_process is not None:
        m.poll_patch();cycles+=1;time.sleep(.02)
        assert time.monotonic()-t<45
    assert m.patch_candidate and m.patch_candidate['expected_gain_m2']>0,events
    assert not m.planning and m.patch_log is None and cycles>1
    m.start_patch();child=m.patch_process;m.cancel_patch()
    assert child.poll() is not None and m.patch_process is None and m.patch_log is None and not m.planning
    print(json.dumps(dict(real_candidate=True,expected_gain_m2=m.patch_candidate['expected_gain_m2'],cost_s=m.patch_candidate['cost_s'],dispatch_wall_s=dispatch,event_loop_cycles=cycles,cancel_reaped=True)))
