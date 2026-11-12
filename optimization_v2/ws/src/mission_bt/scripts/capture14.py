"""Small sensor-only localization diagnostic; no world or truth subscription."""
import sys, argparse, signal
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from common14 import *
import rclpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('s14_sensor_diagnostic');saved=set();terrain_saved=set();perception_saved=set();stop=False
    def cloud(m):
        t=stamp(m);bucket=int(t//10)
        if t<10 or bucket in saved or len(saved)>=4:return
        saved.add(bucket);raw=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=True).reshape(-1,3);np.savez_compressed(a.out/f'cloud_{bucket}.npz',points=raw,stamp=t)
    n.create_subscription(PointCloud2,'/s07/inputs/cloud',cloud,5)
    def terrain(m):
        d=json.loads(m.data);bucket=int(d['t']//10)
        if d['t']<10 or bucket in terrain_saved or len(terrain_saved)>=4:return
        terrain_saved.add(bucket);dump(a.out/f'terrain_{bucket}.json',d)
    def perception(m):
        t=stamp(m);bucket=int(t//10)
        if t<10 or bucket in perception_saved or len(perception_saved)>=4:return
        perception_saved.add(bucket);raw=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=True).reshape(-1,3)
        np.savez_compressed(a.out/f'perception_{bucket}.npz',points=raw,stamp=t,frame=m.header.frame_id)
    n.create_subscription(String,'/s09/terrain/cells',terrain,2)
    n.create_subscription(PointCloud2,'/s04/perception/points',perception,qos_profile_sensor_data)
    def end(s,f):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
    try:
        while not stop:rclpy.spin_once(n,timeout_sec=.05)
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
