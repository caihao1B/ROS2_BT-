"""Store noisy point clouds and NavSatFix, isolated from deployment and truth."""
from common14 import *
import argparse,signal
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,NavSatFix
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('s14_survey_capture');stop=[False];saved=set();f=(a.out/'gnss.jsonl').open('w');q=(a.out/'quality.jsonl').open('w')
    def cloud(m):
        t=stamp(m);bucket=int(t//3)
        if t<9 or bucket in saved or len(saved)>=5:return
        saved.add(bucket);points=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=True).reshape(-1,3)
        np.savez_compressed(a.out/f'cloud_{bucket}.npz',points=points,stamp=t)
    def gps(m):f.write(json.dumps(dict(t=stamp(m),llh=[m.latitude,m.longitude,m.altitude],covariance=list(m.position_covariance),status=int(m.status.status),frame=m.header.frame_id))+'\n');f.flush()
    def quality(m):q.write(m.data+'\n');q.flush()
    n.create_subscription(PointCloud2,'/s07/inputs/cloud',cloud,5)
    n.create_subscription(NavSatFix,'/s04/gnss/fix',gps,qos_profile_sensor_data)
    n.create_subscription(String,'/s04/gnss/quality',quality,qos_profile_sensor_data)
    def end(s,f):stop[0]=True
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,end)
    try:
        while not stop[0]:rclpy.spin_once(n,timeout_sec=.02)
    finally:f.close();q.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
