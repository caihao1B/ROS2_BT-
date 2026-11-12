"""Measured near-field supplement; original localization lidar remains untouched.

Both optical heads are collocated in this simulated commissioning model. Gazebo
adds 1 cm range noise; this adapter drops 2% and delays paired data by 40 ms.
Only sensor messages are inputs. Missing pairs publish no cloud (fail closed).
"""
from common13 import *
import argparse,signal,copy
from collections import deque
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py.point_cloud2 import read_points_numpy

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 # Compose with the qualified S04 fanout in one ROS participant. Localization,
 # stereo pairing and static TF remain its original implementation.
 sys.path.insert(0,str(BASE/'ws/src/sensor_sim_adapter/scripts'))
 from fanout import Fanout
 rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);fanout=Fanout();n=fanout.n
 n.destroy_publisher(fanout.pp)
 pub=n.create_publisher(PointCloud2,'/s04/perception/points',qos_profile_sensor_data)
 near=deque(maxlen=8);pending=deque(maxlen=3);main_queue=deque(maxlen=3);st={'stop':False};rng=np.random.default_rng(41);log=(a.out/'events.jsonl').open('w')
 class PerceptionQueue:
  def publish(self,msg):main_queue.append(msg)
 fanout.pp=PerceptionQueue()
 n.create_subscription(PointCloud2,'/s13/near/raw',lambda msg:near.append(msg),qos_profile_sensor_data)
 def cloud(main):
  if not near:return
  extra=min(near,key=lambda m:abs(stamp(m)-stamp(main)))
  if abs(stamp(extra)-stamp(main))>.055:return
  xyz=read_points_numpy(extra,field_names=['x','y','z'],skip_nans=True).reshape(-1,3)
  xyz=xyz[np.isfinite(xyz).all(axis=1)];xyz=xyz[rng.random(len(xyz))>=.02]
  original=read_points_numpy(main,field_names=['x','y','z'],skip_nans=True).reshape(-1,3)
  points=np.concatenate([original,xyz]).astype('<f4');msg=PointCloud2();msg.header=copy.deepcopy(main.header);msg.height=1;msg.width=len(points);msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(['x','y','z'])];msg.is_bigendian=False;msg.point_step=12;msg.row_step=msg.width*12;msg.is_dense=True;msg.data=points.tobytes();pending.append((msg,len(xyz),abs(stamp(extra)-stamp(main))))
 def tick():
  t=n.get_clock().now().nanoseconds*1e-9
  while main_queue and near:
   main=main_queue[0]
   if abs(stamp(min(near,key=lambda msg:abs(stamp(msg)-stamp(main))))-stamp(main))<=.055:
    cloud(main_queue.popleft())
   elif t-stamp(main)>.20:main_queue.popleft()
   else:break
  while pending and t-stamp(pending[0][0])>=.04:
   msg,count,dt=pending.popleft();age=t-stamp(msg)
   if age>.30:continue
   pub.publish(msg);log.write(json.dumps(dict(t=stamp(msg),delivered_t=t,near_points=count,total_points=msg.width,pair_delta_s=dt,age_s=age))+'\n');log.flush()
 n.create_timer(.02,tick)
 def stop(s,f):st['stop']=True
 for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,stop)
 (a.out/'ready').write_text('supplementary observed near cloud; no truth; noise/dropout/delay')
 try:
  while not st['stop']:rclpy.spin_once(n,timeout_sec=.01)
 finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
