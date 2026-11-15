"""Stationary diagnostic only. No navigation action or work credit."""
import argparse,json,time,signal,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);out=ap.parse_args().out;spec=json.loads((out/'spec.json').read_text());rclpy.init();n=rclpy.create_node('r08_stationary_diagnostic',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);st=dict(t=0.,stop=False,loc=None,gate=None,hinted=False,saved=-100.);pub=n.create_publisher(String,'/r07/mission',10);hp=n.create_publisher(PoseWithCovarianceStamped,'/s07/rtabmap/initialpose',10);samples=[];wall=time.monotonic();capture=out/'probe_clouds';capture.mkdir()
 n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
 for topic,key in [('/r07/localization','loc'),('/r07/gate','gate')]:n.create_subscription(String,topic,lambda m,k=key:st.update({k:json.loads(m.data)}),10)
 def tick():
  d=dict(t=st['t'],phase='DIAGNOSTIC_PARKED',nav_idle=True,complete=False,nav_failed=False,replans=0,deferrals=0,alternative=False,index=0,patch_active=False,path_epoch=0.);pub.publish(String(data=json.dumps(d)))
  if st['t']>8 and not st['hinted']:
   p=spec['operator_hint'];h=PoseWithCovarianceStamped();h.header.frame_id='map';h.pose.pose.position.x=p[0];h.pose.pose.position.y=p[1];h.pose.pose.orientation.z=math.sin(p[2]/2);h.pose.pose.orientation.w=math.cos(p[2]/2);h.pose.covariance=[1. if i in [0,7,14] else .25 if i in [21,28,35] else 0. for i in range(36)];hp.publish(h);st['hinted']=True
  d.update(loc=st['loc'],gate=st['gate']);tmp=out/'live.tmp';tmp.write_text(json.dumps(d));tmp.replace(out/'live.json')
  if st['t']>25 and st['loc']:samples.append(bool(st['loc'].get('valid')))
 def cloud(m):
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if t>25 and t-st['saved']>=5:
   st['saved']=t;x=read_points_numpy(m,field_names=['x','y','z'],skip_nans=True).reshape(-1,3);np.savez_compressed(capture/f'raw_{t:.3f}.npz',raw=x,t=t)
 n.create_subscription(PointCloud2,'/s04/localization/points',cloud,qos_profile_sensor_data);n.create_timer(.03,tick)
 for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,lambda s,f:st.update(stop=True))
 (out/'observer_ready').write_text('stationary sensor diagnostic; no action and no task credit')
 while not st['stop'] and st['t']<spec['sim_seconds']-3 and time.monotonic()-wall<spec['wall_budget']-30:rclpy.spin_once(n,timeout_sec=.01)
 gate=st['gate'] or {};fraction=sum(samples)/len(samples) if samples else 0.;d=dict(passed=fraction>=.8 and gate.get('stopped',False),diagnostic_only=True,formal_credit=False,valid_fraction_after_25s=fraction,t=st['t'],final_gate=gate,scope='Parked localization probe; no route or work completion');(out/'mission_result.json').write_text(json.dumps(d,indent=2));n.destroy_node();rclpy.try_shutdown();return 0
if __name__=='__main__':raise SystemExit(main())
