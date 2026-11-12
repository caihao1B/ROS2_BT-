#!/usr/bin/env python3
"""Per-origin LaserScan self filter; a missing stamped TF drops the scan."""
import collections,copy,json,math,pathlib,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
from sensor_math import inside,rotation

class Filter(Node):
    def __init__(self):
        super().__init__('dino_scan_filter')
        self.declare_parameter('generated_dir','');self.declare_parameter('enable_rear',False)
        d=pathlib.Path(self.get_parameter('generated_dir').value);self.c=json.loads((d/'contract.json').read_text());self.shapes=json.loads((d/'self_geometry.json').read_text())
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.status=self.create_publisher(String,'/dino/sensors/filter_status',50);self.pending={};self.previous={};self.counts=collections.Counter()
        self.pub={}
        for key in ('front','rear'):
            if key=='rear' and not self.get_parameter('enable_rear').value:continue
            self.declare_parameter('raw_'+key,'/dino/lidar/'+key+'/scan');self.declare_parameter('filtered_'+key,'/dino/lidar/'+key+'/scan_filtered')
            self.pub[key]=self.create_publisher(LaserScan,self.get_parameter('filtered_'+key).value,qos_profile_sensor_data)
            self.create_subscription(LaserScan,self.get_parameter('raw_'+key).value,lambda m,k=key:self.receive(k,m),qos_profile_sensor_data)
        self.create_timer(.02,self.flush,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def report(self,key,reason,**data):
        self.counts[key+':'+reason]+=1;m=String();m.data=json.dumps({'sensor':key,'reason':reason,'counts':dict(self.counts),**data});self.status.publish(m)
    def receive(self,key,m):
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if t<=0:self.report(key,'ZERO_STAMP_REJECTED');return
        if m.header.frame_id!='lidar_'+key+'_link':self.report(key,'FRAME_REJECTED');return
        if not math.isfinite(m.angle_increment) or m.angle_increment<=0 or len(m.ranges)<2:self.report(key,'SCAN_GEOMETRY_REJECTED');return
        if m.time_increment!=0:self.report(key,'ROLLING_SCAN_REQUIRES_DESKEW');return
        if t<=self.previous.get(key,-1):self.report(key,'NONMONOTONIC_STAMP');return
        if key in self.pending:self.report(key,'QUEUE_REPLACED')
        self.pending[key]=(m,t,time.monotonic())
    def flush(self):
        now=self.get_clock().now().nanoseconds/1e9
        for key,(m,t,wall) in list(self.pending.items()):
            if now-t>self.c['max_scan_age_sim_s'] or t-now>.05:
                self.report(key,'STALE_OR_FUTURE_SCAN',age_sim_s=now-t);del self.pending[key];continue
            transforms={}
            try:
                for link in {s['link'] for s in self.shapes}:
                    tr=self.buffer.lookup_transform(link,m.header.frame_id,Time.from_msg(m.header.stamp)).transform
                    transforms[link]=(rotation([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]),np.array([tr.translation.x,tr.translation.y,tr.translation.z]))
            except (TransformException,ValueError):
                if time.monotonic()-wall>self.c['transform_wait_wall_s']:
                    self.report(key,'TF_MISSING_DROP',stamp=t);del self.pending[key]
                continue
            ranges=np.asarray(m.ranges,dtype=float);angles=m.angle_min+np.arange(len(ranges))*m.angle_increment
            valid=np.isfinite(ranges)&(ranges>=m.range_min)&(ranges<=m.range_max)
            points=np.c_[ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid]),np.zeros(np.count_nonzero(valid))]
            mask=np.zeros(len(points),dtype=bool)
            for shape in self.shapes:
                rot,trans=transforms[shape['link']];mask|=inside(points@rot.T+trans,shape,self.c['self_filter_padding_m'])
            out=copy.deepcopy(m);filtered=ranges.copy();indices=np.where(valid)[0];filtered[indices[mask]]=float('nan')
            # NaN means unavailable/occluded, never free space behind the robot.
            filtered[np.isfinite(ranges)&~valid]=float('nan');out.ranges=filtered.tolist()
            period=t-self.previous[key] if key in self.previous else self.c['scan_period_s']
            if out.scan_time==0:out.scan_time=float(period)
            self.pub[key].publish(out);self.previous[key]=t;del self.pending[key]
            self.report(key,'OK',stamp=t,masked=int(mask.sum()),valid=int(valid.sum()),age_sim_s=now-t,raw_scan_time=m.scan_time,scan_time=out.scan_time,time_increment=out.time_increment)

def main():
    rclpy.init();n=Filter()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
