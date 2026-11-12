#!/usr/bin/env python3
"""Wheel/gyro input contracts and sole odom -> base_link output publisher."""
import copy,json,math,pathlib,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistWithCovarianceStamped,TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformBroadcaster,TransformException
from sensor_math import rear_to_base,shift_pose_covariance,shift_twist_covariance

class EstimatorIO(Node):
    def __init__(self):
        super().__init__('dino_local_estimator_io');self.declare_parameter('contract_file','')
        self.c=json.loads(pathlib.Path(self.get_parameter('contract_file').value).read_text());self.a=-self.c['rear_x']
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self);self.tf=TransformBroadcaster(self)
        self.wheel=self.create_publisher(TwistWithCovarianceStamped,'/dino/estimator/wheel_twist',20)
        self.imu=self.create_publisher(Imu,'/dino/estimator/imu',20)
        self.odom=self.create_publisher(Odometry,'/dino/odometry/filtered',20)
        self.status=self.create_publisher(String,'/dino/estimator/status',20)
        self.last={};self.wall={};self.fault=False
        self.create_subscription(Odometry,'/dino/wheel/odometry',self.on_wheel,20)
        self.create_subscription(Imu,'/dino/imu',self.on_imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/dino/estimator/rear_odometry',self.output,20)
    def report(self,reason,**kwargs):
        m=String();m.data=json.dumps({'reason':reason,**kwargs});self.status.publish(m)
    def valid(self,key,m,expected):
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;now=self.get_clock().now().nanoseconds/1e9
        if t<=0:self.report('ZERO_STAMP_REJECTED',input=key);return False
        if t<self.last.get(key,-1)-1e-6:self.fault=True;self.report('CLOCK_OR_SENSOR_REWIND');return False
        if self.fault:return False
        if t==self.last.get(key):return False
        if m.header.frame_id!=expected or now-t>self.c['max_input_age_sim_s'] or t-now>.05:
            self.report('INPUT_FRAME_OR_TIME_REJECTED',input=key);return False
        self.last[key]=t;self.wall[key]=time.monotonic();return True
    def on_wheel(self,m):
        if m.child_frame_id!='base_footprint' or not math.isfinite(m.twist.twist.linear.x):self.report('WHEEL_REJECTED');return
        if not self.valid('wheel',m,'odom'):return
        # x velocity is equal at base and rear axle; rear lateral zero is a declared rolling constraint.
        out=TwistWithCovarianceStamped();out.header=copy.deepcopy(m.header);out.header.frame_id='rear_axle_link';out.twist.twist.linear.x=m.twist.twist.linear.x
        for i,value in enumerate([self.c['wheel_vx_variance'],self.c['rear_no_side_slip_variance'],1e6,1e6,1e6,1e6]):out.twist.covariance[7*i]=value
        self.wheel.publish(out)
    def on_imu(self,m):
        if m.header.stamp.sec==0 and m.header.stamp.nanosec==0:self.report('ZERO_STAMP_REJECTED',input='imu');return
        if not all(math.isfinite(x) for x in [m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]):self.report('IMU_NONFINITE');return
        try:self.buffer.lookup_transform('rear_axle_link','imu_link',Time.from_msg(m.header.stamp))
        except TransformException:self.report('IMU_TF_MISSING_DROP');return
        if not self.valid('imu',m,'imu_link'):return
        out=copy.deepcopy(m);out.orientation_covariance=[-1.,0.,0.,0.,0.,0.,0.,0.,0.];out.linear_acceleration_covariance=[-1.,0.,0.,0.,0.,0.,0.,0.,0.]
        out.angular_velocity_covariance=[self.c['imu_yaw_rate_variance'] if i in (0,4,8) else 0. for i in range(9)]
        self.imu.publish(out)
    def output(self,m):
        now=self.get_clock().now().nanoseconds/1e9
        if self.fault or any(now-self.last.get(k,-1e6)>self.c['max_input_age_sim_s'] or time.monotonic()-self.wall.get(k,-1e6)>self.c['max_input_gap_wall_s'] for k in ('imu','wheel')):
            self.report('INPUT_STALE_NO_FORMAL_ODOM');return
        if m.header.frame_id!='odom' or m.child_frame_id!='rear_axle_link':self.report('EKF_FRAME_REJECTED');return
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if t<=0 or now-t>self.c['max_input_age_sim_s'] or t-now>.05:self.report('EKF_TIME_REJECTED');return
        q=m.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        x,y,_=rear_to_base(m.pose.pose.position.x,m.pose.pose.position.y,yaw,self.a)
        if not all(math.isfinite(v) for v in (x,y,yaw,m.twist.twist.angular.z)):self.report('EKF_NONFINITE');return
        out=copy.deepcopy(m);out.child_frame_id='base_link';out.pose.pose.position.x=x;out.pose.pose.position.y=y
        out.pose.covariance=shift_pose_covariance(m.pose.covariance,yaw,self.a).reshape(-1).tolist()
        out.twist.twist.linear.y=m.twist.twist.linear.y+self.a*m.twist.twist.angular.z
        out.twist.covariance=shift_twist_covariance(m.twist.covariance,self.a).reshape(-1).tolist()
        self.odom.publish(out)
        tr=TransformStamped();tr.header=out.header;tr.child_frame_id='base_link';tr.transform.translation.x=x;tr.transform.translation.y=y;tr.transform.rotation=out.pose.pose.orientation;self.tf.sendTransform(tr)
        self.report('OK',stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)

def main():
    rclpy.init();n=EstimatorIO()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
