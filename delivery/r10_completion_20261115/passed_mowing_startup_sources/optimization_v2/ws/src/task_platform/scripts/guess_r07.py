"""Private wheel/IMU TF is an ICP initialization, never a second public TF owner."""
import argparse,signal
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('r07_icp_motion_guess',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);p=n.create_publisher(TFMessage,'/s05/internal/icp_tf',100);st=[False]
    def callback(m):
        t=TransformStamped();t.header=m.header;t.header.frame_id='wheel_imu_odom';t.child_frame_id='base_link';t.transform.translation.x=m.pose.pose.position.x;t.transform.translation.y=m.pose.pose.position.y;t.transform.translation.z=m.pose.pose.position.z;t.transform.rotation=m.pose.pose.orientation;p.publish(TFMessage(transforms=[t]))
    n.create_subscription(Odometry,'/s05/wheel_imu/odometry',callback,50)
    def end(s,f):st[0]=True
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,end)
    (a.out/'ready').write_text('independent wheel/IMU initial guess; private TF only')
    try:
        while not st[0]:rclpy.spin_once(n,timeout_sec=.01)
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
