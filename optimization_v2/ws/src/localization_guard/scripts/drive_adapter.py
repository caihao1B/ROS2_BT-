"""Bootstrap the unchanged S03 arbiter with a private TF topic; no safety logic copied."""
import argparse,os,signal,sys
from support import BASE,Path,read
sys.path.insert(0,str(BASE/'ws/src/chassis_bridge/scripts'))
from drive_bridge import Drive,rclpy
def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    if os.environ.get('ROS_DOMAIN_ID')!='119':raise ValueError('online domain required')
    rclpy.init(args=['--ros-args','-r','/tf:=/s05/internal/chassis_tf']);obj=Drive(read(a.config),a.out);stop=[False]
    def end(s,f):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'bridge_ready').write_text('unchanged S03 Drive; private TF output')
    try:
        while not stop[0]:rclpy.spin_once(obj.n,timeout_sec=.05)
    finally:obj.latched=True;obj.reason='SHUTDOWN';obj.tick();obj.n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
