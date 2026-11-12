"""Real 40-participant roundtrip regression for the former 32-peer limit."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from middleware13 import local_dds
def worker(index,out):
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('s13_discovery_'+str(index));pub=node.create_publisher(String,'/s13/discovery_probe',100);seen=set()
    node.create_subscription(String,'/s13/discovery_probe',lambda m:seen.add(int(m.data)),200)
    node.create_timer(.1,lambda:pub.publish(String(data=str(index))));start=time.monotonic()
    try:
        while time.monotonic()-start<35:
            rclpy.spin_once(node,timeout_sec=.03)
            if not out.exists() and seen==set(range(40)):
                owners=sorted(p.node_name for p in node.get_publishers_info_by_topic('/s13/discovery_probe'))
                if owners==sorted('s13_discovery_'+str(i) for i in range(40)):
                    out.write_text(json.dumps(dict(passed=True,participants=40,received_ids=sorted(seen),owners=owners,wall_s=time.monotonic()-start)))
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('out',type=Path);ap.add_argument('--worker',type=int);a=ap.parse_args()
    if a.worker is not None:worker(a.worker,a.out);return 0
    a.out.mkdir(parents=True,exist_ok=True);result=a.out/'result.json';result.unlink(missing_ok=True)
    for f in a.out.glob('ack_*.json'):f.unlink()
    env=local_dds(os.environ);env['ROS_DOMAIN_ID']='118';children=[];logs=[]
    try:
        for i in range(40):
            log=(a.out/f'participant_{i}.log').open('w');logs.append(log);children.append(subprocess.Popen([sys.executable,__file__,str(a.out/f'ack_{i}.json'),'--worker',str(i)],env=env,stdout=log,stderr=subprocess.STDOUT))
        deadline=time.monotonic()+45
        while len(list(a.out.glob('ack_*.json')))<40 and time.monotonic()<deadline:
            if children[0].poll() is not None:break
            time.sleep(.1)
        acks=[json.loads(f.read_text()) for f in a.out.glob('ack_*.json')];r=dict(passed=len(acks)==40 and all(d['passed'] for d in acks),participants=40,fully_connected_participants=len(acks),directed_message_pairs=sum(len(d['received_ids']) for d in acks),max_discovery_wall_s=max([d['wall_s'] for d in acks] or [0]));result.write_text(json.dumps(r))
        print(json.dumps(r));return 0 if r['passed'] else 1
    finally:
        for p in children:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in children:
            try:p.wait(timeout=4)
            except subprocess.TimeoutExpired:p.kill();p.wait(timeout=2)
        for f in logs:f.close()
if __name__=='__main__':raise SystemExit(main())
