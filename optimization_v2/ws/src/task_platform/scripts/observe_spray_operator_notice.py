"""Read-only operator console for a single live spray handoff experiment."""
import argparse,json,os,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run',type=Path,required=True);ap.add_argument('--max-wall-s',type=float,default=360.);args=ap.parse_args();run=args.run.resolve();spec=json.loads((run/'spec.json').read_text());name=spec['session'];target=run/'operator_attention_wire.jsonl';result=run/'operator_attention_receipt.json'
    if os.environ.get('ROS_DOMAIN_ID')!='119' or run.name!=name or not 0<args.max_wall_s<=600 or target.exists() or result.exists() or (run/'runtime.json').exists():raise SystemExit('Expected one live domain-119 session and fresh observation paths')
    rclpy.init();node=rclpy.create_node('human_operator_notice_observer',enable_rosout=False,start_parameter_services=False);messages=[]
    with target.open('x') as f:
        def receive(message):
            d=json.loads(message.data)
            if d.get('session')!=name:return
            messages.append(d);f.write(json.dumps(d,ensure_ascii=False)+'\n');f.flush()
        node.create_subscription(String,'/r10/operator_attention',receive,QoSProfile(depth=4,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        end=time.monotonic()+args.max_wall_s
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.15)
            if (messages and messages[-1].get('shutdown')) or (run/'runtime.json').exists():break
    node.destroy_node();rclpy.try_shutdown();initial=any(m.get('terminal')=='HUMAN_REQUIRED' and 'shutdown' not in m for m in messages);closed=next((m for m in messages if 'shutdown' in m),{});disk=json.loads((run/'operator_handoff.json').read_text()) if (run/'operator_handoff.json').exists() else {};data=dict(schema='LIVE-OPERATOR-ROS-NOTICE/1',session=name,read_only=True,topic='/r10/operator_attention',message_count=len(messages),initial_request_received=initial,closure_received=bool(closed),closure_matches_durable_case_file=bool(closed) and disk==closed,passed=bool(initial and closed and disk==closed));result.write_text(json.dumps(data,indent=2)+'\n');print(json.dumps(data));return 0 if data['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
