"""Real BehaviorTree.CPP process: cancellation must precede idle acknowledgement."""
import sys,os,subprocess,time,signal
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common14 import *
from middleware13 import local_dds
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock

def main(out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);cfg=yaml.safe_load((BASE/'config/decision_policy.yaml').read_text());dump(out/'policy.json',cfg)
    env=local_dds(os.environ);env['ROS_DOMAIN_ID']='117';os.environ.update(env);rclpy.init();node=rclpy.create_node('s14_protocol_test');pub=node.create_publisher(String,'/s14/context',5);clock=node.create_publisher(Clock,'/clock',5);seen=[];node.create_subscription(String,'/s14/directive',lambda m:seen.append(json.loads(m.data)),10)
    args=[str(BASE/'ws/install/mission_bt/lib/mission_bt/mission_bt'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'policy.json'),'-p','tree_file:='+str(BASE/'behavior_trees/mission.xml'),'-p','log_file:='+str(out/'decisions.jsonl')];log=(out/'node.log').open('w');proc=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT);t=1.;ctx=dict(initialized=True,idle=True,remaining_s=300.,unsafe=False,blocked=False)
    def tick(seconds):
        nonlocal t
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            t+=.01;m=Clock();m.clock.sec=int(t);m.clock.nanosec=int((t-int(t))*1e9);clock.publish(m);pub.publish(String(data=json.dumps(dict(ctx,t=t))));rclpy.spin_once(node,timeout_sec=.015)
    passed=False
    try:
        tick(3);assert any(d['action']=='EXECUTE' for d in seen),'tree did not execute'
        # Deliberately deliver context before its /clock tick. The prior causal
        # sample must remain active, and the future unsafe sample must wait.
        index=len(seen);end=time.monotonic()+.15
        while time.monotonic()<end:
            pub.publish(String(data=json.dumps(dict(ctx,t=t+.01,idle=False,unsafe=True))));rclpy.spin_once(node,timeout_sec=.015)
        assert seen[index:] and all(d['action']=='EXECUTE' for d in seen[index:]),'DDS cross-topic ordering caused a false stop'
        ctx.update(idle=False,unsafe=True);index=len(seen);tick(.8);batch=seen[index:];assert any(d['action']=='CANCEL' for d in batch),'no cancellation';assert not any(d['action']=='STOP' for d in batch),'dispatched before idle acknowledgement'
        ctx['idle']=True;tick(.8);assert seen[-1]['action']=='STOP';passed=True
    finally:
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=3)
        log.close();node.destroy_node();rclpy.try_shutdown();dump(out/'result.json',dict(passed=passed,commands=args,exit_code=proc.returncode,directives=seen))
    print('ASYNC_CANCEL_ACK_PASS');return 0
if __name__=='__main__':raise SystemExit(main(sys.argv[1]))
