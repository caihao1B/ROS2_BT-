"""Deliver the generated worker's real operator message code in an isolated DDS domain."""
import argparse
import ast
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
from spray_handoff_candidate_r10 import handoff_result


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--worker',type=Path,required=True);ap.add_argument('--inputs',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='157':raise SystemExit('Requires isolated ROS_DOMAIN_ID=157')
    worker=a.worker.resolve();text=worker.read_text();tree=ast.parse(text);selected=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='persist_handoff'];assert len(selected)==1
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import String
    observed=[];rclpy.init();writer=rclpy.create_node('handoff_probe_sender');operator=rclpy.create_node('handoff_probe_late_operator');qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub=writer.create_publisher(String,'/r10/operator_attention',qos)
    def until(fn):
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            rclpy.spin_once(writer,timeout_sec=.01);rclpy.spin_once(operator,timeout_sec=.01)
            if fn():return True
        return False
    s=json.loads(a.inputs.read_text())['measurements'];request=deepcopy(s['request']);closure=handoff_result(s)
    try:
        with tempfile.TemporaryDirectory(prefix='operator_handoff_probe_') as tmp:
            out=Path(tmp);scope=dict(out=out,json=json,operator_pub=pub,String=String)
            exec(compile(ast.Module(body=selected,type_ignores=[]),str(worker),'exec'),scope)
            write=scope['persist_handoff'];write(request);initial_disk=json.loads((out/'operator_handoff.json').read_text());
            operator.create_subscription(String,'/r10/operator_attention',lambda m:observed.append(json.loads(m.data)),qos)
            late_received=until(lambda:request in observed)
            final=dict(request,shutdown=closure);write(final);closed_received=until(lambda:final in observed)
            writer.destroy_node();final_disk=json.loads((out/'operator_handoff.json').read_text())
            checks=dict(actual_generated_message_writer=True,late_operator_received_initial=late_received,operator_received_closed_record=closed_received,
                        first_atomic_file_matches=initial_disk==request,final_file_survives_publisher_shutdown=final_disk==final,no_partial_file_left=not (out/'operator_handoff.next').exists(),
                        operator_sees_hold_incomplete=final_disk['terminal']=='HUMAN_REQUIRED' and not final_disk['mission_passed'] and not final_disk['automatic_resume'],residual_original_key_and_amount=final_disk['original_key']==s['request']['original_key'] and final_disk['residual']==3.)
    finally:
        operator.destroy_node();rclpy.try_shutdown()
    r=dict(schema='OPERATOR-NOTICE-PRIVATE-TRANSPORT/1',scope='Real publisher, late dashboard subscription and atomic file path of the generated worker, using a recorded formal measurement snapshot. No Gazebo handoff claimed.',domain=157,topic='/r10/operator_attention',qos='reliable transient local depth 1',passed=all(checks.values()),checks=checks,generated_worker_sha256=digest(worker),probe_sha256=digest(Path(__file__)),inputs_sha256=digest(a.inputs));a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(checks));return 0 if r['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
