"""Replay actual dock-exhaustion evidence and real bounded-stall contexts through the native BT on an isolated domain."""
import argparse,gzip,json,os,signal,subprocess,tempfile,time
from pathlib import Path
from spray_defer_policy_r10 import exhausted_dock
from common import ROOT,APP,PKG,edition,read,dump,sha

def rows(p):
    with p.open() as f:
        for line in f:yield json.loads(line)

def extract(full, sample):
    if full.parent.name!='R12' or sample.parent.name!='spray_defer' or full==sample:raise ValueError('Expected one full original A1 and one experimental interaction')
    fm=read(full/'mission_result.json')
    if fm['error']!="RuntimeError('DOCKING_REALIGNMENT_EXHAUSTED')":raise ValueError('Expected measured docking exhaustion')
    events=list(rows(full/'mission_events.jsonl'));point=next(e['point'] for e in reversed(events) if e['event']=='NAV_GOAL')
    fstop=next(e['t'] for e in reversed(events) if e['event']=='TASK_STOP');last_fused=None
    for e in rows(full/'reference/reference.jsonl'):
        if e['t']>fstop:break
        last_fused=e
    actual_events=list(rows(sample/'mission_events.jsonl'));chosen=next(x for x in actual_events if x['event']=='CANDIDATE_DEFER_CONFIRMED' and x['point']==point and all(x['checks'].values()))
    start=next(e['t'] for e in actual_events if e['event']=='NAV_GOAL' and e.get('point')==point)
    ds=[{'t':d['t'],'evidence':d['evidence']} for d in rows(sample/'decisions.jsonl') if start-.5<=d['t']<=chosen['t']+.1 and d.get('context_sequence',0)>0]
    sources=[full/'mission_events.jsonl', full/'mission_result.json',full/'mission.json',full/'reference/reference.jsonl',sample/'mission_events.jsonl',sample/'bt.json',sample/'decisions.jsonl']
    return dict(schema='SPRAY-DEFER-RECORDED-INPUTS/1',full=full.name,sample=sample.name,point=point,gate=fm['final_gate'],stop=fstop,
       limit=read(full/'mission.json')['docking_realign_max_attempts'],events=[e for e in events if e.get('point')==point and e['event'] in ('LOCAL_DOCK_GOAL','LOCAL_DOCK_NAV_FAILED','DOCKING_SETTLED_CHECK')],
       last_fused={k:last_fused[k] for k in ('t','valid','reason')},selected=chosen,contexts=ds,bt=read(sample/'bt.json'),bindings={str(f.relative_to(ROOT)):sha(f) for f in sources})


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--full',type=Path);ap.add_argument('--interaction',type=Path);ap.add_argument('--bundle',type=Path)
    ap.add_argument('--capture-bundle',type=Path);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='151':raise SystemExit('Requires isolated ROS_DOMAIN_ID=151')
    if a.bundle:
        if a.full or a.interaction or a.capture_bundle:ap.error('--bundle reads captured observations directly')
        data=json.loads(gzip.decompress(a.bundle.read_bytes()));bundle_hash=sha(a.bundle)
    else:
        if not(a.full and a.interaction):ap.error('Use --bundle or supply --full and --interaction')
        data=extract(a.full.resolve(),a.interaction.resolve());bundle_hash=None
        if a.capture_bundle:
            a.capture_bundle.parent.mkdir(parents=True,exist_ok=True);a.capture_bundle.write_bytes(gzip.compress(json.dumps(data,ensure_ascii=False,separators=(',',':')).encode(),mtime=0));bundle_hash=sha(a.capture_bundle)
    if data.get('schema')!='SPRAY-DEFER-RECORDED-INPUTS/1':raise ValueError('Unknown replay format')
    point=data['point'];fstop=data['stop'];last_fused=data['last_fused'];chosen=data['selected'];ds=data['contexts']
    eligible=exhausted_dock(data['events'],point,data['limit'])
    if not eligible['eligible']:raise ValueError('Recorded precision corrections are not exhausted')
    binary=APP/'build/r10_spray_bt/r10_spray_supervisor';tree=edition('R06')/'common.xml'
    from rclpy import init,create_node,spin_once,shutdown
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    init();node=create_node('private_recorded_spray_defer_probe');clock=node.create_publisher(Clock,'/clock',5);pub=node.create_publisher(String,'/r06/context',5);messages=[];node.create_subscription(String,'/r06/directive',lambda m:messages.append(json.loads(m.data)),10)
    def wait(pred,limit):
        end=time.monotonic()+limit
        while time.monotonic()<end:
            spin_once(node,timeout_sec=.01)
            if pred():return True
        return False
    def send(payload,t,seq):
        c=Clock();ns=round(t*1e9);c.clock.sec=ns//10**9;c.clock.nanosec=ns%10**9;clock.publish(c);time.sleep(.008)
        pub.publish(String(data=json.dumps(payload)));off=len(messages)
        if not wait(lambda:any(m.get('context_sequence')==seq for m in messages[off:]),1.):raise RuntimeError(f'No private response to {seq}, received {[(m.get("context_sequence"),m.get("t"),m.get("reason_code")) for m in messages[-2:]]}')
        return next(m for m in reversed(messages) if m.get('context_sequence')==seq)
    with tempfile.TemporaryDirectory(prefix='native_defer_replay_') as tmp:
        out=Path(tmp);dump(out/'bt.json',data['bt']);cmd=[str(binary),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'bt.json'),'-p','tree_file:='+str(tree),'-p','log_file:='+str(out/'native.jsonl')]
        proc=subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True); replies=[];last_seq=0
        try:
            if not wait(lambda:pub.get_subscription_count() and clock.get_subscription_count() and node.count_publishers('/r06/directive'),8):raise RuntimeError('Private native BT failed discovery')
            initial=Clock();ns=round(ds[0]['t']*1e9);initial.clock.sec=ns//10**9;initial.clock.nanosec=ns%10**9
            for _ in range(15):clock.publish(initial);spin_once(node,timeout_sec=.01);time.sleep(.03)
            for d in ds:
                payload=dict(d['evidence']);seq=payload['sequence']
                if seq<=last_seq:continue
                last_seq=seq;message=send(payload,d['t'],seq);replies.append(message)
            # The control replays use measured context unchanged; these two final counterfactuals are explicit negative tests.
            e=dict(ds[-1]['evidence']);e['sequence']=last_seq+1;e['stamp']=ds[-1]['t']+.05;e['localized']=False;loc_stop=send(e,e['stamp'],e['sequence']);e['sequence']+=1;e['stamp']+=.05;e['localized']=True;e['task_version']='invalid-test-version';version_stop=send(e,e['stamp'],e['sequence'])
        finally:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=3)
            node.destroy_node();shutdown()
    native=[m for m in replies if m['action']=='DEFER' and m['reason_code']=='TRY_OTHER_OBLIGATION_RETAIN_RESIDUAL' and m['stopped_confirmation']]
    checks=dict(actual_two_exhausted_precision_motions=eligible['eligible'],fresh_formal_terminal_pose=0<=fstop-last_fused['t']<.15 and last_fused['valid'],native_wait=any(m['action']=='WAIT' for m in replies),native_replan=any(m['action']=='REPLAN' for m in replies),native_defer=bool(native),negative_localization_hold=loc_stop['action'] in ('WAIT','CANCEL','STOP') and loc_stop['reason_code']=='LOCALIZATION_ADMISSION_WAIT',negative_wrong_version_stop=version_stop['action']=='STOP' and version_stop['reason_code']=='VERSION_MISMATCH',clean_private_process=proc.returncode==0)
    p=Path(__file__).resolve();files=[binary,tree,p,PKG/'scripts/spray_defer_policy_r10.py'];receipt=dict(schema='SPRAY-PRECISION-AND-NATIVE-DEFER-REPLAY/1',passed=all(checks.values()),checks=checks,formal_credit=False,gazebo_precision_defer_demonstrated=False,
      scope_zh='以正式 A1 现场日志复算真实精停耗尽，并在独立 DDS 域将随后实体障碍短例随传感器得到的 BT 上下文逐条送入原生 C++ 监督器。两组时序单独记录，未声称正式运行在线产生 DEFER，也未重放车辆动力学。',
      formal_session=data['full'],formal_point=point,formal_stop_sim_s=fstop,formal_last_fused=dict(t=last_fused['t'],valid=last_fused['valid'],reason=last_fused['reason']),formal_gate=data['gate'],exhaustion=eligible,real_interaction_session=data['sample'],real_interaction_stage=chosen['stage'],recorded_contexts=len(replies),private_ros_domain=151,
      native_first_defer={k:native[0][k] for k in ('t','action','reason_code','dispatch_id','context_sequence','stopped_confirmation','blocked_duration_s')} if native else None,negative_controls=dict(loss=loc_stop['action'],version=version_stop['action']),bundle_sha256=bundle_hash,bindings={**data['bindings'],**{str(f.relative_to(ROOT)):sha(f) for f in files}})
    a.out.parent.mkdir(parents=True,exist_ok=True);dump(a.out,receipt);print(json.dumps(checks));return 0 if receipt['passed'] else 2

if __name__=='__main__':raise SystemExit(main())
