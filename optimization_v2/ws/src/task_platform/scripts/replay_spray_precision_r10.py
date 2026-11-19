"""Offline negative tests against the measured A1 shutdown and the sealed dispatch methods."""
import argparse
import ast
import copy
import json
import random
import textwrap
from pathlib import Path
from types import SimpleNamespace

from common import ROOT,read,dump,sha
from spray_handoff_candidate_r10 import handoff_result
from spray_precision_r10 import StoppedWindow


def stream(path):
    with path.open() as lines:
        for line in lines:
            yield json.loads(line)


def require(condition,why):
    if not condition:
        raise AssertionError(why)


def shutdown_replay(case):
    cfg=read(case/'mission.json')
    receipt=read(case/'mission_result.json')
    tool=read(case/'scene/tool.json')
    now=receipt['t']
    ref=next(e for e in reversed(list(stream(case/'reference/reference.jsonl'))) if e.get('valid') and e['t']<=now)
    feedback=next(e for e in reversed(list(stream(case/'tool_observer/observations.jsonl'))) if e['kind']=='feedback' and e['t']<=now)
    limits=dict(max_v=cfg['vehicle']['safety']['stopped_v_mps'],max_w=cfg['vehicle']['safety']['stopped_w_rad_s'],
                max_feedback_age_s=tool['feedback_age_sim_s'],max_tool_rpm_rad_s=tool['stopped_max_rad_s'],
                max_ack_age_s=tool['stop_timeout_s'],tool_revision=tool['tool_revision'])
    snapshot=dict(t=now,request=read(case/'operator_handoff.json'),gate=receipt['final_gate'],
                  odometry=dict(t=ref['odom_t'],v=ref['v'],w=ref['w']),feedback=feedback,
                  events=list(stream(case/'mission_events.jsonl')),limits=limits,mission_passed=False,actions_idle=True)
    require(handoff_result(snapshot)['shutdown_verified'],'Recorded double stop must validate')
    mutations=dict(valve_open=('feedback','request_on',True),nonzero_flow=('feedback','flow',.01),
                   moving_rotor=('feedback','rpm_rad_s',limits['max_tool_rpm_rad_s']),
                   stale_feedback=('feedback','t',now-limits['max_feedback_age_s']-.001),
                   different_epoch=('feedback','sensor_epoch','OTHER'),base_moving=('odometry','v',limits['max_v']),
                   no_chassis_confirmation=('gate','stopped',False))
    out={}
    for name,(where,key,value) in mutations.items():
        modified=copy.deepcopy(snapshot)
        modified[where][key]=value
        assessment=handoff_result(modified)
        require(not assessment['shutdown_verified'] and assessment['process_exit']==2,name)
        out[name]=[k for k,v in assessment['checks'].items() if not v]
    modified=copy.deepcopy(snapshot)
    modified['events']=[e for e in modified['events'] if e['event']!='FINAL_TOOL_STOP']
    assessment=handoff_result(modified)
    require(not assessment['shutdown_verified'],'Missing physical STOP_WORK acknowledgement')
    out['missing_stop_ack']=[k for k,v in assessment['checks'].items() if not v]
    return dict(session=case.name,positive_measured_shutdown=True,faults_rejected=out)


def seeded_windows():
    for seed in range(64):
        rng=random.Random(seed)
        w=StoppedWindow([0.,0.],1.5)
        now=0.;epoch=0;stamp=0.;had_fault=False;fault_end=None;passed=None
        while now<7.:
            now+=rng.uniform(.015,.047)
            if now>4. or rng.random()>.16:
                stamp=now-rng.uniform(.001,.006)
            before=1.5<=now<=1.72
            valid=True;stopped=True;offset=0.
            if before:
                had_fault=True
                if seed%4==0:valid=False
                if seed%4==1:stopped=False
                if seed%4==2:epoch=1;offset=.32
                if seed%4==3:stamp=now-.19
            elif had_fault and fault_end is None:
                fault_end=now
            if seed%4==2 and had_fault:
                epoch=1
            stable,_=w.update(now,dict(valid=valid,pose=[offset+rng.uniform(-.002,.002),0.,0.],odom_t=stamp,t=now,localization_epoch=epoch),
                              dict(t=now,stopped=stopped,tool_stopped=True))
            if stable:
                require(fault_end and now>3.29,'Old epoch or movement data leaked into stop')
                require(w.history[-1][0]-w.history[0][0]>=1.8,'Short stationary observation')
                require(all(a[0]<b[0] for a,b in zip(w.history,w.history[1:])),'Replay counted a duplicate')
                passed=now
                break
        require(passed,'No eventual independent stationary convergence under bounded jitter')
    return dict(seeds=64,cases=['invalid localization','moving chassis','32 cm pose jump into a new localization epoch','stale acquisition'],
                independent_sampling='15–47 ms callback intervals, repeated source stamps, ±2 mm pose noise; guaranteed fresh tail after 4 s')


def service_policy(worker):
    tree=ast.parse(worker.read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    method=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='service')
    code=ast.get_source_segment(worker.read_text(),method)
    code=textwrap.dedent('    '+code)
    wrapper='def create(st,tick,wait,log,tool,result,tool_goal):\n    active_tool=None\n'+textwrap.indent(code,'    ')+'\n    return service\n'
    scoped={}
    exec(compile(wrapper,'sealed-spray-service','exec'),scoped)
    return scoped['create']


def tool_cancellation(worker):
    service_factory=service_policy(worker)
    results={}
    for action in ('STOP','HUMAN','WAIT','CANCEL','DENIED_BEFORE_START'):
        reason='stop test';events=[]
        state=dict(t=10.,gate={'stopped':True,'tool_stopped':True},
                   bt={'action':'STOP' if action=='DENIED_BEFORE_START' else 'EXECUTE','stopped_confirmation':True},
                   feedback={'meter':0.},phase='STOPPING')
        class Device:
            sent=0;cancelled=0;accepted=True
            def send_goal_async(self,goal):
                self.sent+=1;return self
            def get_result_async(self):
                return self
            def done(self):
                return self.cancelled==1 and state['t']>10.1
            def cancel_goal_async(self):
                self.cancelled+=1
            def result(self):
                return SimpleNamespace(result=SimpleNamespace(state='CANCELLED',reason='USER_CANCEL',
                    confirmed_quantity=.25,remaining_quantity=2.75,uncertainty_pending=False,replayed=False))
        device=Device()
        def tick():
            state['t']+=.04
            state['bt'].update(action=action,reason_code=reason)
            require(state['t']<11.,'Unbounded tool cancellation')
        def wait(predicate,seconds):
            if not predicate():
                raise TimeoutError('TEST_WINDOW_EXPIRED')
        def log(event,**fields):
            events.append(dict(event=event,**fields))
        service=service_factory(state,tick,wait,log,device,lambda handle:handle,lambda point:point)
        if action=='DENIED_BEFORE_START':
            try:service({'id':'original'})
            except TimeoutError:pass
            else:raise AssertionError('BT already stopped, unexpectedly admitted tool')
            require(device.sent==0 and device.cancelled==0,'Tool sent during stopped BT')
            results[action]=dict(sent=0)
            continue
        receipt=service({'id':'original'})
        require(device.sent==1 and device.cancelled==1 and receipt['state']=='CANCELLED' and
                receipt['remaining']==2.75 and receipt['confirmed']==.25,action)
        results[action]=dict(sent=1,cancel_requests=1,remaining=receipt['remaining'],state=receipt['state'])
    return results


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--handoff',type=Path,required=True)
    ap.add_argument('--candidate',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    spec=read(a.candidate/'spec.json')
    worker=next(ROOT/p for p,v in spec['implementation_bindings'].items() if v==spec['online_probe_sha256'] and p.endswith('.py'))
    require(sha(worker)==spec['online_probe_sha256'],'Sealed live worker source mismatch')
    result=dict(schema='A1-OFFLINE-ROBUSTNESS/1',scope='recorded shutdown plus deterministic simulated callback tests; no additional driven points',
                worker_sha256=sha(worker),handoff_hash=sha(a.handoff/'operator_handoff.json'),closed_valve_and_double_stop=shutdown_replay(a.handoff),
                randomized_stop_observation=seeded_windows(),frozen_live_dispatch_BT_cancellation=tool_cancellation(worker),formal_credit=False,passed=True)
    dump(a.out,result)
    print(a.out)


if __name__=='__main__':
    main()
