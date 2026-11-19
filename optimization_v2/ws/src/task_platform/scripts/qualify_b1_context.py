"""Bind every non-emergency BT dispatch to the current physical B1 gear obligation and a new sensor observation."""
import json
from pathlib import Path
import sys

import qualify_b1_steering as steering
from common import APP, PKG, ROOT, canonical
from mowing_yaw_candidate_r10 import replace_once

ORIGINAL_WORKER=steering.worker_source
ORIGINAL_DRIVER=steering.select_driver
ORIGINAL_MISSION=steering.effective_mission
STEERING_FILE=Path(steering.__file__)
PROFILE='B1-seed877-current-gear'


def disposition(action, evidence, segment, direction, started):
    if action in ('HUMAN','STOP','RECOVER_LOCALIZATION'):
        return action
    if not (evidence.get('obligation_id')==segment and evidence.get('obligation_direction')==direction and
            evidence.get('stamp',-1)>started and evidence.get('sensor_capture',-1)>started+.12):
        return None
    return action


def worker():
    raw=ORIGINAL_WORKER()
    raw=replace_once(raw, "   if action in ['HUMAN','STOP','RECOVER_LOCALIZATION']:return action",
        "   action=disposition(action,bt.get('evidence',{}),s['id'],st['direction'],st.get('obligation_started',0.))\n   if action in ['HUMAN','STOP','RECOVER_LOCALIZATION']:return action")
    raw=replace_once(raw,"retry_progress=RetryProgress(s,st['t'],cfg['speed_cap_mps'])","retry_progress=RetryProgress(s,st['t'],cfg['speed_cap_mps']);st['obligation_started']=st['t']")
    raw=replace_once(raw,"   if st['stop']:raise InterruptedError('SESSION_STOP')",
        "   if st['stop']:raise InterruptedError('SESSION_STOP')\n   if retry_progress is not None and st['phase']!='DONE' and st['t']>retry_progress.deadline:raise TimeoutError('SEGMENT_HARD_SIM_TIME_BOUND')")
    marker="    if recovered or (f.done() and f.result().status==4):"
    raw=replace_once(raw,marker,"""    if not recovered and (interrupted or (f.done() and f.result().status!=4)) and checkpoint_index==len(s['members']) and s['kind']=='TURN_TOOL_OFF':
     loc=st['loc'] or {}
     if loc.get('valid') and passage(loc['pose'],s['poses'][-1])[0] and (st['bt']or {}).get('action') not in ['STOP','HUMAN']:
      if active:stop_all()
      if stopped_original_arrival(s):
       recovered=True;log('CURRENT_ORIGINAL_GOAL_PHYSICALLY_HELD_AFTER_NAV_STOP',segment=s['id'])
"""+marker)
    return raw


def mission(*args,**kwargs):
    cfg,sections,geometry=ORIGINAL_MISSION(*args,**kwargs)
    cfg['connector_qualification']['parent_steering_version']=cfg.pop('version')
    cfg['connector_qualification']['decision_binding']='matching worker obligation and armed direction, new BT context and terrain capture; STOP/HUMAN always immediate'
    cfg['version']=canonical(cfg)
    return cfg,sections,geometry


def driver(binary):
    original=ORIGINAL_DRIVER(binary)
    control=PKG/'scripts/control_mowing_r10.py'
    raw=control.read_text()
    raw=replace_once(raw,'context=dict(stamp=t,sequence=contextseq,',
        "context=dict(stamp=t,sequence=contextseq,obligation_id=worker.get('execution_segment_id'),obligation_direction=worker.get('direction'),sensor_capture=(s.get('terrain_health')or{}).get('t',-1.),")
    ctrl=steering.BUILD/'current_gear_gate.py'
    generated="import sys\nsys.path.insert(0,"+repr(str(PKG/'scripts'))+")\n"+raw
    if not ctrl.exists() or ctrl.read_text()!=generated:
        ctrl.write_text(generated)
    text=original.read_text()
    text=replace_once(text,"str(PKG/'scripts/control_mowing_r10.py'),'--out'",repr(str(ctrl))+",'--out'")
    text=replace_once(text,"bound += [PKG/'scripts/bridge_r10.py'",
        "bound+=["+','.join('Path('+repr(str(p))+')' for p in (ctrl,control,STEERING_FILE,Path(__file__)))+"];bound += [PKG/'scripts/bridge_r10.py'")
    target=steering.BUILD/'current_gear_recorded_runtime.py'
    if not target.exists() or target.read_text()!=text:
        target.write_text(text)
    return target


if __name__=='__main__':
    steering.PROFILE=PROFILE
    steering.worker_source=worker
    steering.select_driver=driver
    steering.effective_mission=mission
    if sys.argv[1:2]==['worker']:
        del sys.argv[1]
        exec(compile(worker(),str(steering.baseline.WORKER),'exec'),
             dict(__name__='__main__',__file__=str(steering.baseline.WORKER),RecoveryStopEvidence=steering.RecoveryStopEvidence,
                  PROFILE=PROFILE,RetryProgress=steering.RetryProgress,disposition=disposition))
    else:
        steering.__file__=__file__
        steering.main()
