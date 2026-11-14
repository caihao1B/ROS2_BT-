"""Explicit common BT binding; no original mission tree or safety gate modified."""
from common import *
import yaml,copy

def build():
 require('R05');out=edition('R06');out.mkdir(parents=True,exist_ok=True);old=yaml.safe_load((BASE/'config/decision_policy.yaml').read_text());mission=read(edition('R03')/'mission.json')
 for task in TASKS:
  c=read(edition('R04')/'vehicles'/f'{task}.json');rig=read(edition('R05')/task/'sensors.json');cfg=dict(old,task=task,schema='R06-COMMON-BT/1',task_version=mission['version'],vehicle_hash=c['config_hash'],rig_revision=rig['revision'],session='diagnostic_'+task,scope='Common BT and cancellation protocol; no actuator publisher or tool completion authority',speed_cap_mps=.6,cancel_confirm_s=c['safety']['stop_delay_s'],progress_delta_m=.10,progress_timeout_s=10.,follow_min_gap_m=2.,follow_target_gap_m=3.,follow_min_ttc_s=3.)
  cfg['revision']=canonical(cfg);dump(out/(task+'.json'),cfg)
 xml=PKG/'r06_backend/common.xml';(out/'common.xml').write_text(xml.read_text());return out
