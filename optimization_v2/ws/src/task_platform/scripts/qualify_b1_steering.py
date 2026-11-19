"""Drive the four seed877 sections using measured zero-speed steering alignment and directed RPP."""
import argparse
import math
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from xml.etree import ElementTree

import yaml
import qualify_b1_connector as baseline
from common import APP, PKG, ROOT, canonical, dump, edition, read, sha
from geometry_r02 import write
from mowing_yaw_candidate_r10 import replace_once
from recovery_stop_candidate_r10 import RecoveryStopEvidence

SOURCES=PKG/'r10_connector_nav'
BUILD=APP/'build/r10_connector_nav'
INSTALL=APP/'build/r10_connector_nav_install'
REAL_DRIVER=baseline.DRIVER
ORIGINAL_MISSION=baseline.mission_version
ORIGINAL_SCENE=baseline.candidate_scene
STABLE_DRIVER=Path(baseline.__file__)
ORIGINAL_WORKER=baseline.worker_source
PROFILE='B1-seed877-stopped-steering'


class RetryProgress:
    """Rearm a finite observation retry budget only after observed forward arc progress."""
    def __init__(self, segment, begin, max_speed):
        self.points=segment['poses']
        self.arc=[0.]
        for a,b in zip(self.points,self.points[1:]):
            self.arc.append(self.arc[-1]+math.dist(a[:2],b[:2]))
        self.max_speed=max_speed
        self.best=0.
        self.anchor=0.
        self.last=None
        self.total_interruptions=0
        self.deadline=begin+max(135.,min(240.,self.arc[-1]/.12+50.))

    def observe(self,loc,gate):
        if not loc or not loc.get('valid') or not gate or gate.get('mode')!='DRIVE' or not gate.get('allowed'):
            return
        t=loc.get('odom_t',loc['t'])
        if self.last is not None and t<=self.last:
            return
        dt=min(.5,t-self.last) if self.last is not None else .05
        self.last=t
        # Search only physically reachable forward path samples. Never jump to a distant
        # crossing or an alternative recovery path to extend the retry budget.
        location=loc['pose']
        upper=self.best+self.max_speed*dt+.035
        eligible=[(math.dist(p[:2],location[:2]),arc) for p,arc in zip(self.points,self.arc)
                  if self.best-.20<=arc<=upper and abs(math.remainder(p[2]-location[2],math.tau))<=.18]
        if eligible:
            cross,arc=min(eligible)
            if cross<=.15:self.best=max(self.best,arc)

    def interrupted(self,now):
        self.total_interruptions+=1
        if self.total_interruptions>18 or now>self.deadline:
            raise RuntimeError('SEGMENT_TURN_TOTAL_INTERRUPTION_OR_TIME_BOUND')
        if self.best-self.anchor>=.8:
            self.anchor=self.best
            return True
        return False


def worker_source():
    st=ORIGINAL_WORKER()
    st=replace_once(st, " def log(event,**d):", " retry_progress=None\n def log(event,**d):")
    st=replace_once(st,"  if st['phase']=='NAVIGATING':record_passage()",
        "  if st['phase']=='NAVIGATING':\n   record_passage()\n   if retry_progress is not None and not st.get('endpoint_recovery'):retry_progress.observe(st.get('loc'),st.get('gate'))")
    st=replace_once(st,"done=False;attempts=0","done=False;attempts=0;retry_progress=RetryProgress(s,st['t'],cfg['speed_cap_mps'])")
    st=replace_once(st,"    if retry_exhausted(attempts,len(s['members']),done):",
        "    if not done and retry_progress.interrupted(st['t']):\n     log('OBSERVED_PATH_PROGRESS_REARMS_CONSECUTIVE_RETRIES',segment=s['id'],arc_m=retry_progress.best,total_interruptions=retry_progress.total_interruptions);attempts=0\n    if retry_exhausted(attempts,len(s['members']),done):")
    return st


def build_controller():
    subprocess.run(['cmake','-S',str(SOURCES),'-B',str(BUILD),'-DCMAKE_BUILD_TYPE=Release',
                    '-DCMAKE_INSTALL_PREFIX='+str(INSTALL)],check=True)
    subprocess.run(['cmake','--build',str(BUILD),'--parallel','1'],check=True)
    subprocess.run(['cmake','--install',str(BUILD)],check=True)
    return INSTALL/'lib/libr10_path_curvature.so'


def effective_mission(*args,**kwargs):
    cfg,sections,geometry=ORIGINAL_MISSION(*args,**kwargs)
    if not kwargs.get('paired'):
        raise ValueError('Steering qualification requires capture-time terrain pairing')
    parent=cfg.pop('version')
    vehicle=cfg['vehicle']
    base_vehicle_hash=vehicle.pop('config_hash')
    vehicle['safety']['alignment_speed_mps']=0.
    vehicle['alignment_parent_hash']=base_vehicle_hash
    vehicle['config_hash']=canonical(vehicle)
    for cluster in cfg['clusters']:
        if cluster['vehicle_hash']==base_vehicle_hash:
            cluster['vehicle_hash']=vehicle['config_hash']
    cfg['connector_qualification'].update(parent_paired_version=parent,
         steering_profile=dict(alignment_speed_mps=0.,alignment_error_rad=vehicle['safety']['alignment_error_rad'],
                                physical_steering_rate_rad_s=vehicle['steering']['rate_rad_s'],steering_timeout_s=vehicle['safety']['steering_tracking_timeout_s'],
                                plugin='r10::DirectedPathCurvature',transient_retry=dict(rearm_after_observed_arc_m=.8,max_interruptions=18,hard_case_budget_s=390),gains=dict(lateral_per_m2=1.,heading_per_m=2.),
                                internal_position_target_m=.025,internal_heading_target_rad=.04))
    cfg['version']=canonical(cfg)
    return cfg,sections,geometry


def steering_scene(out,task,session,**kwargs):
    context=ORIGINAL_SCENE(out,task,session,**kwargs)
    cfg=read(out/'connector_mission.json')
    v=cfg['vehicle']
    dump(out/'vehicle.json',v)
    tree=ElementTree.parse(out/'world.sdf').getroot()
    plugin=tree.find("world/model[@name='vehicle']/plugin[@name='r02::WheelDrive']/config")
    physical=json.loads(plugin.text)
    if physical['config_hash']!=v['alignment_parent_hash']:
        raise ValueError('Unexpected original drivetrain')
    plugin.text=json.dumps(v)
    write(out/'world.sdf',tree)
    assembly=read(out/'assembly.json')
    assembly['base_vehicle_hash']=v['config_hash']
    assembly['alignment_parent_hash']=v['alignment_parent_hash']
    dump(out/'assembly.json',assembly)
    nav=yaml.safe_load((edition('R10')/'mowing/nav2.yaml').read_text())
    ctl=nav['controller_server']['ros__parameters']
    ctl['TurnRPP'].update(plugin='r10::DirectedPathCurvature',vehicle_max_curvature=v['steering']['max_curvature'],
                          lateral_gain_per_m2=1.,heading_gain_per_m=2.)
    ctl['goal'].update(xy_goal_tolerance=.025,yaw_goal_tolerance=.04)
    (out/'connector_nav.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
    return context


def select_driver(binary):
    raw=REAL_DRIVER.read_text()
    raw=replace_once(raw, "bt.update(session=session,task_version=cfg['version'],rig_revision=cfg['rig']['revision'])",
                     "bt.update(session=session,task_version=cfg['version'],rig_revision=cfg['rig']['revision'],vehicle_hash=cfg['vehicle']['config_hash'])")
    raw=replace_once(raw,"str((edition('R10')/'mowing')/'nav2.yaml')","str(out/'scene/connector_nav.yaml')")
    raw=replace_once(raw,"raw=env.copy();raw['ROS_DOMAIN_ID']='121'",f"env['AMENT_PREFIX_PATH']={str(INSTALL)!r}+':'+env['AMENT_PREFIX_PATH'];env['LD_LIBRARY_PATH']={str(INSTALL/'lib')!r}+':'+env['LD_LIBRARY_PATH'];raw=env.copy();raw['ROS_DOMAIN_ID']='121'")
    sources=[STABLE_DRIVER,REAL_DRIVER,binary,*(p for p in SOURCES.iterdir() if p.is_file()),INSTALL/'share/r10_connector_navigation/plugins.xml']
    raw=replace_once(raw,"bound += [PKG/'scripts/bridge_r10.py'", "bound+=["+','.join('Path('+repr(str(p))+')' for p in sources)+"];bound += [PKG/'scripts/bridge_r10.py'")
    generated=BUILD/'steering_recorded_runtime.py'
    if not generated.exists() or generated.read_text()!=raw:
        generated.write_text(raw)
    return generated


def main():
    baseline.PROFILE=PROFILE
    baseline.worker_source=worker_source
    if sys.argv[1:2]==['worker']:
        del sys.argv[1]
        exec(compile(baseline.worker_source(),str(baseline.WORKER),'exec'),dict(__name__='__main__',__file__=str(baseline.WORKER),
                RecoveryStopEvidence=RecoveryStopEvidence,PROFILE=PROFILE,RetryProgress=RetryProgress))
        return
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['prepare','run'])
    parser.add_argument('--rtf',type=float,default=.5)
    parser.add_argument('--seconds',type=float,default=390)
    args=parser.parse_args()
    baseline.DRIVER=select_driver(build_controller())
    baseline.__file__=__file__
    baseline.mission_version=effective_mission
    baseline.candidate_scene=steering_scene
    cfg,sections,_=effective_mission(paired=True)
    opts=dict(task='mowing',rtf=args.rtf,cluster='B1',case='nominal',segment_start=cfg['connector_qualification']['start_index'])
    launch=baseline.runtime(paired=True)
    if args.mode=='prepare':
        with TemporaryDirectory() as temp:
            scene=Path(temp)/'scene'
            steering_scene(scene,session='prepare',paired=True,**opts)
            file=read(scene/'vehicle.json')
            assert file['config_hash']==canonical({k:v for k,v in file.items() if k!='config_hash'})
            print(json.dumps(dict(fixture=read(scene/'fixture.json'), qualification=cfg['connector_qualification'],
                                  parent_vehicle=file['alignment_parent_hash'],vehicle=file['config_hash']),indent=2))
        return
    out=launch(**opts,seconds=args.seconds,segment_limit=len(sections))
    from score_b1_connector import evaluate
    report=evaluate(out)
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 2)


if __name__=='__main__':
    main()
