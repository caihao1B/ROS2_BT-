"""Publishable bounded-session evidence, including honest PARTIAL outcomes.

Run only after owned simulations end. This never changes acceptance thresholds.
"""
from pathlib import Path
import subprocess,json,fcntl,shutil
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent;APP=ROOT/'optimization_v2/three_tasks'
lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
PY=ROOT/'optimization_v2/venvs/scene/bin/python';steps=[]
def command(name,args,partial=False):
 with (OUT/('bounded_'+name+'.log')).open('w') as f:r=subprocess.run([str(x) for x in args],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
 steps.append(dict(step=name,exit=r.returncode));print(name,r.returncode,flush=True)
 (OUT/'bounded_delivery_steps.json').write_text(json.dumps(steps,indent=2))
 if r.returncode not in ([0,2] if partial else [0]):raise SystemExit(r.returncode)
def stage(name,*args,partial=False):command(name,['bash','optimization_v2/three_tasks/advance.sh',*args],partial)
stage('derivations','R12','build')
command('history',[PY,OUT/'history.py']);command('reproducibility',[PY,OUT/'reproducibility.py'])
stage('R10_initial','R10','finalize',partial=True)
command('diagnostics',[PY,OUT/'diagnostics.py']);command('overview',[PY,OUT/'overview.py'])
stage('R10_evidence','R10','finalize',partial=True);stage('R11_initial','R11','finalize',partial=True)
media=APP/'results/R12/media';backup=OUT/'previous_assembled_media'
if media.exists() and not backup.exists():shutil.copytree(media,backup)
episodes=[]
for name in ['spray_full_A1','spray','inspection','inspection_leader','mowing_full_B1','mowing_startup_perception_loss','inspection_blocked']:
 p=APP/'results/R12'/(name+'_latest.json')
 if not p.exists():continue
 d=json.loads(p.read_text())
 if d.get('video_valid') and d.get('mission_timeline_complete') and d.get('active_video_quality'):episodes.append(name)
if episodes:
 stage('export','R12','export',*episodes)
 command('video_qa',[PY,OUT/'qa_videos.py'])
stage('R12_initial','R12','finalize',partial=True)
command('docs',[PY,OUT/'update_docs.py'])
for s in ['R10','R11','R12']:stage(s+'_final',s,'finalize',partial=True)
for s in ['R08','R09','R10','R11','R12']:stage(s+'_check',s,'check',partial=s in ['R10','R11','R12'])
command('preservation',[PY,OUT/'preservation.py'])
print('Bounded local delivery sealed; review media, then synchronize GitHub.',flush=True)
