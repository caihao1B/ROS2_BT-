"""Seal and export only after all four current R10 real cases actually pass.

Run after simulations have exited. GitHub publication is a separate atomic step.
"""
from pathlib import Path
import subprocess,json,fcntl,shutil,sys
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent;APP=ROOT/'optimization_v2/three_tasks'
lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
for name in ['spray_full_A1','spray_tool_jam','mowing_full_B1','mowing_startup_perception_loss']:
    d=json.loads((APP/'results/R10'/(name+'_latest.json')).read_text())
    if not d.get('passed'):raise SystemExit('Required actual case has not passed: '+name)
PY=ROOT/'optimization_v2/venvs/scene/bin/python'
def command(name,args):
    with (OUT/('finish_'+name+'.log')).open('w') as f:
        r=subprocess.run([str(x) for x in args],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
    print(name,r.returncode,flush=True)
    if r.returncode:raise SystemExit(r.returncode)
def stage(name,*args):command(name,['bash','optimization_v2/three_tasks/advance.sh',*args])
stage('R12_derivation_refresh','R12','build')
command('history',[PY,OUT/'history.py']);command('reproducibility',[PY,OUT/'reproducibility.py'])
stage('R10_initial','R10','finalize')
command('diagnostics',[PY,OUT/'diagnostics.py']);command('overview',[PY,OUT/'overview.py'])
stage('R10_evidence','R10','finalize');stage('R11_initial','R11','finalize')
media=APP/'results/R12/media';backup=OUT/'previous_assembled_media'
if media.exists() and not backup.exists():shutil.copytree(media,backup)
episodes=['spray','inspection','inspection_leader','mowing_startup_perception_loss','inspection_blocked']
full_spray=json.loads((APP/'results/R12/spray_full_A1_latest.json').read_text())
if full_spray.get('passed'):episodes.insert(0,'spray_full_A1')
full_mowing=json.loads((APP/'results/R12/mowing_full_B1_latest.json').read_text())
if full_mowing.get('passed'):episodes.insert(4,'mowing_full_B1')
stage('export','R12','export',*episodes)
command('video_qa',[PY,OUT/'qa_videos.py'])
stage('R12_initial','R12','finalize')
command('docs',[PY,OUT/'update_docs.py'])
for name in ['R10','R11','R12']:stage(name+'_final',name,'finalize')
for name in ['R08','R09','R10','R11','R12']:stage(name+'_check',name,'check')
command('preservation',[PY,OUT/'preservation.py'])
print('Local sealed delivery is ready for visual review and atomic GitHub sync.',flush=True)
