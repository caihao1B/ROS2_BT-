"""Executed component and teaching checks; never full-scene acceptance credit."""
from common17 import *
import argparse,subprocess,datetime

def verify(out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);commands=[]
    scene=str(BASE/'venvs/scene/bin/python');include=str(PKG/'include');build=BASE/'build_s17';build.mkdir(exist_ok=True)
    for p in files(PKG):
        if p.suffix=='.py':compile(p.read_text(),str(p),'exec')
    jobs=[]
    for name in ['contracts17','inspection_contract17','inspection_audit_contract17','inspection_budget_contract17','inspection_obstacles_contract17','dynamic_envelope_contract17','map_memory_contract17','temporal_join_contract17','visual_contract17','prediction_cost_contract17','darp_contract17']:jobs.append((name,[scene,str(PKG/'tests'/(name+'.py'))]))
    jobs.extend([
        ('pose_jump_build',['g++','-std=c++17','-O2','-UNDEBUG','-I/usr/include/eigen3','-I'+include,str(PKG/'tests/pose_jump_contract17.cpp'),'-o',str(build/'pose_jump_contract17')]),
        ('pose_jump',[str(build/'pose_jump_contract17')]),
        ('lesson_cpp_build',['g++','-std=c++20','-O2','-UNDEBUG','-I/usr/include/eigen3','-I'+include,str(BASE/'lessons/S17/lesson.cpp'),'-o',str(build/'lesson_cpp')]),
        ('lesson_cpp',[str(build/'lesson_cpp')]),('lesson_python',[scene,str(BASE/'lessons/S17/lesson.py')])])
    for name,args in jobs:
        begin=time.monotonic();p=subprocess.run(args,capture_output=True,text=True,timeout=90);(out/(name+'.log')).write_text(p.stdout+p.stderr);commands.append(dict(name=name,argv=args,exit_code=p.returncode,wall_s=time.monotonic()-begin))
        dump(out/'verification17.json',dict(passed=all(r['exit_code']==0 for r in commands),complete=len(commands)==len(jobs),commands=commands,source_hashes=source_binding(),checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='component contracts and runnable lessons; no Gazebo task completion credit'))
        if p.returncode:raise ValueError('COMPONENT_CHECK_FAILED '+name)
    print(json.dumps(dict(passed=True,commands=len(commands),out=str(out)),ensure_ascii=False),flush=True)
    return read(out/'verification17.json')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=RESULTS/'component_contracts/current');a=p.parse_args();verify(a.out)
