#!/usr/bin/env python3
"""Audit the runtime actually used, with per-file hashes and recursively resolved assets."""
import hashlib, json, os, platform, subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[5]
BASE = ROOT/'optimization_v2'
NATIVE = ROOT/'native_20261108'
PACKAGES = ['ros-jazzy-ros-gz-sim','ros-jazzy-gz-sim-vendor','ros-jazzy-robot-localization',
            'ros-jazzy-behaviortree-cpp','ros-jazzy-nav2-controller','ros-jazzy-nav2-mppi-controller',
            'ros-jazzy-nav2-regulated-pure-pursuit-controller','ros-jazzy-fields2cover',
            'ros-jazzy-ortools-vendor','libmetis-dev','nlohmann-json3-dev']

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024), b''): h.update(block)
    return h.hexdigest()

def file_row(path):
    p=Path(path)
    if not p.resolve().is_relative_to(ROOT): raise ValueError(f'EXTERNAL_RUNTIME_PATH:{p}')
    return {'path':str(p.relative_to(ROOT)), 'sha256':sha(p)}

def tree_files(path):
    return sorted(p for p in path.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc'))

def capture(cmd):
    p=subprocess.run(cmd, text=True, capture_output=True, timeout=30)
    return {'command':cmd,'exit_code':p.returncode,'stdout':p.stdout.strip(),'stderr':p.stderr.strip()}

def audit(out, freeze=False):
    out=Path(out); out.mkdir(parents=True,exist_ok=True)
    config=BASE/'config'; config.mkdir(exist_ok=True)
    left=ROOT/'ros2_ws/src'; right=NATIVE/'ros2_ws/src'
    a={str(p.relative_to(left)):sha(p) for p in tree_files(left)}
    b={str(p.relative_to(right)):sha(p) for p in tree_files(right)}
    comparison={'root_file_count':len(a),'native_file_count':len(b),
                'changed':[p for p in sorted(a.keys()&b.keys()) if a[p]!=b[p]],
                'only_root':sorted(a.keys()-b.keys()),'only_native':sorted(b.keys()-a.keys()),
                'authoritative_legacy_source':str(right),'new_source':str(BASE/'ws/src'),
                'root_source_policy':'reference_only_not_loaded_or_edited_for_new_features'}
    resource_roots=[NATIVE/'ros2_ws/install/dino_description/share',NATIVE/'farm_scene/gazebo/models']
    queue=[NATIVE/'farm_sensor_smoke.sdf',right/'dino_sensors/generated/validation_dual.sdf']
    visited=set(); refs=[]; missing=[]
    while queue:
        p=queue.pop().resolve()
        if p in visited:continue
        visited.add(p)
        root=ET.parse(p).getroot()
        for node in root.iter():
            tag=node.tag.split('}')[-1]; uri=(node.text or '').strip()
            if tag in ('uri','albedo_map','normal_map','roughness_map','metalness_map','emissive_map') and uri:
                if uri.startswith('model://'):
                    options=[d/uri[8:] for d in resource_roots]
                    target=next((q for q in options if q.exists()),None)
                elif uri.startswith('file://'):target=Path(uri[7:])
                elif '://' in uri:target=None
                else:target=p.parent/uri
            elif tag=='init_from' and uri.lower().endswith(('.png','.jpg','.jpeg','.tga')):
                target=p.parent/uri
            else:continue
            good=bool(target and target.exists() and target.resolve().is_relative_to(ROOT))
            refs.append({'source':str(p.relative_to(ROOT)),'uri':uri,'resolved':str(target) if target else None,'valid':good})
            if not good: missing.append(refs[-1]);continue
            if target.is_dir():
                mf=target/'model.sdf'
                if not mf.exists():missing.append({'uri':uri,'error':'model.sdf missing'})
                else:queue.append(mf)
            elif target.suffix.lower() in ('.sdf','.dae','.urdf'):queue.append(target)
    # Hash all active installed project resources and the selected native source, not old outputs.
    runtime=set(tree_files(right))
    for pkg in ['dino_description','dino_control','dino_sensors','dino_slam','dino_navigation']:
        runtime.update(tree_files(NATIVE/'ros2_ws/install'/pkg/'share'/pkg))
        runtime.update(tree_files(NATIVE/'ros2_ws/install'/pkg/'lib'))
    runtime.update(tree_files(NATIVE/'farm_scene/gazebo/models'))
    runtime.update([NATIVE/'farm_sensor_smoke.sdf',NATIVE/'run.sh',NATIVE/'scripts/env.sh',
                    NATIVE/'task_bt/run.sh',NATIVE/'task_bt/README.md'])
    files=[file_row(p) for p in sorted(runtime)]
    installed={}
    for pkg in PACKAGES:
        r=capture(['dpkg-query','-W','-f=${Version}',pkg])
        installed[pkg]=r['stdout'] if r['exit_code']==0 else None
    lock={'schema_version':1,'platform':'Ubuntu 24.04 amd64 / ROS Jazzy / Gazebo Harmonic',
          'installed':installed,
          'compatibility':{'fields2cover':'v2.1.1 direct C++ headland/swaths/order + METIS compiled probe; full Nav2 adapter not validated in S01',
          'upstream_reference':'https://github.com/open-navigation/opennav_coverage/blob/jazzy-v2/README.md',
          'upstream_verified_version':'v2.0.0','legacy_jazzy_branch':'v1 API; do not substitute for jazzy-v2'},
          'deferred':{'ros-jazzy-rtabmap-ros':{'candidate':'0.23.13','required_by':'S05/S06','state':'not installed; no S02 dependency'},
                      'learned_predictor':'S15 isolated environment; not required for S01'},
          'local_sources':'project_baseline/PROVENANCE.json; source/config hashes in each receipt'}
    lock['rendering']={'server':'ogre2','s01_gui':'ogre',
                       'reason':'Observed Ogre2 GUI material rendering errors; GUI-only Ogre fallback verified on this host.',
                       'camera_followup':'S04 must independently validate image sensor rendering and material fidelity.'}
    (out/'dependency_lock.yaml').write_text(yaml.safe_dump(lock,allow_unicode=True,sort_keys=False))
    if freeze:(config/'dependency_lock.yaml').write_text(yaml.safe_dump(lock,allow_unicode=True,sort_keys=False))
    hardware={k:capture(v) for k,v in {'kernel':['uname','-r'],
              'cpu':['lscpu','-J'],'gpu':['nvidia-smi','--query-gpu=name,driver_version,memory.total','--format=csv,noheader'],
              'memory':['free','-b'],'compiler':['g++','--version']}.items()}
    native_binding=capture(['bash','--noprofile','--norc','-c',
                           f'source {NATIVE}/scripts/env.sh\nros2 pkg prefix dino_sensors'])
    manifest={'schema_version':1,'source_comparison':comparison,'runtime_files':files,
              'asset_references':refs,'missing_resources':missing,'hardware':hardware,
              'native_package_binding':native_binding,'source_roots':{'legacy':str(right),'new':str(BASE/'ws/src')},
              'legacy_bt':{'status':'ISOLATED_NOT_A_FULL_TASK_ENTRY','replacement':'project_baseline/baseline_bt + core tests; S14 adds complete task layer'},
              'model_limitations':['Original steering torque 10 Nm is not adequate in historical grounded steering tests.',
                 'Derived simulation model uses 300 Nm; not a real actuator calibration.',
                 'S01 verifies stationary sensors and components, not full autonomous mowing.']}
    text=json.dumps(manifest,ensure_ascii=False,indent=2)+'\n'
    if freeze:(config/'baseline_manifest.json').write_text(text)
    (out/'baseline_manifest.json').write_text(text)
    checks={'resources_resolved':not missing,'runtime_contained':True,
            'native_source_is_loaded':native_binding['stdout']==str(NATIVE/'ros2_ws/install/dino_sensors'),
            'required_packages_installed':all(installed.values()),
            'external_smoke_dependency_removed':'robot_migration_' not in (NATIVE/'run.sh').read_text(),
            'legacy_bt_explicitly_isolated':'LEGACY_BT_ISOLATED' in (NATIVE/'task_bt/run.sh').read_text()}
    result={'status':'PASS' if all(checks.values()) else 'FAIL','checks':checks,
            'changed_source_files':len(comparison['changed']),'runtime_file_count':len(files),'asset_reference_count':len(refs)}
    (out/'audit_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    result=audit(p.parse_args().out); print(json.dumps(result,ensure_ascii=False,indent=2))
    raise SystemExit(result['status']!='PASS')
