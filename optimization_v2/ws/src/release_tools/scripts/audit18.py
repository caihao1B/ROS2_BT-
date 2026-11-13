"""One-pass receipt, resource and learning audit with truthful partial delivery."""
from common18 import *
import subprocess,time,re,html,os,fcntl

def receipt_inventory():
    expected={};states={};issues=[];receipts={}
    for i in range(1,18):
        sid=f'S{i:02}';path=BASE/f'results/{sid}/latest.json';r=read(path);receipts[sid]=sha(path)
        states[sid]=r.get('automated_status','NOT_RUN')
        for item in r.get('source_files',[])+r.get('artifacts',[]):
            rel=item['path'];p=(ROOT/rel).resolve()
            if not p.is_relative_to(ROOT):issues.append(dict(path=rel,reason='EXTERNAL_RECEIPT_RESOURCE'));continue
            if rel in expected and expected[rel]!=item['sha256']:issues.append(dict(path=rel,reason='CONFLICTING_RECEIPT_HASH'))
            expected[rel]=item['sha256']
        for dep,digest in r.get('dependency_receipts',{}).items():
            if sha(BASE/f'results/{dep}/latest.json')!=digest:issues.append(dict(path=sid,reason='DEPENDENCY_RECEIPT_CHANGED',dependency=dep))
    sizes={};start=time.monotonic()
    for rel,digest in expected.items():
        p=ROOT/rel
        if not p.is_file():issues.append(dict(path=rel,reason='MISSING'));continue
        if sha(p)!=digest:issues.append(dict(path=rel,reason='HASH_CHANGED'))
        sizes[rel]=p.stat().st_size
    return dict(stage_status=states,receipt_hashes=receipts,unique_files=len(expected),
        bound_bytes=sum(sizes.values()),wall_s=time.monotonic()-start,issues=issues,
        passed=not issues,files=[dict(path=p,sha256=h,bytes=sizes.get(p)) for p,h in sorted(expected.items())])

def learning():
    rows=[]
    for i in range(1,19):
        sid=f'S{i:02}';docs=[ROOT/f'docs/optimization_v2/{sid}/{n}.md' for n in ['TECHNICAL','CODE_WALKTHROUGH','MANUAL_CHECK','RESULTS']]
        lessons=[BASE/f'lessons/{sid}/{n}' for n in ['README.md','lesson.py','lesson.cpp']]
        rows.append(dict(stage=sid,files=[str(p.relative_to(ROOT)) for p in docs+lessons],missing=[str(p.relative_to(ROOT)) for p in docs+lessons if not p.is_file()]))
    return rows

def learning_links(rows):
    from urllib.parse import unquote
    checked=0;issues=[]
    for row in rows:
        for rel in row['files']:
            src=ROOT/rel
            if src.suffix!='.md' or not src.is_file():continue
            prose=re.sub(r'```.*?```|`[^`]*`','',src.read_text(),flags=re.S)
            for target in re.findall(r'\[(?:[^\[\]]|\[[^\]]*\])*\]\(([^)]+)\)',prose):
                if '://' in target or target.startswith('#'):continue
                raw=unquote(target.strip('<>')).split('#')[0]
                match=re.match(r'^(.*?):(\d+)$',raw);line=None
                if match:raw=match.group(1);line=int(match.group(2))
                path=Path(raw);path=path if path.is_absolute() else src.parent/path
                checked+=1
                if not path.is_file():issues.append(dict(document=rel,target=target,reason='MISSING_LINK_TARGET'))
                elif line is not None and (line<1 or line>len(path.read_text(errors='replace').splitlines())):
                    issues.append(dict(document=rel,target=target,reason='LINE_OUT_OF_RANGE'))
    return dict(checked=checked,issues=issues,passed=not issues,
        scope='local Markdown link target and line bounds; remote URLs and semantic line meaning not fully verified')

def source_paths():
    findings=[];symlinks=[]
    for folder in [BASE/'ws/src',BASE/'config',BASE/'assets',BASE/'maps']:
        for p in folder.rglob('*'):
            if p.is_symlink():
                resolved=p.resolve();symlinks.append(dict(path=str(p.relative_to(ROOT)),target=str(resolved),in_project=resolved.is_relative_to(ROOT),exists=p.exists()))
            if not p.is_file() or p.suffix not in ['.py','.sh','.cpp','.hpp','.yaml','.sdf','.xml'] or 'vendor' in p.parts:continue
            if p.stat().st_size>4_000_000:continue
            for n,line in enumerate(p.read_text(errors='replace').splitlines(),1):
                if '/media/' in line or '/mnt/' in line:
                    findings.append(dict(path=str(p.relative_to(ROOT)),line=n,text=line[:200],classification='REVIEW_REFERENCE_NOT_AUTOMATIC_RUNTIME_DEPENDENCY'))
    return dict(external_mount_references=findings,symlinks=symlinks,
        scope='active source/config/assets/maps; historical outputs and third-party source excluded; runtime receipts audited separately')

def sizes():
    result={}
    for part in ['ws/src','ws/build','ws/install','assets','maps','plans','results','venvs','showcase']:
        root=BASE/part;total=0;count=0
        for parent,dirs,files in os.walk(root,followlinks=False):
            dirs[:]=[d for d in dirs if not Path(parent,d).is_symlink()]
            for name in files:
                p=Path(parent,name)
                if not p.is_symlink():total+=p.stat().st_size;count+=1
        result[part]=dict(logical_bytes=total,regular_files=count)
    return result

def audit(out):
    lock=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    out=Path(out).resolve()
    if not out.is_relative_to(RESULTS):raise ValueError('S18_OUTPUT_SCOPE')
    out.mkdir(parents=True,exist_ok=True)
    from release18 import doctor
    preflight=doctor();dump(out/'preflight.json',preflight)
    print('Checking unique bound inputs (one hash per file)',flush=True)
    integrity=receipt_inventory();dump(out/'upstream_integrity.json',integrity)
    docs=learning();dump(out/'learning_files.json',docs)
    links_checked=learning_links(docs);dump(out/'learning_links.json',links_checked)
    paths=source_paths();dump(out/'path_audit.json',paths)
    inventory=sizes();dump(out/'sizes.json',inventory)
    packages=subprocess.run(['dpkg-query','-W','-f=${Package}\t${Version}\n','ros-jazzy-nav2-controller','ros-jazzy-rtabmap-slam','ros-jazzy-rtabmap-odom','ros-jazzy-rtabmap-sync','ros-jazzy-rtabmap-util','ros-jazzy-robot-localization','ros-jazzy-ros-gz-sim','ros-jazzy-fields2cover','libmetis-dev'],capture_output=True,text=True,timeout=15)
    (out/'system_packages.tsv').write_text(packages.stdout);dump(out/'system_packages_result.json',dict(exit_code=packages.returncode,stderr=packages.stderr))
    tests=[]
    for name in ['contracts18.py','follow_contract18.py','visual_contract18.py']:
        args=[str(BASE/'venvs/scene/bin/python'),str(PKG/'tests'/name)]
        p=subprocess.run(args,capture_output=True,text=True,timeout=30);(out/(name+'.log')).write_text(p.stdout+p.stderr)
        tests.append(dict(command=args,exit_code=p.returncode))
    clean={k:os.environ[k] for k in ['HOME','USER','LANG'] if k in os.environ};clean['PATH']='/usr/bin:/bin'
    args=['/bin/bash','--noprofile','--norc',str(BASE/'run.sh'),'doctor']
    p=subprocess.run(args,env=clean,cwd=out,capture_output=True,text=True,timeout=120)
    (out/'clean_shell.log').write_text(p.stdout+p.stderr);tests.append(dict(command=args,exit_code=p.returncode,environment='minimal fresh shell; existing system packages and built workspace'))
    cpp=BASE/'lessons/S18/lesson.cpp';exe=out/'lesson_cpp'
    for args in [['/usr/bin/g++','-std=c++17','-O2',str(cpp),'-o',str(exe)],[str(exe)],[str(BASE/'venvs/scene/bin/python'),str(BASE/'lessons/S18/lesson.py')]]:
        p=subprocess.run(args,capture_output=True,text=True,timeout=30);tests.append(dict(command=args,exit_code=p.returncode,output=p.stdout+p.stderr))
    dump(out/'commands.json',tests)
    checks={'S18-C01':'PARTIAL','S18-C02':'NOT_RUN','S18-C03':'PASS' if not any(r['missing'] for r in docs) and links_checked['passed'] and integrity['passed'] and all(t['exit_code']==0 for t in tests) else 'PARTIAL','S18-C04':'PARTIAL'}
    reasons=['S17 formal matrix and full connector qualification incomplete',
        'Clean shell uses this host existing build; full clean-machine rebuild NOT_RUN',
        'Unified three task entries and explicit stop/restart qualification incomplete; see actual runtime results',
        'Gazebo server shutdown returned -11 in current trials; task stop and process shutdown are separate checks',
        'Presentation assets are opt-in; camera/lidar equivalence and autonomous qualification NOT_RUN',
        'Bounded storage component tested; legacy live recorders not yet redirected',
        'Leader-following core tested offline; ROS/BT closed loop NOT_RUN']
    # Existing S18-owned actual runs, if any, are reported separately from historic S17.
    runtime_results=[]
    for d in sorted((BASE/'results/S17').glob('release18_*/release_result.json')):
        runtime_results.append(dict(path=str(d.relative_to(ROOT)),metrics=read(d)))
    if runtime_results:checks['S18-C02']='PARTIAL'
    summary=dict(status='PARTIAL',checks=checks,reasons=reasons,upstream_status=integrity['stage_status'],
        upstream_integrity=integrity['passed'],verified_files=integrity['unique_files'],
        component_commands_passed=all(t['exit_code']==0 for t in tests),clean_shell_passed=tests[3]['exit_code']==0,
        missing_learning_files=sum(len(r['missing']) for r in docs),runtime_results=runtime_results,
        local_learning_links=links_checked,
        manual_status='NOT_CHECKED',full_project_accepted=False)
    dump(out/'summary.json',summary)
    source_files=[p for p in PKG.rglob('*') if p.is_file() and '__pycache__' not in p.parts]+[BASE/'run.sh',BASE/'stages/S18/run.sh',BASE/'config/storage_policy.yaml']+[p for p in (BASE/'assets/presentation_v1').glob('*') if p.is_file()]
    manifest=dict(schema='S18-RELEASE/1',status='PARTIAL',root=str(ROOT),relocatable_to_new_machine=False,
        sources={str(p.relative_to(ROOT)):sha(p) for p in sorted(source_files)},
        map_revision=preflight['map_revision'],plan_revision=preflight['plan_revision'],
        upstream_receipts=integrity['receipt_hashes'],upstream_payload_inventory=str((out/'upstream_integrity.json').relative_to(ROOT)),
        upstream_payload_inventory_sha256=sha(out/'upstream_integrity.json'),system_packages=str((out/'system_packages.tsv').relative_to(ROOT)),
        disabled=['learned_predictor','experience_cache','leader_following_live','presentation_skin_in_formal_tasks'],remaining=reasons)
    dump(BASE/'config/release_manifest.json',manifest)
    links=''.join(f'<li><a href="{html.escape(str((ROOT/f).relative_to(ROOT)))}">{html.escape(f)}</a></li>' for row in docs for f in row['files'])
    # Base URL resolves all project-relative evidence from this nested report.
    prefix=os.path.relpath(ROOT,out)+'/'
    page=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><base href="{prefix}"><title>S18 部分交付</title><style>body{{font:17px system-ui;background:#eef3f6;color:#173047;max-width:1100px;margin:40px auto;padding:24px}}pre{{white-space:pre-wrap;background:white;padding:20px}}a{{color:#12679a}}li{{margin:7px}}</style><h1>S18 部分交付 · PARTIAL</h1><p>用户允许提前整理交付；不等于全地图或正式矩阵通过。人工验证尚未执行。</p><pre>{html.escape(json.dumps(summary,indent=2,ensure_ascii=False))}</pre><h2>三个任务入口</h2><pre>bash optimization_v2/run.sh spray --cluster A1\nbash optimization_v2/run.sh mow --cluster B1\nbash optimization_v2/run.sh inspect --vehicle diff\nbash optimization_v2/run.sh stop</pre><p><a href="docs/optimization_v2/快速开始.md">快速开始</a> · <a href="docs/optimization_v2/学习导航.md">学习导航</a> · <a href="{str((out/'sizes.json').relative_to(ROOT))}">体积清单</a> · <a href="{str((out/'path_audit.json').relative_to(ROOT))}">路径依赖审计</a></p><h2>十八步材料</h2><ul>{links}</ul></html>'''
    (out/'index.html').write_text(page)
    docs18=[ROOT/f'docs/optimization_v2/S18/{n}.md' for n in ['TECHNICAL','CODE_WALKTHROUGH','MANUAL_CHECK','RESULTS']]
    lessons=list((BASE/'lessons/S18').glob('*'));artifacts=[p for p in out.iterdir() if p.is_file()]+docs18+lessons+[ROOT/'docs/optimization_v2/快速开始.md',ROOT/'docs/optimization_v2/学习导航.md',BASE/'README.md']
    artifacts += [p for p in (BASE/'showcase/presentation_v1').rglob('*') if p.is_file() and p.name in ['preview.png','preview.json','result.json','skin.visual.json','skin.sdf','preview.sdf','world.sdf']]
    artifacts += [d for d in (BASE/'results/S17').glob('release18_*/*') if d.is_file() and d.name in ['release_result.json','spec.json','release_command.json','cleanup14.json','runtime.json','metrics17.json','manual_stop18.json','resume_source.json']]
    receipt=dict(schema_version=1,step='S18',plan_revision='20261109.1',run_id=out.name,
        checked_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),automated_status='PARTIAL',manual_status='NOT_CHECKED',
        reason='已交付独立启动/审计/学习与存储组件；S17、全新构建、完整入口恢复和记录器限额仍有缺口',check_exit_code=2,checks=checks,
        report_path=str((out/'index.html').relative_to(ROOT)),source_files=[dict(path=str(p.relative_to(ROOT)),sha256=sha(p)) for p in source_files+[BASE/'config/release_manifest.json']],
        artifacts=[dict(path=str(p.relative_to(ROOT)),sha256=sha(p)) for p in artifacts if p.is_file()],dependency_receipts={'S17':sha(BASE/'results/S17/latest.json')})
    dump(RESULTS/'latest.json',receipt)
    print(json.dumps(summary,indent=2,ensure_ascii=False));print(out/'index.html')
    return summary
