"""R01 lifecycle receipt, deliberately separate from S17/S18 mission verdicts."""
from common_r1 import *
import html
from registry_r1 import TASKS,load_task

def generate():
    out=APP/'results/R01';runs={};issues=[];artifacts=[]
    def bind(p):
        p=Path(p)
        if not p.is_file():issues.append('MISSING '+str(p));return
        artifacts.append(dict(path=str(p.relative_to(ROOT)),sha256=sha(p)))
    for task in TASKS:
        last=read(APP/'results'/task/'last_run.json');p=ROOT/last['run'];r=read(p/'result.json');runs[task]=dict(path=str(p.relative_to(ROOT)),result=r)
        if r['task_version']!=load_task(task)['version'] or r['lifecycle_status']!='PASS' or not r['safety'] or not r['localization_pass'] or r['server_exit']!=0 or r['formal_credit']:
            issues.append('TASK_PROBE_FAILED '+task)
        for name in ['result.json','task_config.json','spec.json','runtime_source_binding.json','runtime.json','metrics17.json','cleanup_r1.json','loaded_compat.json','shutdown_compat.json','startup_samples.json','stop_request.json','command.json','adapter_r1.json','launcher.log']:
            bind(p/name)
    attempts=[]
    for task in TASKS:
        for p in sorted((APP/'results'/task).glob('r01_*/result.json')):
            if str(p.parent.relative_to(ROOT))!=runs[task]['path']:
                attempts.append(dict(path=str(p.relative_to(ROOT)),result=read(p)));bind(p)
    before=read(APP/'baseline/before.json');after=read(APP/'baseline/after.json')
    if not before['passed'] or not after['passed'] or before['stage_states']!=after['stage_states']:issues.append('FROZEN_HISTORY_CHANGED')
    validation=read(out/'validation.json')
    if not validation['passed']:issues.append('CONTRACT_OR_CONTROL_CHECK_FAILED')
    probe_paths=sorted(out.glob('exit_compat_*_project_*/result.json'))
    probes=[dict(path=str(p.relative_to(ROOT)),result=read(p)) for p in probe_paths]
    if not probes or not all(x['result']['passed'] for x in probes) or {x['result']['mode'] for x in probes}!={'service','signal'}:issues.append('REAL_SHUTDOWN_PROBES_FAILED')
    for p in probe_paths:
        bind(p);bind(p.parent/'commands.json');bind(p.parent/'server.log')
    for p in [APP/'baseline/frozen_s01_s18.json',APP/'baseline/before.json',APP/'baseline/after.json',out/'validation.json',out/'contracts.log',out/'cpp_race.json',out/'controls.json',out/'build.json',out/'build.log',APP/'build/libr01_shutdown_compat.so']:
        bind(p)
    bind(out/'clean_shell.json')
    for pattern in ['exit_*/result.json','gui_exit_*/result.json','gui_exit_*/view_shutdown_r1.json','rviz_gdb_*/gdb.log','gdb_*/gdb.log']:
        for p in sorted(out.glob(pattern)):bind(p)
    frozen=read(APP/'baseline/frozen_s01_s18.json')
    for sid in frozen['stage_states']:bind(BASE/'results'/sid/'latest.json')
    # Source/config/docs are stage-owned. New run outputs never replace this receipt.
    sources=[]
    for root in [PKG,APP/'config',APP/'versions',ROOT/'docs/three_tasks',ROOT/'lessons/three_tasks']:
        if root.exists():
            sources.extend(p for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    sources.extend([APP/'run.sh',APP/'build_compat.sh'])
    for p in sources:bind(p)
    artifact_map={x['path']:x for x in artifacts}
    result=dict(schema='R01-LIFECYCLE-ACCEPTANCE/1',stage='R01',automated_status='PASS' if not issues else 'FAIL',
        formal_task_acceptance=False,scope='Three task startup, localization admission and ordered stop only',
        original_stage_states=after['stage_states'],frozen_history=dict(files=after['files'],bytes=after['bytes'],changed=len(after['issues'])),
        runs=runs,previous_attempts=attempts,shutdown_probes=probes,validation=validation,issues=issues,
        report_path=str((out/'report.html').relative_to(ROOT)),artifacts=list(artifact_map.values()),
        next_step='R02' if not issues else None,limitations=['R01 uses existing lightweight diff/ackermann models.',
        'R01 does not complete original task clusters, route acceptance, new vehicle geometries, dedicated BTs or recording.',
        'S17/S18 remain PARTIAL. Host-local runtime bindings require requalification on another machine.',
        'Shutdown compatibility is limited to the current hashed Gazebo 8.15.0 and Boost build.',
        'Installed RViz ImageDisplay destructor crashes; default GUI omits only that display, leaving camera perception active.'])
    rows=''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in [task,v['result']['lifecycle_status'],v['result']['mission_status'],round(v['result']['sim_seconds'],2),round(v['result']['wall_seconds'],2),v['result']['server_exit'],v['result']['tool_shutdown']['state']])+ '</tr>' for task,v in runs.items())
    links=''.join('<li><a href="../'+task+'/'+Path(v['path']).name+'/result.json">'+task+'：原始结果</a> · <a href="../'+task+'/'+Path(v['path']).name+'/metrics17.json">原任务评分</a></li>' for task,v in runs.items())
    page='''<!doctype html><html lang="zh-CN"><meta charset="UTF-8"><title>R01 三任务验收</title><style>body{font:17px/1.6 system-ui;max-width:1100px;margin:40px auto;padding:24px;background:#f4f7f8;color:#17313b}table{border-collapse:collapse;background:white;width:100%}th,td{padding:12px;border:1px solid #ccd7dd}code{background:#e7edef;padding:4px}a{color:#00698b}.badge{font-weight:bold;color:#166b47}</style><h1>R01 三任务启动与退出</h1>'''
    page+='<p class="badge">R01 '+result['automated_status']+' · 原 S17 PARTIAL / S18 PARTIAL</p>'
    page+='<p>本页是短时真实启动、定位准入与停车退出验收。未把被中止任务视为完成；三种新车辆模型及专用行为树尚未实施。</p>'
    page+='<table><tr><th>任务</th><th>启动/退出</th><th>原任务完成</th><th>仿真秒</th><th>现实秒</th><th>Gazebo 退出</th><th>工具</th></tr>'+rows+'</table>'
    page+='<p>原始冻结文件 '+str(after['files'])+' 个；复查变化 '+str(len(after['issues']))+' 个。</p><ul>'+links+'</ul><p><a href="latest.json">完整回执和绑定哈希</a> · <a href="validation.json">控制/保护验证</a></p>'
    view=ROOT/runs['inspection']['path']/'rviz.png'
    if view.exists():
        bind(view);bind(view.parent/'view_r1.json');bind(view.parent/'view_shutdown_r1.json')
        page+='<h2>巡迹真实 RViz 画面</h2><p>点云、估计底盘坐标及导航数据；不是整条道路完成的证据。ImageDisplay 面板暂不加载，摄像头感知保持运行。</p><img style="max-width:100%" src="../inspection/'+view.parent.name+'/rviz.png">'
    if attempts:page+='<p>保留 '+str(len(attempts))+' 次前序尝试。前序 GUI 退出出现 -15/-6，生命周期为 FAIL；修复后的复验另存，不覆盖失败。</p>'
    page+='<p>下一步：'+('可以执行 R02；本轮不自动实施。' if not issues else '暂不能进入 R02。')+'</p></html>'
    (out/'report.html').write_text(page);bind(out/'report.html');result['artifacts']=list({x['path']:x for x in artifacts}.values())
    dump(out/'latest.json',result)
    return {k:result[k] for k in ['stage','automated_status','issues','frozen_history','next_step']}

def verify_receipt():
    p=APP/'results/R01/latest.json'
    if not p.exists():return dict(passed=False,reason='NO_R01_RECEIPT')
    d=read(p);bad=[]
    for x in d['artifacts']:
        p=ROOT/x['path']
        if not p.is_file() or sha(p)!=x['sha256']:bad.append(x['path'])
    return dict(passed=d['automated_status']=='PASS' and not bad,stage='R01',status=d['automated_status'],changed=bad,
        checked=len(d['artifacts']),historical_scope='Original receipt hashes and completed full-history audit; run baseline_r1.py to rehash all old payloads.',next_step=d['next_step'])

if __name__=='__main__':print(json.dumps(generate(),ensure_ascii=False))
