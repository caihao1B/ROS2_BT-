#!/usr/bin/env python3
"""Strict display-only S16 correction; never re-label the physical matrix."""
from pathlib import Path
import json,hashlib,datetime,sys
ROOT=Path(__file__).resolve().parents[2];BASE=ROOT/'optimization_v2'
PROOF=BASE/'results/S16/report_regression_20261112'
TEMPLATE='optimization_v2/ws/src/experience_memory/scripts/report16.html'
INSTALLED='optimization_v2/ws/install/experience_memory/share/experience_memory/scripts/report16.html'
BROWSER='optimization_v2/ws/src/experience_memory/tests/browser16.py'
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def row(p):return dict(path=str(p.relative_to(ROOT)),sha256=sha(p))
def fixed_html(s):
 s=s.replace('body{font:16px','body{overflow-wrap:anywhere;font:16px')
 start=s.index("$('summary').innerHTML=");end=s.index('\nfunction metric',start)
 replacement="""const sel=D.result.selection;const names={NO_NET_CPU_GAIN:'预测链 CPU 净收益未达 5% 门槛',INSUFFICIENT_TEST_HITS:'可评分命中样本不足',INSUFFICIENT_CONFLICT_EXPOSURE:'冲突暴露样本不足',S15_PREDICTIVE_POLICIES_NOT_QUALIFIED:'S15 预测策略尚未取得默认启用资格',TASK_TIME_REGRESSION:'任务时间退化',SAFETY_FAILURE:'安全检查未通过',WRONG_HIT_OR_INSUFFICIENT_EVIDENCE:'误命中或证据不足',ACCURACY_REGRESSION:'预测误差退化'};
$('summary').innerHTML='<b>实验自动状态：'+D.result.status+' · 缓存默认关闭</b>'+(sel?'<p>相对相同事件触发基线：CPU 节省 '+(100*sel.cpu_saving_fraction_vs_event).toFixed(2)+'%，墙钟节省 '+(100*sel.wall_saving_fraction_vs_event).toFixed(2)+'%。</p><p class="warn">'+sel.reasons.map(x=>names[x]||(x.startsWith('INCOMPLETE_PAIRED_TASK:')?'配对任务未全部成功：'+x.split(':')[1]:x)).join('；')+'。</p>':'<p>实验记录已加载，正式启用结论以 test 回执为准。</p>')+'<small>'+D.result.run_id+'</small>';"""
 s=s[:start]+replacement+s[end:]
 s=s.replace('GPU 推理：N/A（CPU 模型）。','GPU 推理：N/A（CPU 模型）。计时与调用次数为三次重复合计，精度只统计第一轮可评分窗口。')
 s=s.replace('所有图均来自实际记录。','所有图均来自实际记录；每个场景最多展示 24 个抽样窗口，完整查询见预测日志。')
 return s

def main():
 old=read(PROOF/'prior_receipt.json');out=ROOT/old['report_path'];run=out.parent
 assert old['automated_status']=='PASS' and all(v=='PASS' for v in old['checks'].values())
 assert sha(BASE/'results/S15/latest.json')==old['dependency_receipts']['S15']
 permitted={TEMPLATE,INSTALLED,BROWSER}
 for r in old['source_files']:
  before=PROOF/'before'/r['path'] if r['path'] in permitted else ROOT/r['path']
  assert sha(before)==r['sha256'],r['path']
 for path in [TEMPLATE,INSTALLED]:assert (ROOT/path).read_text()==fixed_html((PROOF/'before'/path).read_text())
 before=(PROOF/'before'/BROWSER).read_text()
 assert (ROOT/BROWSER).read_text()==before.replace('q.data=D.examples','q.no_horizontal_overflow=document.documentElement.scrollWidth<=window.innerWidth;q.data=D.examples')
 changed={str((run/n).relative_to(ROOT)) for n in ['index.html','_qa.html','report_preview.png','browser_check.json']}
 for r in old['artifacts']:
  p=PROOF/'before'/r['path'] if r['path'] in changed else ROOT/r['path']
  assert sha(p)==r['sha256'],r['path']
 data=read(run/'result.json');assert data['status']=='PASS' and len(data['cases'])==13
 assert all(c['safe'] for c in data['cases'].values())
 assert sum(c['completed'] for c in data['cases'].values())==12
 assert data['cases']['occlusion_cache_none']['reason']=='BLOCK_RETRY_EXHAUSTED'
 browser=read(run/'browser_check.json');assert browser['passed'] and browser['no_horizontal_overflow'] and browser['exit_code']==0
 proof=dict(schema='S16-DISPLAY-ONLY-REVALIDATION/1',original_receipt_sha256=sha(PROOF/'prior_receipt.json'),original_matrix_run=data['run_id'],original_matrix_reexecuted=False,nonvisual_sources_and_evidence_unchanged=True,allowed_changed_sources=sorted(permitted),allowed_changed_artifacts=sorted(changed),browser=browser,manual_status='NOT_CHECKED',passed=True)
 (PROOF/'verification.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2)+'\n')
 updated=dict(old);updated['source_files']=[row(ROOT/r['path']) for r in old['source_files']]+[row(Path(__file__))]
 artifacts={r['path']:row(ROOT/r['path']) for r in old['artifacts']}
 for p in PROOF.rglob('*'):
  if p.is_file():artifacts[row(p)['path']]=row(p)
 updated['artifacts']=list(artifacts.values());updated['reason']='Original 13-case matrix, 12 complete and all safe; display-only layout regression passed. Cache disabled; no physical experiment relabeled.'
 updated['checked_at_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat();updated['presentation_regression']=str((PROOF/'verification.json').relative_to(ROOT))
 dest=BASE/'results/S16/latest.json';tmp=dest.with_suffix('.tmp');tmp.write_text(json.dumps(updated,ensure_ascii=False,indent=2)+'\n');tmp.replace(dest)
 print(json.dumps(proof,ensure_ascii=False))
if __name__=='__main__':main()
