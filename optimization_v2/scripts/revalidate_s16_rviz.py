#!/usr/bin/env python3
"""Prove that only marker display TF stamps changed, then attach real GUI evidence."""
from pathlib import Path
import json,hashlib,datetime,sys
ROOT=Path(__file__).resolve().parents[2];BASE=ROOT/'optimization_v2';PROOF=BASE/'results/S16/rviz_regression_20261112'
NODE='optimization_v2/ws/src/experience_memory/scripts/node16.py'
INSTALLED='optimization_v2/ws/install/experience_memory/share/experience_memory/scripts/node16.py'
TAIL="exec(compile(source,str(P15/'scripts/prediction_node15.py'),'exec'),globals())"
PATCH='''# Display-only: MarkerArray does not defer future-stamped TF lookups like PointCloud.
# Packet t / model inputs / all safety deadlines stay unchanged. Zero = latest TF.
source=once(source,'from geometry_msgs.msg import Point','from geometry_msgs.msg import Point\\nfrom builtin_interfaces.msg import Time as MarkerTime')
marker_stamp='m.header.stamp=n.get_clock().now().to_msg()'
if source.count(marker_stamp)!=2:raise ValueError('Unexpected S15 marker layout')
source=source.replace(marker_stamp,'m.header.stamp=MarkerTime()')
'''
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def row(p):return dict(path=str(p.relative_to(ROOT)),sha256=sha(p))
def main():
 gui=Path(sys.argv[1]).resolve();assert gui.is_relative_to(BASE/'results/S16')
 old=read(PROOF/'prior_receipt.json');assert old['automated_status']=='PASS'
 assert sha(BASE/'results/S15/latest.json')==old['dependency_receipts']['S15']
 for r in old['source_files']:
  p=PROOF/'before'/r['path'] if r['path'] in [NODE,INSTALLED] else ROOT/r['path']
  assert sha(p)==r['sha256'],r['path']
 for r in old['artifacts']:assert sha(ROOT/r['path'])==r['sha256'],r['path']
 for key in [NODE,INSTALLED]:
  before=(PROOF/'before'/key).read_text();assert before.count(TAIL)==1
  assert (ROOT/key).read_text()==before.replace(TAIL,PATCH+TAIL),'nonvisual node edit'
 snapshot=read(gui/'source_snapshot.json');previous={x['path']:x['sha256'] for x in old['source_files']}
 assert all(sha(ROOT/k)==h for k,h in snapshot.items())
 assert {k for k,h in snapshot.items() if previous.get(k)!=h}=={NODE,INSTALLED}
 d=read(gui/'result.json');assert d['status']=='PASS' and d['mode']=='demo' and len(d['cases'])==1
 case=gui/next(iter(d['cases']));r=read(case/'runtime.json');m=next(iter(d['cases'].values()))
 assert r['gui'] and r['rviz_capture'] and m['safe'] and m['completed'] and m['injected_version_rejected']
 review=read(PROOF/'visual_review.json');assert review['reviewer']=='assistant' and review['user_manual_status']=='NOT_CHECKED' and review['visible_tracks_and_forecasts'] and review['prediction_display_ok']
 screenshot=(ROOT/review['screenshot_path']).resolve();assert screenshot.is_relative_to(case) and sha(screenshot)==review['screenshot_sha256']
 evidence=dict(schema='S16-MARKER-TF-DISPLAY-REVALIDATION/1',passed=True,original_receipt_sha256=sha(PROOF/'prior_receipt.json'),changed_sources=[NODE,INSTALLED],control_forecast_t_and_algorithms_unchanged=True,original_matrix_reexecuted=False,gui_run=d['run_id'],visual_review=review)
 (PROOF/'verification.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')
 doc=ROOT/'docs/optimization_v2/S16/UI_VALIDATION.md';doc.write_text('# S16 显示复验\n\n正式 13 案例数据保持原始内容；12 案例完成，遮挡缓存组 1/2 后按原重试上限安全停止。\n\n报告长文本溢出已单独复验。随后 GUI 发现 MarkerArray 请求发布时刻的 TF 可能超前于最新变换；只把两个显示标记头的 stamp 改为零（使用最近可用 TF）。预测消息的 t、输入、时效、风险和控制未改，S15 原源码也未改。它不把过期预测伪装为新数据。\n\n'+f'实际 GUI：`{d["run_id"]}`，任务完成、安全和版本拒绝通过。助手检查真实点云、目标跟踪与未来分支，预测显示状态正常。用户人工仍 NOT_CHECKED。\n\n'+f'字节差异、全部旧源码/证据哈希和 GUI 证明：`{PROOF.relative_to(ROOT)}/verification.json`。没有宣称重跑整套物理矩阵。\n')
 updated=dict(old);updated['source_files']=[row(ROOT/r['path']) for r in old['source_files']]+[row(Path(__file__))]
 artifacts={r['path']:r for r in old['artifacts']}
 for folder in [PROOF,gui]:
  for p in folder.rglob('*'):
   if p.is_file() and '__pycache__' not in p.parts:artifacts[row(p)['path']]=row(p)
 artifacts[row(doc)['path']]=row(doc);updated['artifacts']=list(artifacts.values());updated['checked_at_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat();updated['marker_display_regression']=str((PROOF/'verification.json').relative_to(ROOT));updated['reason']='Original 13-case matrix (12 complete, all safe), report layout and actual marker-TF GUI regressions; cache disabled, physical evidence unchanged.'
 dest=BASE/'results/S16/latest.json';tmp=dest.with_suffix('.tmp');tmp.write_text(json.dumps(updated,ensure_ascii=False,indent=2)+'\n');tmp.replace(dest);print(json.dumps(evidence,ensure_ascii=False))
if __name__=='__main__':main()
