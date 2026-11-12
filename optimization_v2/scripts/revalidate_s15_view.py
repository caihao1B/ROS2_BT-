#!/usr/bin/env python3
"""Reissue S15 evidence after one display-only fixed-frame correction.

Never accepts algorithm, map, dataset, policy, threshold or task changes. The
original headless matrix and its hashes remain intact, alongside a real GUI run.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'optimization_v2'
CONFIG='optimization_v2/config/rviz/prediction.rviz'

def read(path):
    return json.loads(path.read_text())

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def row(path):
    path=path.resolve()
    if not path.is_relative_to(ROOT):raise ValueError('Evidence outside project')
    return dict(path=str(path.relative_to(ROOT)),sha256=sha(path))

def require(value,message):
    if not value:raise ValueError(message)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--proof',type=Path,required=True)
    p.add_argument('--gui-run',type=Path,required=True)
    a=p.parse_args();proof=a.proof.resolve();gui=a.gui_run.resolve()
    require(proof.is_relative_to(BASE/'results/S15') and gui.is_relative_to(BASE/'results/S15'),'Project-owned S15 evidence required')
    old=read(proof/'prior_receipt.json')
    require(old['automated_status']=='PASS' and all(old['checks'].get(f'S15-C{i:02}')=='PASS' for i in range(1,5)),'Original formal matrix must have passed')
    require(sha(BASE/'results/S14/latest.json')==old['dependency_receipts']['S14'],'S14 dependency changed')
    previous=(proof/'rviz_before.rviz').read_bytes();current=(ROOT/CONFIG).read_bytes()
    require(previous.count(b'Fixed Frame: odom')==1 and current==previous.replace(b'Fixed Frame: odom',b'Fixed Frame: map'),'Only the RViz display frame may change')
    for r in old['source_files']:
        expected=sha(proof/'rviz_before.rviz') if r['path']==CONFIG else sha(ROOT/r['path'])
        require(expected==r['sha256'],'Original source differs: '+r['path'])
    for r in old['artifacts']:
        require(sha(ROOT/r['path'])==r['sha256'],'Original evidence differs: '+r['path'])
    original=ROOT/old['report_path'];matrix=read(original.parent/'result.json')
    require(matrix['status']=='PASS' and matrix['mode']=='test' and len(matrix['cases'])==15,'Complete formal experiment matrix required')
    old_sources=read(original.parent/'source_snapshot.json');new_sources=read(gui/'source_snapshot.json')
    require(set(old_sources)==set(new_sources),'GUI source inventory differs')
    differences=[k for k in old_sources if old_sources[k]!=new_sources[k]]
    require(differences==[CONFIG],'Nonvisual runtime changed')
    require(all(sha(ROOT/k)==v for k,v in new_sources.items()),'GUI source snapshot is stale')
    result=read(gui/'result.json')
    require(result['mode']=='demo' and result['status']=='PASS' and len(result['cases'])==1,'Real GUI demonstration required')
    name=next(iter(result['cases']));case=gui/name;runtime=read(case/'runtime.json')
    require(runtime.get('gui') and runtime.get('rviz_capture') and runtime['success'],'Actual RViz capture and task success required')
    require(result['cases'][name]['safe'] and result['cases'][name]['completed'],'GUI task must be safe and complete')
    view=yaml.safe_load((case/'scene/localization.rviz').read_text())['Visualization Manager']
    require(view['Global Options']['Fixed Frame']=='map','Generated viewer uses wrong frame')
    review=read(proof/'visual_review.json')
    require(review['reviewer']=='assistant' and review['user_manual_status']=='NOT_CHECKED' and review['visible_cloud_tracks_modes_probabilities'],'Do not substitute GUI process exit for visual inspection')
    screenshot=(ROOT/review['screenshot_path']).resolve()
    require(screenshot.is_relative_to(case),'Screenshot is not from the owned GUI case')
    require(review['screenshot_sha256']==sha(screenshot),'Visual review belongs to a different image')
    verification=dict(schema='S15-DISPLAY-ONLY-REVALIDATION/1',
        original_matrix_run=matrix['run_id'],gui_run=result['run_id'],changed_source_keys=differences,
        original_matrix_reexecuted=False,nonvisual_source_unchanged=True,
        original_receipt_sha256=sha(proof/'prior_receipt.json'),
        before_sha256=sha(proof/'rviz_before.rviz'),after_sha256=sha(ROOT/CONFIG),
        actual_gui_safe=True,actual_gui_completed=True,visual_review=review,
        explanation='RViz focal point already used map coordinates. Change display frame only; prediction, risk and control remain odom.',passed=True)
    (proof/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
    doc=ROOT/'docs/optimization_v2/S15/UI_VALIDATION.md'
    doc.write_text('# S15 显示配置复验\n\n正式 15 项无界面矩阵保持原始文件和源码哈希。随后发现 RViz 观察中心为 map 坐标，而显示参考系为 odom，导致初始视图偏离车辆。\n\n仅将 `config/rviz/prediction.rviz` 的 `Fixed Frame: odom` 改为 `map`；预测、风险和控制仍使用 odom，不改变算法、场景、数据、阈值、任务和地图。\n\n'+
        f'实际 GUI 复验：`{result["run_id"]}`。任务与安全检查通过；助手检查真实截图中的点云、跟踪框、未来分支和概率。用户人工验收仍为 NOT_CHECKED。\n\n'+
        '新回执组合原始无界面矩阵与这次显示复验，不宣称完整矩阵重新执行。`scripts/revalidate_s15_view.py` 验证唯一字节差异、全部原始证据、原有源码、S14 依赖及实际 GUI 结果后签发。\n\n'+f'版本证明：`{proof.relative_to(ROOT)}/verification.json`。\n')
    updated=dict(old)
    updated['source_files']=[row(ROOT/r['path']) for r in old['source_files']]+[row(Path(__file__))]
    artifacts={r['path']:r for r in old['artifacts']}
    for folder in [proof,gui]:
        for path in folder.rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts:artifacts[row(path)['path']]=row(path)
    artifacts[row(doc)['path']]=row(doc)
    updated['artifacts']=list(artifacts.values())
    updated['checked_at_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
    updated['reason']='Original 15-case headless matrix plus actual display-only RViz regression; all nonvisual source/evidence unchanged.'
    updated['presentation_regression']=str((proof/'verification.json').relative_to(ROOT))
    destination=BASE/'results/S15/latest.json';temporary=destination.with_suffix('.tmp')
    temporary.write_text(json.dumps(updated,indent=2,ensure_ascii=False)+'\n');temporary.replace(destination)
    print(json.dumps(verification,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
