"""Readable derived view of exact R10 case_summary evidence; no new PASS claims."""
from pathlib import Path
import json,html
ROOT=Path(__file__).resolve().parents[2];APP=ROOT/'optimization_v2/three_tasks';OUT=APP/'results/R10'
def main():
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 import numpy as np
 s=json.loads((APP/'results/R10/case_summary.json').read_text());cases={x['case']:x for x in s['cases']};fig,axes=plt.subplots(1,3,figsize=(17,5.6))
 for ax,name,title in zip(axes,['spray_full_A1','inspection_full_connector','mowing_full_B1'],['Point service / A1','Inspection / full connector','Mowing / B1']):
  d=cases.get(name)
  if not d:ax.text(.5,.5,'NOT RUN',ha='center');continue
  p=ROOT/d['path'];file=p/('errors.npz' if name.startswith('spray') else 'trajectory.npz')
  if file.exists():
   z=np.load(file);key='truth_xy' if 'truth_xy' in z else 'xy';xy=z[key];ax.plot(xy[:,0],xy[:,1],lw=1.2,label='Actual motion');ax.scatter(*xy[0],marker='s',s=35,label='Start');ax.scatter(*xy[-1],marker='x',s=45,label='End')
  if name.startswith('spray'):
   cfg=json.loads((p/'mission.json').read_text());m=json.loads((p/'mission_result.json').read_text());done={x['point'] for x in m['completed']};points=cfg['clusters'][0]['points']
   for passed,color,label in [(False,'#de9a28','Unconfirmed'),(True,'#158765','Feedback confirmed')]:
    q=np.array([x['target'][:2] for x in points if (x['id'] in done)==passed])
    if len(q):q=q+[28,14];ax.scatter(q[:,0],q[:,1],s=18,color=color,label=label)
  if name.startswith('mowing'):
   cfg=json.loads((p/'mission.json').read_text())
   for seg in cfg['clusters'][0]['segments']:
    q=np.array(seg['poses_world']);ax.plot(q[:,0],q[:,1],color='.7',lw=.5,zorder=0)
  ax.set_title(title+'\n'+('PASS' if d['passed'] else 'PARTIAL'));ax.set_xlabel('Site X [m]');ax.set_ylabel('Site Y [m]');ax.axis('equal');ax.grid(alpha=.25);ax.legend(fontsize=8)
 fig.tight_layout();fig.savefig(OUT/'r10_overview.png',dpi=150);plt.close(fig)
 rows=[]
 for c in s['cases']:
  amount=''
  if c.get('selected_points') is not None:amount+=f"点 {c['confirmed_points']}/{c['selected_points']}；原 {c['original_points']}"
  if c.get('selected_segments') is not None:amount+=f"段 {c['confirmed_segments']}/{c['selected_segments']}；当前版本总段数 {c['original_segments']}，原参考 {c.get('reference_original_segments') or c['original_segments']}"
  if c.get('confirmed_area_m2') is not None:amount+=f"；确认 {c['confirmed_area_m2']:.2f} m²，残余 {c['residual_area_m2']:.2f} m²"
  rows.append('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in [c['case'],'PASS' if c['passed'] else 'PARTIAL',round(c.get('simulation_seconds') or 0,2),amount,c.get('reason') or c.get('terminal')])+'</tr>')
 body='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>R10 三任务验收</title><style>body{font:16px/1.7 sans-serif;max-width:1400px;margin:35px auto;padding:0 24px;color:#19332b}h1{font-size:30px}table{border-collapse:collapse;width:100%;font-size:14px}td,th{border:1px solid #cbd9d2;padding:10px;text-align:left;overflow-wrap:anywhere}th{background:#e6efe9}img{width:100%}.note{background:#f3f6ee;padding:16px}</style>'''
 body+='<h1>R10 三任务分别验收：'+('PASS' if s['passed'] else 'PARTIAL')+'</h1><p class="note">派生查看页，最终准入以 advance.sh R10 check 的封存回执为准。原 S17/S18 仍为 PARTIAL，未执行六十次矩阵。受阻案例通过表示停止与残余处理正确，不表示完成受阻作业；路径通过和工具确认面积分别统计。</p><img src="r10_overview.png" alt="三个任务的实际运动与确认范围"><table><tr><th>案例</th><th>结果</th><th>仿真秒</th><th>完成量与残余</th><th>终态/原因</th></tr>'+''.join(rows)+'</table><h2>待处理</h2><ul>'+''.join('<li>'+html.escape(x)+'</li>' for x in (s['pending'] or ['本阶段声明范围内无待处理项']))+'</ul><p>算法、C++/Python 逻辑和命令见 docs/three_tasks/R08、R09、R10。R11 外观与 R12 录像另有独立回执，不能由本表推定全部通过。</p></html>'
 (OUT/'acceptance.html').write_text(body)
if __name__=='__main__':main()
