"""Render the R10 case table and physical trajectories from the assessed runs."""
import html

from common import APP, ROOT, read


def result_amount(case):
    if case.get('selected_points') is not None:
        return f"确认 {case['confirmed_points']}/{case['selected_points']} 个选择点，原簇 {case['original_points']} 点"
    if case.get('selected_segments') is not None:
        description = f"到达 {case['confirmed_segments']}/{case['selected_segments']} 个选择段"
        if case.get('reference_original_segments') is not None:
            description += f"，原参考 {case['reference_original_segments']} 段"
        if case.get('confirmed_area_m2') is not None:
            description += f"，确认 {case['confirmed_area_m2']:.2f} m²，残余 {case['residual_area_m2']:.2f} m²"
        return description
    return case.get('terminal') or '未记录'


def plot_case(ax, case, task):
    import numpy as np
    run = ROOT/case['path']
    archive = run / ('errors.npz' if task=='spray' else 'trajectory.npz')
    if archive.is_file():
        with np.load(archive) as measurements:
            xy = measurements['truth_xy' if task=='spray' else 'xy']
            if len(xy):
                ax.plot(xy[:,0],xy[:,1],lw=1.25,label='Observed motion (evaluation)')
                ax.scatter(*xy[0],marker='s',s=32,label='Start')
                ax.scatter(*xy[-1],marker='x',s=42,label='End')
    cfg = read(run/'mission.json')
    spec = read(run/'spec.json')
    if task=='spray':
        mission = read(run/'mission_result.json')
        confirmed = {job['point'] for job in mission['completed']}
        cluster = next(c for c in cfg['clusters'] if c['id']==spec['cluster'])
        for completed,color,label in [(False,'#de9a28','Unconfirmed'),(True,'#158765','Feedback confirmed')]:
            goals = [p['target'][:2] for p in cluster['points'] if (p['id'] in confirmed)==completed]
            if goals:
                xy = np.asarray(goals)+cfg['map_offset']
                ax.scatter(xy[:,0],xy[:,1],s=18,color=color,label=label)
    if task=='mowing':
        cluster = next(c for c in cfg['clusters'] if c['cluster']==spec['cluster'])
        for seg in cluster['segments']:
            xy = np.asarray(seg['poses_world'])
            ax.plot(xy[:,0],xy[:,1],color='.7',lw=.6,zorder=0)
    ax.set_aspect('equal',adjustable='datalim')
    ax.set_xlabel('Site X [m]')
    ax.set_ylabel('Site Y [m]')
    ax.grid(alpha=.25)
    if ax.get_legend_handles_labels()[0]:
        ax.legend(fontsize=8)


def build():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    out = APP/'results/R10'
    summary = read(out/'case_summary.json')
    cases = {r['case']:r for r in summary['cases']}
    fig,axes = plt.subplots(1,3,figsize=(16,5))
    views = [('spray_full_A1','spray','Point service / A1'),
             ('inspection_full_connector','inspection','Inspection / connector'),
             ('mowing_full_B1','mowing','Mowing / B1')]
    for ax,(name,task,title) in zip(axes,views):
        case = cases.get(name)
        if case is None:
            ax.set_title(title+' | NOT RUN')
            continue
        ax.set_title(title+' | '+('PASS' if case['passed'] else 'PARTIAL'))
        plot_case(ax,case,task)
    fig.tight_layout()
    fig.savefig(out/'r10_overview.png',dpi=150)
    plt.close(fig)

    rows = []
    for case in cases.values():
        seconds = case.get('simulation_seconds')
        fields = [case['case'],'PASS' if case['passed'] else 'PARTIAL',
                  f'{seconds:.2f}' if seconds is not None else '—',
                  result_amount(case),case.get('reason') or case.get('terminal') or '—']
        rows.append('<tr>'+''.join('<td>'+html.escape(str(cell))+'</td>' for cell in fields)+'</tr>')
    verdict = 'PASS' if summary['passed'] else 'PARTIAL'
    pending = ''.join('<li>'+html.escape(message)+'</li>' for message in summary['pending'])
    body = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>R10 三任务验收</title>
<style>body{{font:16px/1.7 sans-serif;max-width:1400px;margin:32px auto;padding:0 22px;color:#19332b}}
table{{border-collapse:collapse;width:100%}}td,th{{border:1px solid #cbd9d2;padding:8px;text-align:left}}
img{{width:100%}}th{{background:#e6efe9}}</style><h1>三项作业验收 · {verdict}</h1>
<p>路线到达、工具反馈、终态分别登记。持续封堵专项检查停车和人工交接。原 S17/S18 仍为 PARTIAL。</p>
<img src="r10_overview.png" alt="点喷、巡检、割草的实测轨迹和确认范围">
<table><tr><th>案例</th><th>结论</th><th>仿真秒</th><th>完成量与残余</th><th>终态或原因</th></tr>
{''.join(rows)}</table><h2>待处理</h2><ul>{pending}</ul></html>'''
    (out/'acceptance.html').write_text(body,encoding='utf-8')


if __name__=='__main__':
    build()
