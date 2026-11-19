"""Draw unmodified geometry from the two published B1 connector inputs."""
from pathlib import Path
import json
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from shapely.geometry import shape

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/task_platform/scripts'))
from recovery_mowing_r10 import gear_runs


def read(path):
    return json.loads(path.read_text())


if __name__ == '__main__':
    edition = ROOT/'optimization_v2/three_tasks/editions'
    safe = read(edition/'R10/mowing_safe_plan.json')
    offset = read(edition/'R09/mission.json')['map_offset']
    candidate = read(ROOT/'optimization_v2/three_tasks/candidates/B1_late_connector.json')
    report = read(ROOT/'validation/closeout_20261119/B1_offline_connector.json')
    approved = [s for s in safe['cluster']['segments'] if s['id'] in report['covered_original_connector_ids']]
    new = gear_runs(candidate['points'])
    fig, axes = plt.subplots(1,2,figsize=(11,5.3),sharex=True,sharey=True)
    obstacle = shape(safe['known_static_exclusion']['geometry']).exterior
    x,y = obstacle.xy
    points = [pt for section in [*approved,*new] for pt in section['poses']]
    xmin,ymin = [min(p[i] for p in points)-2 for i in (0,1)]
    xmax,ymax = [max(p[i] for p in points)+2 for i in (0,1)]
    for ax, sections, name, key in zip(axes, [approved,new], ['Approved connection','Independent candidate'], ['original','proposed']):
        ax.fill([u-offset[0] for u in x], [v-offset[1] for v in y],facecolor='#f8dcd3',edgecolor='#aa493d')
        for n,s in enumerate(sections,1):
            xs,ys = zip(*[p[:2] for p in s['poses']])
            ax.plot(xs,ys,'-' if s['direction']==1 else '--',color='#195e79' if s['direction']==1 else '#ad5813',lw=2.4)
            if n>1:ax.plot(xs[0],ys[0],marker='s',mfc='white',mec='#1d3035',ms=5)
        for i,j,marker,color in [(0,0,'o','#287547'),(-1,-1,'*','#922b4c')]:
            ax.plot(*sections[i]['poses'][j][:2],marker=marker,ms=10,color=color)
        metrics=report[key]
        ax.set_title(f"{name}\n{metrics['chord_length_m']:.2f} m, {metrics['switches_between_forward_work_strips']} gear changes")
        ax.set(xlim=(xmin,xmax),ylim=(ymin,ymax),xlabel='Map east (m)',aspect='equal')
        ax.grid(alpha=.2)
    axes[0].set_ylabel('Map north (m)')
    fig.legend(handles=[Line2D([],[],color='#195e79',label='Forward'),Line2D([],[],ls='--',color='#ad5813',label='Reverse'),
        Line2D([],[],marker='s',color='#1d3035',mfc='white',ls='',label='Gear change'),
        Line2D([],[],marker='o',color='#287547',ls='',label='Start'),Line2D([],[],marker='*',color='#922b4c',ls='',label='Finish'),
        Line2D([],[],color='#f0ad9f',lw=7,label='Known slope exclusion')],loc='lower center',ncol=6,frameon=False)
    fig.tight_layout(rect=(0,.08,1,1))
    fig.savefig(Path(__file__).with_name('B1_connector.png'),dpi=155)
