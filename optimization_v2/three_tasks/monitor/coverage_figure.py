"""Draw the approved Ackermann plan and its observed work credit in the same map frame."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, PathPatch
from matplotlib.path import Path as PlotPath
from shapely.geometry import shape, Polygon
from shapely.geometry.polygon import orient


def fill(ax, geometry, color):
    pieces = [geometry] if isinstance(geometry, Polygon) else geometry.geoms
    for polygon in pieces:
        polygon = orient(polygon, sign=1.)
        paths = [PlotPath(list(ring.coords), closed=True) for ring in [polygon.exterior,*polygon.interiors]]
        ax.add_patch(PathPatch(PlotPath.make_compound_path(*paths), facecolor=color, edgecolor='none'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    def read(name):
        return json.loads((args.run/name).read_text(encoding='utf-8'))
    spec = read('spec.json')
    cluster = next(c for c in read('mission.json')['clusters'] if c['cluster'] == spec['cluster'])
    evidence = read('analysis_mowing_r10.json')
    regions = {e['properties']['kind']:shape(e['geometry']) for e in read('coverage.geojson')['features']}
    colors={'target':'#c9d6c7','confirmed':'#27785d','residual':'#e2aa5a','work':'#193f3c','turn':'#828b96','track':'#193e76'}
    cjk = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if cjk.exists():
        font_manager.fontManager.addfont(cjk)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=cjk).get_name()
    fig, axes = plt.subplots(1,2,figsize=(12,4.3),sharex=True,sharey=True)
    for a in axes:
        a.set_aspect('equal', adjustable='box')
        a.set_xlabel('地图 X（米）')
        a.set_ylabel('地图 Y（米）')
        a.grid(alpha=.16)
    fill(axes[0],regions['selected_target'],colors['target'])
    for seg in cluster['segments']:
        x,y=zip(*(p[:2] for p in seg['poses']))
        axes[0].plot(x,y,color=colors['work' if seg['tool_on'] else 'turn'],lw=1.45 if seg['tool_on'] else .95,
                     ls='-' if seg['tool_on'] else '--')
    axes[0].set_title('安全执行路径 · '+str(len(cluster['segments']))+' 段',loc='left')
    axes[0].legend(handles=[Patch(color=colors['target'],label='原始作业目标'),
                           Line2D([],[],color=colors['work'],label='开刀条带'),Line2D([],[],ls='--',color=colors['turn'],label='关刀连接')],
                   loc='upper center',bbox_to_anchor=(.5,-.19),ncol=3,frameon=False,fontsize=9)
    fill(axes[1],regions['residual'],colors['residual'])
    fill(axes[1],regions['confirmed_feedback'],colors['confirmed'])
    poses=[];last=None
    with (args.run/'reference/reference.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if not row.get('valid') or not row.get('pose'):
                if poses and poses[-1] is not None:
                    poses.append(None)
                continue
            if last is None or row['t']-last>=.5:
                poses.append(row['pose']);last=row['t']
    x=[p[0] if p else float('nan') for p in poses];y=[p[1] if p else float('nan') for p in poses]
    axes[1].plot(x,y,color=colors['track'],lw=1.05)
    axes[1].set_title(f"工具反馈 · {evidence['selected_confirmed_union_m2']:.2f} / {evidence['selected_requested_area_m2']:.2f} m²",loc='left')
    axes[1].legend(handles=[Patch(color=colors['confirmed'],label='已确认'),Patch(color=colors['residual'],label='残余'),
                            Line2D([],[],color=colors['track'],label='定位轨迹')],
                   loc='upper center',bbox_to_anchor=(.5,-.19),ncol=3,frameon=False,fontsize=9)
    fig.suptitle(spec['cluster']+' 割草 · 地头转向与有效作业面积',fontsize=16)
    fig.tight_layout(rect=(0,.04,1,.97))
    fig.savefig(args.output,dpi=135,bbox_inches='tight')
    plt.close(fig)


if __name__=='__main__':
    main()
