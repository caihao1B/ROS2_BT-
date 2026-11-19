"""Draw the A1 per-stop clock and B1 entrance cut from the public measurements."""
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Patch
from shapely import from_wkb, union_all
from shapely.geometry import shape

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT/'validation/r10_bounded_20261119'


def load(path):
    raw = path.read_bytes()
    return json.loads(gzip.decompress(raw) if path.suffix=='.gz' else raw)


def fill_parts(axis, geometry, color, alpha):
    parts = [geometry] if geometry.geom_type == 'Polygon' else geometry.geoms
    for part in parts:
        if part.geom_type == 'Polygon' and not part.is_empty:
            x,y=part.exterior.xy
            axis.fill(x,y,facecolor=color,edgecolor=color,alpha=alpha,linewidth=.7)
            for hole in part.interiors:
                axis.fill(*hole.xy,color='white')


def main():
    cjk = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if cjk.is_file():
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=cjk).get_name()
    plt.rcParams.update({'axes.unicode_minus':False,'font.size':10})
    fig, (time_ax, map_ax) = plt.subplots(1,2,figsize=(15.8,5.4),width_ratios=[1.65,1],layout='constrained')
    measured = [r for r in load(EVIDENCE/'a1/point_phase_profile.json')['rows'] if r['elapsed_s'] is not None]
    layers=[('travel_s','导航','#285d77'),('precision_s','精停','#7bb1b3'),('localization_wait_s','定位等待','#e8a23d'),
            ('spray_flow_s','实际出液','#409b72'),('other_s','其他','#bac4c4')]
    accumulated=[0.]*len(measured)
    for key, label, color in layers:
        duration=[r[key] for r in measured]
        bars=time_ax.bar([r['ordinal'] for r in measured],duration,bottom=accumulated,width=.76,
                         label=label,color=color)
        bars[-1].set_hatch('///')
        bars[-1].set_edgecolor('#39424d')
        accumulated=[a+b for a,b in zip(accumulated,duration)]
    time_ax.set(title='A1 原点序逐站工时',xlabel='目标序号（第 27 点在导航中停车）',ylabel='仿真秒',
                xticks=[1,3,5,7,9,11,13,15,17,19,21,23,25,27],xlim=(.2,27.8))
    time_ax.set_axisbelow(True)
    time_ax.grid(axis='y',alpha=.18)
    time_ax.legend(ncols=5,loc='upper center',bbox_to_anchor=(.5,-.14),frameon=False)
    raw = load(EVIDENCE/'b1/area_and_stop_observations.json.gz')
    obs = load(EVIDENCE/'b1/entrance_observations.json')
    cfg = load(ROOT/'validation/r10_continuation_20261119/b1/mission_46.json.gz')
    xoff,yoff=cfg['map_offset']
    segment=next(c for c in cfg['clusters'] if c['cluster']=='B1')['segments'][0]
    swath=shape(raw['target_map'])
    footprints=union_all([shape(r['area']) for r in raw['rotor_evaluator_jobs'].values()])
    credited=union_all([from_wkb(bytes.fromhex(r['wkb_hex'])) for r in raw['sqlite_jobs'] if r.get('wkb_hex')])
    fill_parts(map_ax,swath,'#cdd9c9',.6)
    fill_parts(map_ax,footprints.intersection(swath),'#e9a849',.92)
    fill_parts(map_ax,credited.intersection(swath),'#44936d',.86)
    points=segment['poses']
    map_ax.plot(*zip(*(p[:2] for p in points)),ls='--',lw=1.3,color='#334459',label='第一条带中心线')
    map_ax.scatter(*points[0][:2],marker='o',s=38,facecolor='white',edgecolor='#2b424d')
    map_ax.scatter(*points[-1][:2],marker='x',s=56,color='#23566c')
    world=obs['stop']['world_xyz']
    stopped=(world[0]-xoff,world[1]-yoff)
    map_ax.scatter(*stopped,marker='s',s=32,color='#242732',zorder=8)
    for pos,label,offset in [(points[0],'入口',(0,-16)),(stopped,'停机',(0,10)),(points[-1],'第一终点',(0,10))]:
        map_ax.annotate(label,xy=pos[:2],xytext=offset,textcoords='offset points',ha='center')
    map_ax.set(title='B1 第一条带的实际割草与停机位置',xlabel='地图 X（m）',ylabel='地图 Y（m）',
               xlim=(points[0][0]-1.0,points[-1][0]+1.2),ylim=(points[0][1]-1.55,points[-1][1]+1.45),aspect='equal')
    map_ax.legend(handles=[Patch(color='#cdd9c9',label='原目标'),Patch(color='#e9a849',label='刀盘实割范围'),
                           Patch(color='#44936d',label='账本确认范围')],loc='upper center',bbox_to_anchor=(.5,-.18),
                  frameon=False,ncols=3)
    for ax in (time_ax,map_ax):
        ax.spines[['top','right']].set_visible(False)
    fig.savefig(EVIDENCE/'workload_measurements.png',dpi=145)
    plt.close(fig)


if __name__=='__main__':
    main()
