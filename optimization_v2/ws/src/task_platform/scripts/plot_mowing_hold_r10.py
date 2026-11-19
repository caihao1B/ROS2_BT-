"""Render the declared archived B1 yaw and GNSS stop window as an offline audit figure."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from replay_mowing_yaw_r10 import rows


def plot(run, output):
    limits=(1116.,1173.702)
    start_heading=1119.
    def segment(filename):
        return [r for r in rows(run/filename) if limits[0]<=r['t']<=limits[1]]
    reference=segment('reference/reference.jsonl')
    truth=segment('evaluation/truth.jsonl')
    map_rows=segment('quality/evidence.jsonl')
    plt.rcParams.update({'font.family':['Noto Sans CJK JP','DejaVu Sans'],'axes.spines.top':False,
                         'axes.spines.right':False,'axes.unicode_minus':False,'font.size':10})
    fig,(ax,gnss)=plt.subplots(2,1,figsize=(9,5.7),sharex=True,layout='constrained')
    fig.suptitle('B1 停车窗口中的航向与独立观测',fontsize=16,x=.07,ha='left')
    for samples,field,name,color in ((reference,'odom','局部里程计','#a64817'),(truth,'rpy','实际车身（仅离线评分）','#176b60')):
        t=np.array([r['t'] for r in samples]);angles=np.unwrap([r[field][2] for r in samples])
        ax.plot(t,np.rad2deg(angles-np.interp(start_heading,t,angles)),label=name,color=color,lw=2)
    ax.set_ylabel('相对 1119 s 的航向变化（°）')
    ax.legend(loc='lower left',ncol=2,frameon=False)
    measured=[r for r in map_rows if r.get('gnss_innovation_m') is not None]
    rejected=[r for r in measured if not r.get('valid')]
    accepted=[r for r in measured if r.get('valid')]
    for series,name,color in ((rejected,'未准入候选','#a64817'),(accepted,'已准入地图候选','#176b60')):
        gnss.scatter([r['t'] for r in series],[r['gnss_innovation_m'] for r in series],s=11,
                     color=color,label=name,zorder=3)
    no_measure=[r['t'] for r in map_rows if r.get('gnss_innovation_m') is None]
    gnss.plot(no_measure,np.zeros(len(no_measure))-.015,'|',ms=8,color='#637381',label='无可用配准候选')
    gnss.axhline(.15,c='#493779',ls='--',lw=1.2,label='0.15 m 准入门限')
    gnss.set_ylim(bottom=-.04);gnss.set_ylabel('原始地图与 GNSS 平面偏差（m）')
    gnss.set_xlabel('仿真时间（s）');gnss.legend(loc='upper right',ncol=2,frameon=True,fontsize=8)
    for a in (ax,gnss):
        a.axvline(1118.702,color='#475467',lw=.8,ls=':')
        a.set_xlim(*limits);a.grid(axis='y',alpha=.18)
    ax.annotate('车体与刀盘停车',xy=(1118.702,.3),xytext=(1125.,2.),fontsize=9,
                arrowprops=dict(arrowstyle='-',color='#475467',lw=.8))
    fig.savefig(output,dpi=150,facecolor='white')
    plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('out',type=Path)
    args=parser.parse_args();plot(args.run,args.out)
