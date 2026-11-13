"""Publish qualification progress without launching or disturbing a simulator."""
from common17 import *
from report17 import aggregate,render
from stage17 import publish
from checkpoint17 import Journal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

def main():
    out=RESULTS/'qualification_progress';out.mkdir(parents=True,exist_ok=True);plan=read(BASE/'plans/validated/mission_plan.json');atlas=read(BASE/'maps/s14_regions/manifest.json');cfg=read(RESULTS/'qualification/stations.json');summary={}
    fig,axes=plt.subplots(1,2,figsize=(12,6))
    for ax,z in zip(axes,['A','B']):
        for i,p in enumerate(plan['zones'][z]['plans']):
            for j,s in enumerate(p['segments']):
                xy=np.array(s['poses'])[:,:2]-[28,14];ax.plot(*xy.T,lw=.5,alpha=.7,color=['#2c7fb8','#42aa8b','#da854b'][i],label=z+str(i+1)+' planned route' if j==0 else None)
        lo,hi=np.array(atlas['regions'][z]['site_extent_xy']);ax.add_patch(Rectangle(lo,*(hi-lo),fill=False,ec='#785e91',lw=2,ls='--',label='S14 qualified boundary'))
        for name,s in cfg['stations'].items():
            if s['region']!=z:continue
            p=RESULTS/'qualification/survey'/name;f=p/'qualification17.json'
            v=read(f) if f.exists() else {};summary[name]={k:v[k] for k in ['passed','residual_m','temporal_scatter_m','error'] if k in v};summary[name]['capture_completed']=(p/'runtime.json').exists() and read(p/'runtime.json').get('success',False)
            xy=np.array(v['map_pose'][:2]) if 'map_pose' in v else np.array(s['prior_pose'][:2])-[28,14]
            ax.scatter(*xy,s=50,c='#118765' if v.get('passed') else '#c65242' if v else '#9099a0',edgecolors='white',zorder=5);ax.annotate('Q-'+name,xy,xytext=(3,4),textcoords='offset points',fontsize=8)
        ax.set_aspect('equal');ax.set(title=z+' / planned task and measured station checks',xlabel='map x (m)',ylabel='map y (m)');ax.legend(fontsize=7,loc='upper center',bbox_to_anchor=(.5,-.16),ncol=2);ax.grid(alpha=.15)
    fig.tight_layout();fig.savefig(out/'qualification.png',dpi=140);plt.close(fig)
    dump(RESULTS/'qualification/surface_qualification.json',dict(stations=summary,required=len(cfg['stations']),executed=sum('passed' in s for s in summary.values()),passed=len(summary)==16 and all(s.get('passed') for s in summary.values()),scope='Fresh stationary sensor qualification; not full-cluster trials'))
    binding=dict(sources=source_binding(),dependencies=dependencies(),assets={});data=aggregate(Journal(EVAL/'checkpoints').collect(binding));data['error']='区域扩展资格正在验证；正式矩阵和必需消融尚未全部执行。';publish(out,data,binding)
    dump(out/'qualification_stations.json',summary)
    return out
if __name__=='__main__':print(main())
