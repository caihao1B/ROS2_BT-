"""Static diagram of declared tasks, computed partitions and offline routes."""
from common_showcase import *
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def main():
    p=read(OUT/'v1/mission_plan.json');colors=['#319a93','#e6a43b','#9065bd'];fig,axes=plt.subplots(1,3,figsize=(16,5),gridspec_kw={'width_ratios':[1.3,1,1]});ax=axes[0]
    for zone in ['A','B']:
        z=p['zones'][zone];xy=np.asarray(z['motion_domain']['coordinates'][0]);ax.fill(*xy.T,alpha=.12,color='#536675');q=np.asarray(z['task_domain']['coordinates'][0]);ax.fill(*q.T,color='#157d84' if zone=='A' else '#8c64aa');ax.text(*q.mean(axis=0),zone,fontsize=13)
    route=next(r for r in p['connector_routes'] if r['vehicle']=='diff' and r['direction']=='A_to_B')
    for s in route['segments']:
        q=np.asarray(s['poses']);ax.plot(q[:,0],q[:,1],color='#dd8d32',lw=2)
    ax.set_title('Unchanged map / full inspection corridor');ax.set_xlabel('Declared ENU x (m)');ax.set_ylabel('Declared ENU y (m)')
    for i,zone in enumerate(['A','B'],1):
        ax=axes[i];z=p['zones'][zone]
        for k,f in enumerate(z['partition']['clusters']['features']):
            xy=np.array(f['geometry']['coordinates'][0]);ax.add_patch(Polygon(xy,facecolor=colors[k],edgecolor='white',alpha=.5));label=f['properties']['label_point'];ax.text(*label,zone+str(k+1),ha='center',fontweight='bold')
        for k,plan in enumerate(z['plans']):
            if zone=='A':
                xy=np.array([v['xy'] for v in plan['points']]);ax.scatter(*xy.T,c=colors[k],s=42,marker='x')
            else:
                for s in plan['segments']:
                    if s['tool_on']:
                        xy=np.array(s['poses']);ax.plot(xy[:,0],xy[:,1],color=colors[k],lw=2)
        ax.autoscale();ax.set_title(('Point service: 6 targets' if zone=='A' else 'Mowing: 4.104 m²')+' / computed 3 clusters');ax.set_xlabel('Declared ENU x (m)')
    for ax in axes:ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2)
    fig.suptitle('Independent demonstration tasks · geometry eligibility only · measured completion audited separately',fontsize=13);fig.tight_layout();fig.savefig(OUT/'v1/plan_overview.png',dpi=160)
if __name__=='__main__':main()
