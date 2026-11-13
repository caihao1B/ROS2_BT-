"""Readable current evidence index; development runs never become formal rows."""
from common17 import *
from inputs17 import active
from report17 import aggregate
from checkpoint17 import Journal
from stage17 import publish
import argparse,numpy as np

def figure(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from shapely.geometry import shape
    from shapely.affinity import translate
    plan=read(BASE/'plans/validated/mission_plan.json');fig,axes=plt.subplots(1,3,figsize=(15,5),gridspec_kw=dict(width_ratios=[1.8,1,1]))
    colors=['#177fab','#20956c','#d57b37']
    for ax in axes:
        ax.set_aspect('equal');ax.grid(alpha=.18);ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
    for f in read(BASE/'maps/prior/workspaces.geojson')['features']:
        geom=translate(shape(f['geometry']),xoff=-28,yoff=-14)
        if geom.geom_type=='Polygon':axes[0].fill(*geom.exterior.xy,color='#eeeeea',edgecolor='#9b9c99',lw=.8)
    route=next(r for r in plan['connector_routes'] if r['vehicle']=='diff' and r['direction']=='A_to_B');xy=np.array([q[:2] for s in route['segments'] for q in s['poses']])-[28,14]
    axes[0].plot(*xy.T,color='#5b6f8b',lw=1.3,label='Approved inspection reference')
    cfg=read(RESULTS/'corridor_qualification/stations.json')
    for name,s in cfg['stations'].items():
        xy=np.array(read(RESULTS/'corridor_qualification/survey'/name/'registration_C17.json')['map_pose'][:2]);held=name.startswith('H')
        axes[0].scatter(*xy,marker='s' if held else 'o',color='#c35a74' if held else '#348a81',s=18,zorder=5,label='Independent heldout' if name=='H00' else 'Mapping station' if name=='C00' else None)
    for z,ax in zip(['A','B'],axes[1:]):
        region=translate(shape(plan['zones'][z]['task_domain']),xoff=-28,yoff=-14)
        for plot in [axes[0],ax]:plot.plot(*region.exterior.xy,color='#333',lw=1.)
        for i,c in enumerate(plan['zones'][z]['plans']):
            if z=='A':
                pts=np.array([s['target_xy'] for s in c['segments'] if s['kind']=='POINT_SERVICE'])-[28,14]
                for plot in [axes[0],ax]:plot.scatter(*pts.T,color=colors[i],s=18 if plot==ax else 8,label=z+str(i+1) if plot==ax else None)
            else:
                polygon=translate(shape(c['coverage_check']['covered']),xoff=-28,yoff=-14)
                parts=list(polygon.geoms) if polygon.geom_type=='MultiPolygon' else [polygon]
                for plot in [axes[0],ax]:
                    for k,g in enumerate(parts):plot.fill(*g.exterior.xy,color=colors[i],alpha=.65,label=z+str(i+1) if plot==ax and k==0 else None)
        ax.set_title(z+' / frozen S12 task domains');ax.legend(fontsize=8,loc='upper center',bbox_to_anchor=(.5,-.18),ncol=3)
    axes[0].set_title('Continuous prior and measured road registration');axes[0].legend(fontsize=7,loc='upper center',bbox_to_anchor=(.5,-.18),ncol=2)
    fig.suptitle('S17 qualification geometry — not completed coverage or executed trajectories',fontsize=12)
    fig.tight_layout();fig.savefig(out/'qualification.png',dpi=150,bbox_inches='tight');plt.close(fig)

def main(rtf=1.):
    out=RESULTS/'current_overview';out.mkdir(exist_ok=True)
    binding=dict(sources=source_binding(),dependencies=dependencies(),assets={str(p.relative_to(ROOT)):sha(p) for p in active().values()},execution=dict(requested_rtf=rtf))
    data=aggregate(Journal(EVAL/'checkpoints').collect(binding));data['scope']='本轮正式矩阵针对 S12 冻结的六个限时作业域；大场地其余区域仍未作业。下方开发测试不计入 60 次正式成绩。'
    cases=[]
    for folder in [*sorted(RESULTS.glob('demo_*/*')),*sorted(RESULTS.glob('pilot_*')),*sorted(RESULTS.glob('study_probe_*'))]:
        if not (folder/'metrics17.json').exists():continue
        if (folder/'spec.json').exists():spec=read(folder/'spec.json')
        elif folder.name.startswith('study_probe_') and (folder/'command.json').exists():
            command=read(folder/'command.json');argv=command['argv'];spec=dict(id=folder.name,seed=int(argv[argv.index('--seed')+1]),scope='DEVELOPMENT_PROBE_NOT_FORMAL',provenance='identity reconstructed from retained executed command',formal_credit=False)
        else:continue
        cases.append(dict(spec=spec,metrics=read(folder/'metrics17.json'),case_path=str(folder.relative_to(ROOT))))
    corridor=read(RESULTS/'corridor_qualification/qualification.json')
    data['demo']=cases;data['development_findings']=dict(corridor_registration={k:corridor[k] for k in ['passed','train_count','heldout_count','errors_m','truth_used_for_registration']},partition=read(RESULTS/'partition_comparison/summary17.json'),formal_executed=60-len(data['missing_formal']))
    if (out/'failure_evidence17.json').exists():data['development_findings']['rejected_candidate']=read(out/'failure_evidence17.json')
    checks=RESULTS/'component_contracts/current/verification17.json'
    if checks.exists():data['development_findings']['component_verification']=read(checks)
    data['error']='完整矩阵、两种底盘连接路及规定故障/配对消融尚未全部通过；开发测试保留原始失败与残余。'
    figure(out);dump(out/'binding.json',binding);publish(out,data,binding)
    return out
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--rtf',type=float,default=1.);a=p.parse_args();print(main(a.rtf))
