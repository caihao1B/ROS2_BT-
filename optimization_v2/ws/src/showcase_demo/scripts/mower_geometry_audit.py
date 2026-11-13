"""Reproducible geometry/scale audit; no ROS, truth feed or simulation launch."""
from common_showcase import *
import argparse, math
import numpy as np
import yaml
from shapely.geometry import shape


def vehicle_metrics(v, qualified, tools):
    L=v['wheelbase'];track=v['wheel']['track'];limit=v['steering']['joint_limit_rad']
    radius=qualified['planning_min_radius_m']
    # Limit is the individual inner knuckle joint, not virtual bicycle steering.
    mechanical=L/math.tan(limit)+track/2
    control=1/v['steering']['max_curvature']
    outer=max(math.hypot(x,y-radius) for x,y in v['footprint'])
    return dict(reference='base_link / rear axle midpoint',wheelbase_m=L,track_m=track,
        footprint_length_m=np.ptp(np.array(v['footprint'])[:,0]).item(),
        footprint_width_m=np.ptp(np.array(v['footprint'])[:,1]).item(),
        body_size_m=v['body']['size'],tool_width_m=tools['virtual_effective_width_m'],
        inner_joint_limit_deg=math.degrees(limit),kinematic_joint_limit_radius_m=mechanical,
        commanded_min_radius_m=control,planning_radius_m=radius,
        qualified_sim_measured_radius_m=qualified['physical_min_radius_m'],
        planning_inner_steering_deg=math.degrees(math.atan(L/(radius-track/2))),
        rear_axle_turn_diameter_m=2*radius,footprint_outer_circle_diameter_m=2*outer,
        outer_diameter_note='constant-curvature full circle, configured footprint; excludes safety padding',
        physical_parameters_consistent=radius>=control>=mechanical,
        hardware_calibration='NOT_CALIBRATED')


def route_metrics(zone):
    domain=shape(zone['task_domain']);rows=[]
    for i,p in enumerate(zone['plans']):
        on=[];off=[]
        for s in p['segments']:
            if len(s.get('poses',[]))<2:continue
            length=float(np.linalg.norm(np.diff(np.asarray(s['poses'])[:,:2],axis=0),axis=1).sum())
            (on if s.get('tool_on') else off).append(length)
        work=sum(on);nonwork=sum(off)
        rows.append(dict(cluster='B'+str(i+1),area_m2=p['coverage_check']['domain_area_m2'],
            work_length_m=work,tool_off_length_m=nonwork,
            tool_off_distance_fraction=nonwork/(work+nonwork),
            individual_work_lengths_m=on,nominal_sim_seconds=p['timing']['total_s'],
            nominal_under_30_min=p['timing']['total_s']<=1800))
    return dict(task_area_m2=domain.area,task_bbox_m=list(domain.bounds),
        task_bbox_size_m=[domain.bounds[2]-domain.bounds[0],domain.bounds[3]-domain.bounds[1]],
        permitted_motion_area_m2=shape(zone['motion_domain']).area,clusters=rows,
        total_work_length_m=sum(x['work_length_m'] for x in rows),
        total_tool_off_length_m=sum(x['tool_off_length_m'] for x in rows))


def scale_warnings(metrics, vehicle):
    """Project design heuristics, not manufacturer limits or safety criteria."""
    warnings=[]
    if max(metrics['task_bbox_size_m'])<vehicle['rear_axle_turn_diameter_m']:
        warnings.append('TASK_SHORTER_THAN_TURN_DIAMETER')
    if any(x['tool_off_distance_fraction']>.75 for x in metrics['clusters']):
        warnings.append('TOOL_OFF_DISTANCE_DOMINATES_OVER_75_PERCENT')
    return warnings


def plot(old, candidate, report, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as Patch, Circle
    from matplotlib.lines import Line2D
    plt.rcParams['font.family']='DejaVu Sans'
    zones=[old['zones']['B']]
    if candidate:zones.append(candidate)
    fig,axes=plt.subplots(1,len(zones),figsize=(7*len(zones),7),squeeze=False)
    for idx,z in enumerate(zones):
        ax=axes[0,idx];g=shape(z['task_domain']);colors=['#69b5a6','#efc577','#af9cce']
        # Same world extent and aspect: changing the crop cannot disguise scale.
        ax.set(xlim=(117,138),ylim=(49,66),aspect='equal',xlabel='World/prior X (m)',ylabel='World/prior Y (m)')
        motion=shape(z['motion_domain'])
        for poly in ([motion] if motion.geom_type=='Polygon' else motion.geoms):
            ax.add_patch(Patch(poly.exterior.coords,facecolor='#edf0f2',edgecolor='#8395a2',lw=1))
        for j,f in enumerate(z['partition']['clusters']['features']):
            poly=shape(f['geometry']);ax.add_patch(Patch(poly.exterior.coords,facecolor=colors[j],edgecolor='#3b4e5c',lw=1))
            x,y=poly.representative_point().coords[0];ax.text(x,y,'B'+str(j+1),ha='center',va='center',fontsize=9)
        for p in z['plans']:
            for s in p['segments']:
                points=np.asarray(s.get('poses',[]))
                if len(points)<2:continue
                work=s.get('tool_on');ax.plot(points[:,0],points[:,1],color='#1879ad' if work else '#af6b30',lw=1.7 if work else .8,ls='-' if work else '--',alpha=.85)
        depot=z['plans'][0]['start'];ax.plot(depot[0],depot[1],'s',ms=5,color='black')
        ax.add_patch(Circle((120,52),report['vehicle']['planning_radius_m'],fill=False,edgecolor='#7d4d85',ls=':'))
        ax.text(120,52,'R = 2.11 m',ha='center',fontsize=8,color='#7d4d85')
        metrics=report['demo' if idx==0 else 'candidate']
        frac=metrics['total_tool_off_length_m']/(metrics['total_work_length_m']+metrics['total_tool_off_length_m'])
        ax.set_title(('Recorded v1: planned references' if idx==0 else 'Offline trial: efficiency still REJECTED')+f'\nTask {g.area:.2f} m² | tool-off distance {frac:.1%}',fontsize=12)
        ax.grid(alpha=.2)
    handles=[Line2D([0],[0],color='#1879ad',lw=2,label='Planned tool-on'),Line2D([0],[0],color='#af6b30',ls='--',label='Planned tool-off / depot travel'),Line2D([0],[0],color='#8395a2',lw=6,label='Permitted movement (not mowing credit)')]
    fig.legend(handles=handles,loc='lower center',ncol=3,fontsize=10)
    fig.suptitle('Same vehicle, same metre scale. No executed trajectory is inferred.',fontsize=14)
    fig.tight_layout(rect=[0,.05,1,.94]);fig.savefig(out/'scale_comparison.png',dpi=160);plt.close(fig)


def audit(candidate_path,out):
    vpath=BASE/'config/vehicles/ackermann.yaml';qpath=BASE/'config/vehicles/qualified_profiles.json';tpath=BASE/'config/tool_profiles.yaml'
    v=yaml.safe_load(vpath.read_text());q=read(qpath)['vehicles']['ackermann'];t=yaml.safe_load(tpath.read_text())['mower']
    oldpath=OUT/'v1/mission_plan.json';formalpath=BASE/'plans/s17_full/mission_plan.json'
    old=read(oldpath);formal=read(formalpath);vehicle=vehicle_metrics(v,q,t)
    candidate=read(candidate_path) if candidate_path and candidate_path.exists() else None
    report=dict(schema='MOWER-GEOMETRY-AUDIT/1',scope='offline analysis, not closed-loop qualification',
        vehicle=vehicle,demo=route_metrics(old['zones']['B']),formal=route_metrics(formal['zones']['B']),
        screenshot_gray_lines='planned reference, not measured trail',
        formal_s17_status='PARTIAL',s18_executed=False,simulation_started=False)
    report['demo_design_warnings']=scale_warnings(report['demo'],vehicle)
    if candidate:
        report['candidate']=route_metrics(candidate)
        report['candidate_design_warnings']=scale_warnings(report['candidate'],vehicle)
        report['candidate_representative_design_accepted']=not report['candidate_design_warnings']
        report['candidate_provenance_valid']=all((ROOT/p).is_file() and sha(ROOT/p)==h for p,h in candidate['provenance'].items())
        copy=dict(candidate);revision=copy.pop('revision')
        report['candidate_revision_valid']=canonical(copy)==revision
        report['candidate_checks']=dict(partition=candidate['partition']['verification']['passed'],
            routes=candidate['route_verification']['passed'],
            budgets=all(p['nominal_under_30_min'] for p in report['candidate']['clusters']),
            coverage=all(p['coverage_check']['fraction']>=.95 for p in candidate['plans']),
            reference_closed_loop_not_run=candidate['closed_loop_status']=='NOT_RUN',
            provenance=report['candidate_provenance_valid'],revision=report['candidate_revision_valid'])
    sources=[Path(__file__).resolve(),vpath,qpath,tpath,oldpath,formalpath]
    if candidate:sources.append(candidate_path)
    report['sources']={str(p.relative_to(ROOT)):sha(p) for p in sources}
    dump(out/'audit.json',report);plot(old,candidate,report,out)
    print(json.dumps({k:report[k] for k in ['vehicle','demo_design_warnings','candidate_design_warnings','candidate_checks'] if k in report},indent=2))
    if candidate and not all(report['candidate_checks'].values()):raise ValueError('CANDIDATE_AUDIT_FAILED')
    return report


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--candidate',type=Path,default=OUT/'mower_scale_v2c/candidate.json')
    ap.add_argument('--out',type=Path,default=OUT/'mower_geometry_review')
    ap.add_argument('--require-representative',action='store_true',help='Exit 3 if task scale or tool-off fraction fails project design heuristics')
    a=ap.parse_args()
    if not a.out.resolve().is_relative_to(OUT):raise ValueError('SHOWCASE_OUTPUT_SCOPE_REQUIRED')
    report=audit(a.candidate,a.out)
    if a.require_representative and not report.get('candidate_representative_design_accepted',False):
        print('REPRESENTATIVE_DESIGN_REJECTED: geometry PASS does not establish efficient mowing')
        raise SystemExit(3)
