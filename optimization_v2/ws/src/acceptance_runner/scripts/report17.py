"""Self-contained browser report of measured cases, including missing trials."""
from common17 import *
from statistics17 import wilson,quantiles
import yaml

def aggregate(checkpoints):
    matrix=yaml.safe_load((EVAL/'acceptance_matrix.yaml').read_text());by={c['spec']['id']:c for c in checkpoints};groups={};formal=[]
    for spec in matrix['formal']:
        r=by.get(spec['id']);m=r['metrics'] if r else dict(status='NOT_RUN',reason='No version-matched actual execution evidence')
        formal.append(dict(spec=spec,metrics=m,case_path=r['case_path'] if r else None))
    for c in ['A1','A2','A3','B1','B2','B3']:
        selected=[r for r in formal if r['spec']['cluster']==c];done=[r['metrics'] for r in selected if r['metrics']['status']!='NOT_RUN'];passed=sum(x['status']=='PASS' for x in done)
        groups[c]=dict(required=10,executed=len(done),passed=passed,partial=sum(x['status']=='PARTIAL' for x in done),failed=sum(x['status']=='FAIL' for x in done),success_wilson95=wilson(passed,len(done)),sim_seconds=quantiles([x['sim_seconds_including_stop'] for x in done]))
    missing=[r['spec']['id'] for r in formal if r['metrics']['status']=='NOT_RUN']
    # Each extra family is an explicit required result, never inferred from S16.
    extra=[*matrix['connector'],*matrix['faults'],*matrix['ablations']]
    extras=[dict(spec=s,metrics=by[s['id']]['metrics'] if s['id'] in by else dict(status='NOT_RUN'),case_path=by[s['id']]['case_path'] if s['id'] in by else None) for s in extra]
    geometry=by.get('geometry_variants',{}).get('metrics',dict(status='NOT_RUN'))
    complete=not missing and all(r['metrics']['status']=='PASS' for r in formal)
    all_extra=all(r['metrics']['status']=='PASS' for r in extras) and geometry['status']=='PASS'
    checks={f'S17-C{i:02}':'NOT_RUN' for i in range(1,6)}
    checks['S17-C01']='PASS' if complete and all_extra else 'NOT_RUN'
    checks['S17-C02']='PASS' if complete else ('FAIL' if any(r['metrics']['status']=='FAIL' for r in formal) else 'NOT_RUN')
    checks['S17-C03']='PASS' if complete and all_extra else 'NOT_RUN'
    checks['S17-C04']='PASS' if all(r['metrics']['status']=='PASS' for r in extras if r['spec'] in matrix['ablations']) else 'NOT_RUN'
    checks['S17-C05']='PASS' if complete and all_extra and 'resume_contract' in by and by['resume_contract']['metrics']['status']=='PASS' else 'NOT_RUN'
    formal_ids=[s['id'] for s in matrix['formal']]
    required={
        'R01':['geometry_variants']+[s['id'] for s in matrix['connector']],
        'R02':['partition'],
        'R03':formal_ids+['controller_diff','controller_ackermann'],
        'R04':formal_ids+['localization','fault_gnss_false_fixed','fault_gnss_occlusion','fault_localization_loss','fault_sensor_delay'],
        'R05':formal_ids+['fault_tool_loss','fault_mission_restart'],
        'R06':formal_ids,
        'R07':[s['id'] for s in matrix['faults']],
        'R08':formal_ids+['resume_contract','prediction','memory','coverage']}
    def requirement_status(ids):
        states=[by[i]['metrics']['status'] if i in by else 'NOT_RUN' for i in ids]
        return 'FAIL' if 'FAIL' in states else 'PARTIAL' if 'PARTIAL' in states else 'NOT_RUN' if 'NOT_RUN' in states else 'PASS'
    trace={key:dict(description=matrix['requirements'][key],status=requirement_status(ids),evidence=[dict(id=i,status=by[i]['metrics']['status'] if i in by else 'NOT_RUN',case_path=by[i]['case_path'] if i in by else None,checkpoint_identity=by[i]['identity'] if i in by else None) for i in ids]) for key,ids in required.items()}
    return dict(schema='S17-SUMMARY/1',status='PASS' if all(v=='PASS' for v in checks.values()) else 'PARTIAL',scope=matrix['scope'],clusters=groups,formal=formal,extra=extras,geometry=geometry,missing_formal=missing,checks=checks,requirements=matrix['requirements'],traceability=trace)

def render(out,data):
    view=dict(data)
    studies=[r for family in view['extra'] for r in family['metrics'].get('cases',[]) if isinstance(r,dict) and 'case_path' in r and 'spec' in r]
    for row in view['formal']+view.get('demo',[])+view['extra']+studies:
        p=ROOT/row['case_path']/'replay17.json' if row.get('case_path') else None
        if p and p.exists():
            row['replay']=read(p)
            row['replay']['events']=[e for e in row['replay'].get('events',[]) if e.get('event')!='mission_ownership']
            if 'points' not in row['replay'] and (p.parent/'client/obligations.json').exists():
                m=row['replay']['mission'];known={r['key'] for r in m.get('tool_results',[]) if r.get('state')=='SUCCEEDED' and r.get('quantity',0)>=2 and not r.get('uncertain',True)}
                row['replay']['points']=[dict(xy=j['target'],id=j['id'],state='CONFIRMED' if m.get('plan_revision','')+':'+j['id'] in known else 'UNCOMPLETED') for j in read(p.parent/'client/obligations.json')['jobs'] if j['kind']=='POINT_SERVICE']
    template=(PKG/'scripts/report17.html').read_text()
    (out/'index.html').write_text(template.replace('/*DATA*/',json.dumps(view,ensure_ascii=False).replace('</','<\\/')))
