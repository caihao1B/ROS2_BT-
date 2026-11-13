"""Reuse measured safety/localization auditing with an explicit new denominator."""
from common_showcase import *
import types

def evaluate(out,spec):
    from overlay17 import replace
    source=(BASE/'ws/src/acceptance_runner/scripts/evaluate17.py').read_text()
    source=replace(source,"plan=read(BASE/'plans/validated/mission_plan.json')","plan=read(Path(spec['showcase_plan']))")
    if spec.get('resume_from') and spec['operation']=='inspection':
        source=replace(source,"from inspection_audit17 import audit","from inspection_recovery import audit_chain\n        from functools import partial\n        audit=partial(audit_chain,previous=ROOT/spec['resume_from'])")
    module=types.ModuleType('showcase_measured_audit');exec(compile(source,str(Path(__file__)),'exec'),module.__dict__)
    result=module.evaluate(out,spec,BASE/'build_s17/monitor');mission=read(out/'client/mission.json') if (out/'client/mission.json').exists() else {};plan=read(Path(spec['showcase_plan']))
    runtime=read(out/'runtime.json');binding=runtime.get('regional_binding',{})
    version=mission.get('plan_revision')==plan['plan_revision']==binding.get('plan_revision')==spec['plan_revision']
    result.update(scope='INDEPENDENT_SHOWCASE_ONLY',formal_s17_credit=False,plan_revision=plan['plan_revision'],declared_demo_denominator=result.pop('original_denominator'),demo_version_valid=version)
    if not version:result.update(status='FAIL',complete=False,reason='DEMO_RUNTIME_VERSION_MISMATCH')
    handoff=read(out/'client/manual_handoff.json') if (out/'client/manual_handoff.json').exists() else None
    result['manual_handoff']=dict(required=bool(handoff and handoff.get('requires_manual')),stopped_confirmed=bool(handoff and handoff.get('stop_confirmed')),reason=handoff.get('reason') if handoff else None)
    if handoff and handoff.get('requires_manual') and result['status']!='PASS':result['operational_outcome']='SAFE_MANUAL_HANDOFF' if result['safety'] and handoff.get('stop_confirmed') else 'MANUAL_HANDOFF_UNCONFIRMED'
    if spec.get('resume_from'):
        previous=ROOT/spec['resume_from'];proof=read(out/'showcase_resume_proof.json');restored=read(out/'client/restored17.json');prior=read(previous/'showcase_result.json');manifest=read(out/'resume_source.json')
        valid=proof['source']==spec['resume_from']==restored['source'] and proof['source_result_sha256']==sha(previous/'showcase_result.json') and proof['source_spec_sha256']==sha(previous/'spec.json') and proof['manifest_sha256']==sha(out/'resume_source.json') and restored['plan_revision']==plan['plan_revision'] and prior['safety'] and prior['localization_pass'] and all(sha(ROOT/p)==h for p,h in manifest['source_hashes'].items())
        result['resume_chain_verified']=bool(valid);result['resumed_from']=spec['resume_from'];result['execution_episodes']=prior.get('execution_episodes',[spec['resume_from']])+[str(out.relative_to(ROOT))]
        if not valid:result.update(status='FAIL',complete=False,reason='RESUME_EVIDENCE_CHANGED')
    else:result['execution_episodes']=[str(out.relative_to(ROOT))]
    result['evaluation_source_sha256']=sha(Path(__file__))
    if runtime.get('error'):
        result['runtime_termination']=runtime['error']
        if 'case wall watchdog' in runtime['error'] and not result['complete']:
            result['last_bt_reason']=result['reason'];result['reason']='WALL_BUDGET_STOP'
            result['termination_scope']='external hard wall deadline; not evidence of an impassable route'
    dump(out/'metrics17.json',result);dump(out/'showcase_result.json',result);return result
