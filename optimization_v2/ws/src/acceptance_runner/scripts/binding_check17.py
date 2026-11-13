"""Require a separately qualified atlas before any formal six-cluster run."""
from common17 import *

def validate():
    from inputs17 import active
    paths=active();a=paths['atlas'];p=paths['plan']
    if not a.exists() or not p.exists():raise ValueError('S17_REGIONAL_EXTENSION_NOT_QUALIFIED')
    atlas=read(a);plan=read(p)
    if canonical({k:v for k,v in atlas.items() if k!='revision'})!=atlas['revision']:raise ValueError('ATLAS_HASH')
    if canonical({k:v for k,v in plan.items() if k!='plan_revision'})!=plan['plan_revision']:raise ValueError('PLAN_HASH')
    if not atlas['passed'] or plan['atlas_revision']!=atlas['revision']:raise ValueError('ATLAS_NOT_PASSED')
    for k,v in atlas['bindings'].items():
        q=(ROOT/k).resolve()
        if not q.is_relative_to(ROOT) or not q.exists() or sha(q)!=v:raise ValueError('BOUND_INPUT_CHANGED '+k)
    original=read(BASE/'plans/validated/mission_plan.json')
    for k in ['zones','connector_routes','task_revision','normal_budget_s','hard_budget_s']:
        if plan[k]!=original[k]:raise ValueError('ORIGINAL_OBLIGATION_CHANGED '+k)
    if set(plan.get('execution_routes',{}))!={'B1','B2','B3'}:raise ValueError('THREE_MEASURED_ACKERMANN_ROUTES_REQUIRED')
    return atlas,plan
