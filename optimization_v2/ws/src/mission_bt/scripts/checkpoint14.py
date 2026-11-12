"""Reuse only completed full-cluster evidence with identical code and inputs.
This is execution evidence reuse, never a shortcut around a failed obligation.
"""
from common14 import *
from common11 import files

def complete(case,result):
    m=result.get('mission',{})
    if not result.get('passed') or not result.get('completed') or result.get('sim_seconds',1e9)>1800:return False
    if case=='nominal_spray_cluster':return m.get('required_points')==21 and m.get('confirmed_points')==21
    if case=='nominal_mow_cluster':return abs(m.get('required_area_m2',0)-16.35945787959566)<1e-5 and m.get('coverage_fraction',0)>=.95
    return False

def save(out,case,result,binding,dependencies):
    if not complete(case,result):return
    proof=dict(case=case,source_snapshot=binding,dependencies=dependencies,complete=True,
               evidence={str(p.relative_to(out)):sha(p) for p in files(out) if p.name!='checkpoint.json'})
    dump(out/'checkpoint.json',proof)

def reuse(parents,out,case,binding,dependencies):
    for parent in parents:
        source=(parent/case).resolve()
        if not source.is_relative_to(BASE/'results/S14') or source==out.resolve():raise ValueError('UNSAFE_CHECKPOINT_PATH')
        if not (source/'checkpoint.json').is_file():continue
        proof=read(source/'checkpoint.json')
        if proof.get('source_snapshot')!=binding or proof.get('dependencies')!=dependencies or proof.get('case')!=case or not proof.get('complete'):
            print('S14 CHECKPOINT DECLINED (version mismatch) '+str(source),flush=True);continue
        valid=True
        for name,h in proof['evidence'].items():
            p=(source/name).resolve()
            if not p.is_relative_to(source) or not p.is_file() or sha(p)!=h:valid=False;break
        if not valid:
            print('S14 CHECKPOINT DECLINED (evidence changed) '+str(source),flush=True);continue
        out.mkdir(parents=True)
        # Evidence is read-only after completion; hard links avoid duplicating
        # sensor logs. The new checkpoint itself is a separate file.
        for name in proof['evidence']:
            target=out/name;target.parent.mkdir(parents=True,exist_ok=True);os.link(source/name,target)
        dump(out/'checkpoint.json',dict(proof,reused_from=str(source)))
        print('S14 VERIFIED CHECKPOINT '+str(source),flush=True);return str(source)
    return None
