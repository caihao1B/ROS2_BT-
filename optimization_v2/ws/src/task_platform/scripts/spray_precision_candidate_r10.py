"""Run a hash-bound A1 precision candidate with original task, Nav2 and tool gates."""
import argparse
import importlib.util
from pathlib import Path

from common import APP, PKG, read
import terrain_pairing_r10 as pairing
from spray_defer_candidate_r10 import replace_one
from spray_handoff_candidate_r10 import build as build_parent


def build():
    parent = build_parent()
    source = replace_one(parent.read_text(), 'from docking_r10 import stable_dock',
                         'from spray_precision_r10 import StoppedWindow,LowSpeedCapture')
    first = source.index("            since=st['t'];wait(lambda:st['t']>=since+cfg['docking_settle_s']")
    last = source.index('            loc=st[\'loc\'] or {};distance=', first)
    source = source[:first] + """            window=StoppedWindow(p['target'],cfg['docking_settle_s'])
            stable=False;metrics={};observation_start=st['t']
            while st['t']-observation_start < 8.+cfg['docking_settle_s']:
                tick();loc=st['loc'] or {};gate=st['gate'] or {}
                if (st['bt'] or {}).get('action') in ['HUMAN','STOP']:raise RuntimeError('DOCKING_MANUAL_OR_SAFETY_HOLD '+str((st['bt'] or {}).get('reason_code')))
                if localization_hold():break
                stable,metrics=window.update(st['t'],loc,gate)
                if stable:break
                if attempt<cfg['docking_realign_max_attempts'] and offset_observable(metrics) and metrics['continuous_stopped_s']>=cfg['docking_settle_s']:
                    log('DOCKING_STABLE_OFFSET',point=p['id'],stability=metrics,work_admitted=False);break
            history=window.history
            if localization_hold():
                recover_localization(p)
                # Revalidate the new epoch from a full fresh stopped window.
                continue
""" + source[last:]
    source=replace_one(source,'        for attempt in range(3):\n',"""        attempt=0;observations=0
        # Two physical corrections and a bounded number of observation-only restarts.
        while observations < 2*cfg['docking_realign_max_attempts']+3:
            observations+=1
""")
    source=replace_one(source,"            future=active_nav.get_result_async();start=st['t'];interrupted=False;local_samples=[]",
                       "            attempt+=1\n            future=active_nav.get_result_async();start=st['t'];interrupted=False;local_samples=[]")
    source=replace_one(source,"approach=ApproachHandoff(cfg['global_approach_handoff']);handoff=None",
                       "approach=ApproachHandoff(cfg['global_approach_handoff']);capture=LowSpeedCapture(cfg['global_approach_handoff']['navigation_goal_m']);handoff=None")
    source=replace_one(source,"handoff=approach.update(loc.get('t',st['t']),loc['pose'],p['target'],qualified,gate.get('v',0.))",
                       "handoff=capture.update(st['t'],p['target'],loc,gate,qualified) or approach.update(loc.get('t',st['t']),loc['pose'],p['target'],qualified,gate.get('v',0.))")
    out=parent.parent.parent/'spray_precision_candidate'/'mission_spray_precision.py'
    out.parent.mkdir(parents=True,exist_ok=True)
    compile(source,str(out),'exec')
    out.write_text(source)
    return out


def run(seconds,rtf,first,count):
    here=PKG/'scripts'
    fixture_path=here/'spray_defer_fixture_r10.py'
    spec=importlib.util.spec_from_file_location('precision_fixture',fixture_path)
    fixture=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture)
    fixture.EXPERIMENTS=fixture.EXPERIMENTS.parent/'spray_precision'
    cfg=read(fixture.BASE_MISSION)
    full=next(c['points'] for c in cfg['clusters'] if c['id']=='A1')
    if not 0<=first<len(full) or not 2<=count<=len(full)-first:
        raise ValueError('Choose at least two consecutive original A1 points inside the 56-point plan')
    subset=full[first:first+count]
    fixture.FOUR=[p['id'] for p in subset]
    fixture.original_points=lambda:(read(fixture.BASE_MISSION),subset)
    binary=pairing.build()
    original_runtime=fixture.runtime_source
    def precision_runtime():
        template=original_runtime()
        template=replace_one(template,"require('R09');lock=lock_session()", "require('R09');point_limit=len(selected_ids);lock=lock_session()")
        return replace_one(template,"launch('terrain',[str(APP/'build/r08/r08_terrain')",
                           "launch('terrain',[str(APP/'"+str(binary.relative_to(APP))+"')")
    fixture.runtime_source=precision_runtime
    fixture.FIXTURE_SOURCES+=[here/name for name in ('spray_handoff_candidate_r10.py','mission_spray_r10.py',
                                                   'spray_precision_r10.py','spray_precision_candidate_r10.py','terrain_pairing_r10.py')]
    fixture.FIXTURE_SOURCES+=[pairing.INCLUDE,pairing.SOURCE,pairing.CMAKE,pairing.OUT/'terrain.cpp',binary]
    return fixture.build_runtime(build(),obstacle=False)(seconds=seconds,rtf=rtf)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('prepare','run'))
    parser.add_argument('--seconds',type=int,default=300)
    parser.add_argument('--rtf',type=float,default=.55)
    parser.add_argument('--first-index',type=int,default=21,help='Zero-based index in original A1 plan')
    parser.add_argument('--count',type=int,default=4)
    args=parser.parse_args()
    print((build(),pairing.build()) if args.mode=='prepare' else run(args.seconds,args.rtf,args.first_index,args.count))


if __name__=='__main__':
    main()
