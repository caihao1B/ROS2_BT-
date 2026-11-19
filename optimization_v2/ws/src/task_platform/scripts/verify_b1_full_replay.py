"""Replay the B1 physical score on four previously qualified real stops and a partial entrance."""
import argparse
from collections import defaultdict
from copy import deepcopy
import gzip
import json
from pathlib import Path

from audit_b1_entrance import replay_area
from common import ROOT, dump, read, sha
from score_b1_full import FROZEN, Timeline, stop_row
from score_b1_connector import rows

PUBLIC=ROOT/'validation/r10_bounded_20261119/b1'
STREAMS={
    'evaluation/truth.jsonl': ('t','xyz','rpy'),
    'tool_evaluation/physical.jsonl': ('t','body_v','rpm_rad_s','on','healthy'),
    'control/gate.jsonl': ('t','mode','v','tool_stopped','stopped'),
}


def export_inputs(directory,destination):
    spec=read(directory/'spec.json')
    result=read(directory/'mission_result.json')
    receipt=read(FROZEN/'physical_connector_report.json')
    if directory.name!=receipt['session'] or spec['segment_start']!=22 or spec['segment_limit']!=4:
        raise ValueError('Unexpected four-gear source')
    record=dict(session=directory.name,plan_revision=receipt['plan_revision'],
                spec={'segment_start':22,'segment_limit':4},completed_segments=result['completed_segments'],
                events=[e for e in rows(directory/'mission_events.jsonl') if e.get('segment') in result['completed_segments']],
                streams={},full_source_sha256={p:sha(directory/p) for p in (*STREAMS,'mission_events.jsonl','mission_result.json','spec.json')})
    for filename,fields in STREAMS.items():
        record['streams'][filename]=[[r[k] for k in fields] for r in rows(directory/filename)]
    destination.write_bytes(gzip.compress(json.dumps(record,separators=(',',':'),allow_nan=False).encode(),mtime=0))


def verify(directory=PUBLIC):
    path=directory/'four_gear_replay_inputs.json.gz'
    source=json.loads(gzip.decompress(path.read_bytes()))
    frozen=json.loads(gzip.decompress((FROZEN/'mission_46.json.gz').read_bytes()))
    old=read(FROZEN/'physical_connector_report.json')
    cluster=next(c for c in frozen['clusters'] if c['cluster']=='B1')
    sections=cluster['segments'][source['spec']['segment_start']:][:source['spec']['segment_limit']]
    events=defaultdict(list)
    for e in source['events']:
        events[e['segment']].append(e)
    data={name:[dict(zip(keys,row)) for row in source['streams'][name]] for name,keys in STREAMS.items()}
    timeline=[Timeline(v) for v in data.values()]
    accepted=[]
    for i,s in enumerate(sections):
        row=stop_row(s,events[s['id']],source,frozen,timeline,sections[i-1]['direction'] if i else None,
                     sections[i+1]['direction'] if i+1<len(sections) else None)
        expected=old['trajectory'][i]['parking_window']
        assert row['state']=='PHYSICALLY_VERIFIED'
        assert row['max_xy_m']==expected['max_physical_xy_m'] and row['max_yaw_rad']==expected['max_physical_yaw_rad']
        accepted.append(dict(segment=row['segment'],xy_m=row['max_xy_m'],yaw_rad=row['max_yaw_rad'],gear=row['measured_drive_directions']))
    first=sections[0]
    sid=first['id']
    held=[e for e in events[sid] if e['event']=='STOPPED_ORIGINAL_ARRIVAL_CONFIRMED'][-1]
    end=held['t']
    time_window=(end-held['span_s'],end)
    truth=deepcopy(timeline[0].records)
    altered=next(p for p in truth if time_window[0]+.1<=p['t']<=time_window[1]-.1)
    altered['rpy'][2]+=.20
    yaw=stop_row(first,events[sid],source,frozen,[Timeline(truth),timeline[1],timeline[2]],None,-1)
    no_rotor=stop_row(first,events[sid],source,frozen,[timeline[0],Timeline([]),timeline[2]],None,-1)
    wrong=[dict(g,v=-g['v']) for g in timeline[2].records]
    gear=stop_row(first,events[sid],source,frozen,[timeline[0],timeline[1],Timeline(wrong)],None,-1)
    partial=read(directory/'full_46_physical_report.json')
    area=replay_area(directory/'area_and_stop_observations.json.gz')
    checks=dict(
       candidate_and_route_identity=source['plan_revision']==frozen['version']==old['plan_revision'] and
            [s['id'] for s in sections]==source['completed_segments'] and len(accepted)==4,
       four_positions_headings_equal_sealed_values=True,
       changed_heading_rejected=yaw['state']=='PARKING_NOT_CONFIRMED' and not yaw['checks']['physical_yaw'],
       missing_rotor_rejected=no_rotor['state']=='PARKING_NOT_CONFIRMED' and not no_rotor['checks']['sampled'],
       opposite_gear_rejected=gear['state']=='PARKING_NOT_CONFIRMED' and not gear['checks']['strict_original_gear_and_route'],
       actual_entrance_not_promoted=(partial['status']=='PARTIAL' and partial['total_segments']==46 and
            partial['completed_segments']==partial['parking_verified']==0 and partial['formal_credit'] is False),
       no_credit_for_unmeasured_positions=all(r['state'] in ('ATTEMPTED','NOT_STARTED') and 'max_xy_m' not in r and
            'max_yaw_rad' not in r for r in partial['parking']),
       scoped_physical_and_ledger_areas=all(abs(partial[k]-v)<1e-8 for k,v in area.items()) and
            0<area['confirmed_union_m2']<=area['actual_cut_m2']<area['target_m2'],
       retained_uncut_work=not partial['checks']['measured_feedback_for_each_selected_work_fragment'] and
            sum(f['measured'] for f in partial['work_fragments'])==1)
    if not all(checks.values()):
        raise ValueError(f'B1 replay failed: {checks}')
    return dict(schema='B1-SCORER-REGRESSION/1',passed=True,checks=checks,
                earlier_four_gear_run=source['session'],scope='Earlier four real sensor stops with intentional offline mutations. No simulated distance added.',
                earlier_stops=accepted,partial_entrance=partial['session'],areas_m2=area,bundle_sha256=sha(path))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--export-old-inputs',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.export_old_inputs:
        export_inputs(args.export_old_inputs.resolve(),PUBLIC/'four_gear_replay_inputs.json.gz')
    result=verify()
    if args.output:
        dump(args.output,result)
    print(json.dumps(result,indent=2))
