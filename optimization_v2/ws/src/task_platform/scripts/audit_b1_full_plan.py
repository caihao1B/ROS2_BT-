"""Rebuild and verify the full 46-section mission that contains the four tested sections. Offline only."""
import argparse
import gzip
import json
import math
from pathlib import Path

from common import APP, ROOT, canonical, dump, edition, read, sha
from qualify_b1_context import mission
from safe_plan_r10 import requested_target, validate


def near(a,b):
    return math.dist(a[:2],b[:2])<=1e-6 and abs(math.remainder(a[2]-b[2],math.tau))<=1e-6


def evaluate(session,export=None):
    cfg,sections,offline=mission(paired=True)
    sample=read(session/'physical_connector_report.json')
    executed=read(session/'mission.json')
    cluster=next(x for x in cfg['clusters'] if x['cluster']=='B1')
    reference=read(edition('R09')/'mission.json')
    latest=read(edition('R10')/'mowing/mission.json')
    source=next(x for x in latest['clusters'] if x['cluster']=='B1')
    historical=next(x for x in reference['clusters'] if x['cluster']=='B1')
    inputs=read(edition('R10')/'mowing_safe_plan.json')
    inputs['cluster']=cluster
    full_geometry=validate(inputs,reference)
    old=source['segments']
    new=cluster['segments']
    offset=cfg['connector_qualification']['start_index']
    ids=[s['id'] for s in sections]
    full_target=requested_target(cluster,new,True)
    original_target=requested_target(historical,historical['segments'],True)
    old_work=[s for s in old if s['tool_on']]
    new_work=[s for s in new if s['tool_on']]
    original_keys=[s['id'] for s in historical['segments'] if s['tool_on']]
    passage=[]
    for a,b in zip(new,new[1:]):
        passage.append(near(a['poses'][-1],b['poses'][0]) and near(a['poses_world'][-1],b['poses_world'][0]))
    trial=read(session/'spec.json')
    tests=dict(exact_run_configuration=cfg==executed and cfg['version']==sample['plan_revision'],
        full_plan_geometry_validator=full_geometry['passed'],original_34_reference_status_partial=full_geometry['original_reference_status']=='PARTIAL',
        complete_46=len(new)==46 and len(old)==47,first_and_final_poses_unchanged=near(old[0]['poses'][0],new[0]['poses'][0]) and near(old[-1]['poses'][-1],new[-1]['poses'][-1])
          and near(source['start'],cluster['start']) and near(source['end'],cluster['end']),
        only_five_original_turns_replaced_by_four=new[:offset]==old[:offset] and new[offset+len(sections):]==old[offset+len(offline['covered_original_connector_ids']):],
        all_45_seams_continuous=len(passage)==45 and all(passage),single_gear_connector=[s['direction'] for s in sections]==[1,-1,1,-1]
          and all(not s['tool_on'] and s['requested_area_m2']==0 for s in sections),
        exact_10_approved_work_fragments=len(new_work)==10 and new_work==old_work,
        original_nine_obligations_retained=cluster['original_work_segments']==[s for s in historical['segments'] if s['tool_on']] and len(original_keys)==9,
        approved_omitted_work_retained=cluster['omitted_original_work']==source['omitted_original_work'] and
          {s['work_key'] for s in new_work}|{item['segment'] for item in cluster['omitted_original_work']}==set(original_keys),
        original_requested_geometry=full_target.symmetric_difference(original_target).area<1e-8,
        same_original_cluster_area_and_polygon=cluster['work_area_m2']==historical['work_area_m2'] and cluster['area_map']==historical['area_map'],
        map_identity_and_frame=cfg['map_id']==latest['map_id'] and cfg['map_revision']==latest['map_revision'] and cfg['map_offset']==latest['map_offset'],
        actual_qualified_subset_only=sample['passed'] and trial['segment_start']==offset and trial['segment_limit']==4 and
          len(sample['trajectory'])==4 and all(r['state']=='PHYSICALLY_VERIFIED' for r in sample['trajectory']) and [r['segment'] for r in sample['trajectory']]==ids)
    if not all(tests.values()):
        raise ValueError('Failed full-mission check: '+str([key for key,ok in tests.items() if not ok]))
    saved=None
    if export:
        # Preserve the exact, already verified bytes of the entire executed edition, including
        # its other clusters and vehicle/tool binding context. mtime=0 gives a stable archive.
        raw=(session/'mission.json').read_bytes()
        payload=gzip.compress(raw,compresslevel=9,mtime=0) if export.suffix=='.gz' else raw
        if export.exists() and export.read_bytes()!=payload:
            raise ValueError('Requested export already contains a different version')
        export.write_bytes(payload)
        saved=dict(filename=export.name,sha256=sha(export),uncompressed_sha256=sha(session/'mission.json'),size_bytes=len(payload))
    return dict(schema='B1-FULL-PLAN-PREFLIGHT/1',passed=True,tests=tests,candidate_version=cfg['version'],parent_47_version=latest['version'],
        source_geometry=offline['source_revision'],selected_connector_sha256=offline['candidate_sha256'],reference_version=reference['version'],
        published_map_sha256=cfg['map_revision'],segment_count=len(new),selected_indices_zero_based=list(range(offset,offset+4)),
        actual_strict_four_gear_report=sample['session'],actual_full_cluster_status='NOT_RUN_FOR_THIS_EDITION',
        original_full_cluster_area_m2=cluster['work_area_m2'],original_work_target_union_m2=original_target.area,target_symmetric_difference_m2=full_target.symmetric_difference(original_target).area,
        preserved_original_obligation_order=original_keys,
        approved_work_fragments=[dict(segment=s['id'],source_obligation=s['work_key']) for s in new_work],
        retained_work_without_safe_fragment=[dict(obligation=s['segment'],reason=s['reason']) for s in cluster['omitted_original_work']],
        whole_plan_distance_m=cluster['motion_m'],offline_expected_seconds=cluster['estimated_seconds'],original_sim_time_gate_s=cluster['budget_s'],
        start_map=new[0]['poses'][0],end_map=new[-1]['poses'][-1],nominal_connector_sections=ids,
        full_ordered_sections=[s['id'] for s in new],compressed_full_edition=saved,
        scope='Offline preflight of the full mission. The vehicle trial started at index 22 and drove exactly four turn sections. Full-cluster passage, cutting, coverage fraction and elapsed work time remain unmeasured for this edition.')


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('session',type=Path)
    ap.add_argument('--output',type=Path)
    ap.add_argument('--export',type=Path,help='Save the byte-identical complete JSON mission (.json or .json.gz).')
    args=ap.parse_args()
    result=evaluate(args.session.resolve(),args.export)
    if args.output:
        dump(args.output,result)
    print(json.dumps(result,indent=2))
