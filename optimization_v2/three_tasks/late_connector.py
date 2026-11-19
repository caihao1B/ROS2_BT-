#!/usr/bin/env python3
"""Recheck an independent, tool-off B1 connector against the published full-plan geometry."""
import argparse
import copy
import json
import math
from pathlib import Path
import sys
import subprocess
from tempfile import TemporaryDirectory

from shapely.geometry import mapping, shape

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / 'optimization_v2/three_tasks'
SCRIPTS = ROOT / 'optimization_v2/ws/src/task_platform/scripts'
sys.dont_write_bytecode = True
sys.path.insert(0, str(SCRIPTS))
from common import canonical, read, sha
from footprint_r09 import body_polygon
from recovery_mowing_r10 import gear_runs, yaw_error
from safe_plan_r10 import load_checked, validate

ROUTE = APP / 'candidates/B1_late_connector.json'
PREFIX = 'B1:safe_turn:6:'


def arc_length(points):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))


def measure(points, envelope, authority, offset):
    """Clearance is measured at native primitive samples. The separate lower bound covers the arcs between them."""
    boundary = authority.boundary
    sampled_clearance = []
    for x, y, heading in points:
        body = body_polygon(envelope, x + offset[0], y + offset[1], heading)
        if not authority.covers(body):
            raise ValueError('Padded body crosses the original authority.')
        sampled_clearance.append(body.distance(boundary))
    radius = max(math.hypot(x, y) for x, y in envelope)
    lower_bound = math.inf
    for i, (a, b) in enumerate(zip(points, points[1:])):
        chord = math.dist(a[:2], b[:2])
        angle = yaw_error(b[2], a[2])
        if (chord <= 0 or chord > .04001 or angle >= .02):
            raise ValueError('Path is not densely sampled at original primitive boundaries.')
        # Each source interval lies within one straight or constant-curvature Reeds–Shepp
        # primitive. Verify the tangent at mid-chord before applying the arc bound.
        delta = math.remainder(b[2] - a[2], math.tau)
        tangent = a[2] + delta / 2
        travel = math.atan2(b[1] - a[1], b[0] - a[0])
        if abs(math.sin(travel - tangent)) > 1e-6:
            raise ValueError('Interval does not follow the supplied tangent headings.')
        arc = chord if angle < 1e-8 else chord * angle / (2 * math.sin(angle/2))
        # A point on the moving body is at most half the translation and half the
        # corner rotation from its nearer endpoint in arc time.
        excursion = (arc + radius * angle) / 2
        lower_bound = min(lower_bound, min(sampled_clearance[i:i+2]) - excursion)
    gears = gear_runs(points)
    directions = [1, *[g['direction'] for g in gears], 1]
    return dict(chord_length_m=arc_length(points), gear_sections=len(gears),
                forward_m=sum(arc_length(g['poses']) for g in gears if g['direction']==1),
                reverse_m=sum(arc_length(g['poses']) for g in gears if g['direction']==-1),
                switches_between_forward_work_strips=sum(a != b for a,b in zip(directions, directions[1:])),
                shortest_single_gear_section_m=min(arc_length(g['poses']) for g in gears),
                min_sampled_envelope_clearance_m=min(sampled_clearance),
                continuous_envelope_clearance_lower_bound_m=lower_bound)


def compare(path):
    safe = load_checked()
    reference = read(APP / 'editions/R09/mission.json')
    candidate = read(path)
    if (candidate['source_plan_revision'] != safe['revision'] or
            candidate['backend_sha256'] != safe['backend_sha256'] or
            candidate['revision'] != canonical({k:v for k,v in candidate.items() if k != 'revision'})):
        raise ValueError('Unbound candidate input.')
    all_segments = safe['cluster']['segments']
    indices = [i for i,s in enumerate(all_segments) if s['id'].startswith(PREFIX)]
    start, end = indices[0], indices[-1]
    if indices != list(range(start, end+1)) or not (all_segments[start-1]['tool_on'] and all_segments[end+1]['tool_on']):
        raise ValueError('Expected one tool-off connection between two original working strips.')
    original = list(all_segments[start]['poses'])
    for section in all_segments[start+1:end+1]:
        if section['poses'][0] != original[-1]:
            raise ValueError('Original change of gear is discontinuous.')
        original.extend(section['poses'][1:])
    proposal = candidate['points']
    if any(math.dist(a[:2], b[:2]) > 1e-7 or yaw_error(a[2], b[2]) > 1e-8
           for a,b in ((proposal[0],original[0]),(proposal[-1],original[-1]))):
        raise ValueError('Original connection poses must be retained.')
    offset = reference['map_offset']
    authority = shape(safe['navigation_prior']).buffer(-reference['footprint_margin_m'])
    args = safe['planning_envelope'], authority, offset
    old, new = measure(original, *args), measure(proposal, *args)
    full = copy.deepcopy(safe)
    cluster = full['cluster']
    replacements = []
    for i, gear in enumerate(gear_runs(proposal)):
        poses = gear['poses']
        length = arc_length(poses)
        segment = dict(all_segments[start], id=f'B1:offline_connection:6:{i}', direction=gear['direction'],
                       poses=poses, poses_world=[[x+offset[0],y+offset[1],h] for x,y,h in poses],
                       length_m=length, nominal_seconds=length/.35)
        replacements.append(segment)
    cluster['segments'][start:end+1] = replacements
    cluster['motion_m'] = sum(s['length_m'] for s in cluster['segments'])
    cluster['tool_off_m'] = cluster['motion_m'] - cluster['work_m']
    # Same estimate used by the approved edition; never substitute it for observed task time.
    cluster['estimated_seconds'] = cluster['motion_m']/.25 + len(cluster['segments'])*5 + 100
    if [s for s in cluster['segments'] if s['tool_on']] != [s for s in all_segments if s['tool_on']]:
        raise ValueError('Work quantities or geometry changed.')
    full_checks = validate(full, reference)
    checks = dict(full_plan_geometry=full_checks['passed'],
                  original_work_polygons_and_strips=True,
                  shorter_path=new['chord_length_m'] < old['chord_length_m'],
                  no_extra_reverse_travel=new['reverse_m'] <= old['reverse_m'],
                  fewer_switches=new['switches_between_forward_work_strips'] < old['switches_between_forward_work_strips'],
                  shortest_gear_no_worse=new['shortest_single_gear_section_m'] >= old['shortest_single_gear_section_m'],
                  envelope_clearance_improved=new['min_sampled_envelope_clearance_m'] > old['min_sampled_envelope_clearance_m'],
                  continuous_padded_envelope_inside_prior=new['continuous_envelope_clearance_lower_bound_m'] > 0)
    return dict(schema='B1-OFFLINE-CONNECTOR-COMPARISON/1', passed=all(checks.values()), checks=checks,
                source_revision=safe['revision'], candidate_sha256=sha(path), checker_sha256=sha(Path(__file__)),
                covered_original_connector_ids=[s['id'] for s in all_segments[start:end+1]],
                original=old, proposed=new, proposed_full_plan_checks=full_checks,
                original_full_plan_estimate_s=safe['cluster']['estimated_seconds'],
                proposed_full_plan_estimate_s=cluster['estimated_seconds'], original_budget_s=cluster['budget_s'],
                scope='Offline geometry only. New tool-off connection retaining both original work poses and all ten work strips. No Nav2/Gazebo driving, collision feedback, localization, completed area or R10 credit.')


def regenerate(source, output):
    """Repeat the selected native search in a scratch directory; preserve the checked-in proposal."""
    if output.exists():
        raise ValueError('Select a new output filename for this independent search.')
    safe = load_checked()
    base = read(source)
    if base['source_plan_revision'] != safe['revision'] or base['backend_sha256'] != safe['backend_sha256']:
        raise ValueError('The search inputs must match the approved edition.')
    ref = read(APP / 'editions/R09/mission.json')
    original = [s for s in safe['cluster']['segments'] if s['id'].startswith(PREFIX)]
    endpoints = [original[0]['poses'][0], original[-1]['poses'][-1]]
    reverse = base.get('reverse_search', False)
    if reverse:
        endpoints.reverse()
    request = dict(start=endpoints[0], goal=endpoints[1], radius=safe['cluster']['turn_radius_m'],
                   footprint=safe['planning_envelope'], seed=base['seed'], seconds=base['per_trial_seconds'],
                   allowed=mapping(shape(safe['navigation_prior']).buffer(-ref['footprint_margin_m'])))
    with TemporaryDirectory() as tmp:
        query, answer = Path(tmp)/'request.json', Path(tmp)/'result.json'
        query.write_text(json.dumps(request))
        subprocess.run([str(APP/'build/r10_safe_route/r10_safe_route'), str(query), str(answer)],
                       stdout=sys.stderr, check=True, timeout=request['seconds']+12)
        result = read(answer)
    if reverse:
        result['points'].reverse()
    generated = {**base, **result}
    del generated['revision']
    generated['revision'] = canonical(generated)
    output.write_text(json.dumps(generated, separators=(',',':'))+'\n')
    return compare(output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, default=ROUTE)
    parser.add_argument('--regenerate', metavar='NEW_FILE', type=Path,
                        help='Rerun the native selected trial and validate the resulting independent proposal.')
    args = parser.parse_args()
    report = regenerate(args.candidate, args.regenerate) if args.regenerate else compare(args.candidate)
    print(json.dumps(report, indent=2, allow_nan=False))
    raise SystemExit(0 if report['passed'] else 2)
