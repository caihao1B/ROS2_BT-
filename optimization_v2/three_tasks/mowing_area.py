#!/usr/bin/env python3
"""Estimate the planar deck sweep of the declared B1 work strips. No simulator or ledger writes."""
import hashlib
import json
import math
import sys
from pathlib import Path

from shapely import union_all
from shapely.affinity import translate
from shapely.geometry import Polygon, box, shape

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'optimization_v2'
EDITIONS = BASE / 'three_tasks/editions'
sys.dont_write_bytecode = True
sys.path.insert(0, str(BASE / 'ws/src/task_platform/scripts'))
from common import canonical
from ledger_core_r09 import footprint


def read(path):
    return json.loads(path.read_text())


def file_hash(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def deck_sweep(poses, tool, erosion=0.):
    """Exact translation of a fixed-heading rectangle over one straight work fragment."""
    x0, y0, h = poses[0]
    co, si = math.cos(h), math.sin(h)
    longitudinal = 0.
    for x, y, angle in poses:
        dx, dy = x - x0, y - y0
        travel = dx * co + dy * si
        if (abs(math.remainder(angle - h, math.tau)) > 1e-8 or
                abs(-dx * si + dy * co) > 1e-7 or travel < longitudinal - 1e-7):
            raise ValueError('The work strip is not straight forward travel at fixed yaw.')
        longitudinal = travel
    offset_x, offset_y, _ = tool['mount_xyz']
    center_x = x0 + co * offset_x - si * offset_y
    center_y = y0 + si * offset_x + co * offset_y
    args = (h, tool['virtual_effective_length_m'], tool['virtual_effective_width_m'], erosion)
    start = footprint(center_x, center_y, *args)
    end = footprint(center_x + co * longitudinal, center_y + si * longitudinal, *args)
    return union_all([start, end]).convex_hull


def compute(segments, targets, contract, map_offset):
    tool = contract['tool']
    margin = (contract['pose_erosion_m'] + contract['ledger_precision_m'] +
              math.hypot(tool['virtual_effective_length_m'], tool['virtual_effective_width_m'])
              * .5 * contract['yaw_erosion_rad'])
    raw = []
    sampled_by_key = {k: [] for k in targets}
    normal = {k: [] for k in targets}
    eroded = {k: [] for k in targets}
    lengths = []
    for segment in segments:
        if segment['tool_on'] != (segment['kind'] == 'COVERAGE'):
            raise ValueError(f"Conflicting tool eligibility in {segment['id']}")
        if not segment['tool_on']:
            continue
        key = segment.get('work_key', segment['id'])
        if not targets[key].equals(Polygon(segment['target_area'])):
            raise ValueError(f'Changed original work target for {key}')
        poses = segment['poses']
        for (x, y, h), world in zip(poses, segment['poses_world'], strict=True):
            if math.dist((x + map_offset[0], y + map_offset[1], h), world) > 1e-7:
                raise ValueError(f'Inconsistent reference frames in {key}')
        # Independent cross-check in Gazebo XY: transform each corner at every stored pose.
        # Samples overlap longitudinally on the current fixed-yaw straight work strips.
        corners = [(tool['mount_xyz'][0] + dx * tool['virtual_effective_length_m'] / 2,
                    tool['mount_xyz'][1] + dy * tool['virtual_effective_width_m'] / 2)
                   for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        sampled = []
        for wx, wy, h in segment['poses_world']:
            co, si = math.cos(h), math.sin(h)
            sampled.append(Polygon([(wx - map_offset[0] + co * cx - si * cy,
                                     wy - map_offset[1] + si * cx + co * cy) for cx, cy in corners]))
        sampled_by_key[key].append(union_all(sampled).intersection(targets[key]))
        full = deck_sweep(poses, tool)
        raw.append(full)
        normal[key].append(full.intersection(targets[key]))
        shrunken = deck_sweep(poses, tool, margin)
        eroded[key].append(shrunken.intersection(targets[key].buffer(-contract['ledger_precision_m'])))
        lengths.append(sum(math.dist(a[:2], b[:2]) for a, b in zip(poses, poses[1:])))
    totals = {k: union_all(parts) for k, parts in normal.items()}
    total = union_all(list(totals.values())).area
    target = union_all(list(targets.values())).area
    sampled = union_all([part for parts in sampled_by_key.values() for part in parts])
    crosscheck_difference = sampled.symmetric_difference(union_all(list(totals.values()))).area
    if crosscheck_difference > 1e-6:
        raise ValueError('Independent discrete world-frame check differs from continuous map-frame sweep.')
    return dict(sampled_world_frame_symmetric_difference_m2=crosscheck_difference, all_segments=len(segments), enabled_fragments=len(lengths),
                disabled_segments=len(segments) - len(lengths), enabled_length_m=sum(lengths),
                full_deck_unclipped_union_m2=union_all(raw).area,
                own_target_clipped_union_m2=total, own_target_unswept_m2=target - total,
                own_target_fraction=total / target,
                eroded_continuous_own_target_union_m2=union_all(
                    [part for parts in eroded.values() for part in parts]).area,
                ledger_footprint_margin_m=margin,
                original_keys={k: dict(fragments=len(normal[k]), target_m2=v.area,
                                        nominal_own_target_m2=totals[k].area) for k, v in targets.items()})


def main():
    paths = dict(approved=EDITIONS / 'R10/mowing_safe_plan.json',
                 reference=EDITIONS / 'R09/mission.json', mission=EDITIONS / 'R10/mowing/mission.json',
                 tool=EDITIONS / 'R10/mowing/tool.json', reference_tool=EDITIONS / 'R09/tool.json',
                 historical=ROOT / 'validation/R10/bounded_B1_result.json')
    safe, reference, mission, tool, old_tool, hist = [read(p) for p in paths.values()]
    for data, key in ((reference, 'version'), (mission, 'version'),
                      (tool, 'tool_revision'), (old_tool, 'tool_revision')):
        if data[key] != canonical({k: v for k, v in data.items() if k != key}):
            raise ValueError(f'Invalid input binding {key}')
    for work, contract in ((mission, tool), (reference, old_tool)):
        if any(work[k] != contract[k] for k in ('map_id', 'map_revision', 'frame_epoch')):
            raise ValueError('Task and tool frames differ.')
        if work['vehicle']['tool']['mount_xyz'] != contract['tool']['mount_xyz']:
            raise ValueError('Task and tool mounts differ.')
    route = safe['cluster']
    original = next(c for c in reference['clusters'] if c['cluster'] == route['cluster'])
    current = next(c for c in mission['clusters'] if c['cluster'] == route['cluster'])
    if (safe['revision'] != canonical({k: v for k, v in safe.items() if k != 'revision'})
            or safe['source_sha256'] != file_hash(paths['reference'])
            or safe['source_version'] != reference['version'] or current != route
            or mission['safe_execution_plan']['revision'] != safe['revision']
            or tool['plan_revision'] != mission['version'] or old_tool['plan_revision'] != reference['version']
            or tool['tool'] != old_tool['tool'] or mission['map_offset'] != reference['map_offset']):
        raise ValueError('Reference, active plan, or tool binding differs.')
    targets = {s['id']: Polygon(s['target_area']) for s in original['segments'] if s['tool_on']}
    target_area = union_all(list(targets.values())).area
    cluster_area = shape(original['area_map']).area
    if (abs(original['planned_cut_m2'] - target_area) > 1e-7
            or abs(route['planned_cut_m2'] - original['planned_cut_m2']) > 1e-7
            or abs(route['planned_fraction'] - original['planned_fraction']) > 1e-9
            or abs(target_area - safe['original_work_target_union_m2']) > 1e-7
            or abs(cluster_area - safe['original_cluster_area_m2']) > 1e-7):
        raise ValueError('Original denominators differ.')
    results = {k: compute(r['segments'], targets, t, mission['map_offset'])
               for k, r, t in [('original_reference', original, old_tool), ('approved_execution', route, tool)]}
    for d in results.values():
        d['fraction_of_original_cluster'] = d['own_target_clipped_union_m2'] / cluster_area
    if abs(results['original_reference']['own_target_clipped_union_m2'] - target_area) > 1e-6:
        raise ValueError('Reference-route sweep does not reproduce the original target union.')
    # The 2D deck is wholly inside the body. Under the existing footprint test, every
    # nominal deck point must therefore lie in the admitted navigation polygon.
    deck_cfg = tool['tool']
    cx, cy, _ = deck_cfg['mount_xyz']
    half_l = deck_cfg['virtual_effective_length_m'] / 2
    half_w = deck_cfg['virtual_effective_width_m'] / 2
    deck = box(cx-half_l, cy-half_w, cx+half_l, cy+half_w)
    if not Polygon(mission['vehicle']['footprint']).covers(deck):
        raise ValueError('Planar deck protrudes beyond the body; the footprint area bound does not apply.')
    target_world = translate(union_all(list(targets.values())), *mission['map_offset'])
    authorized = shape(safe['navigation_prior'])
    authorized_body = authorized.buffer(-mission['footprint_margin_m'])
    slope = shape(safe['known_static_exclusion']['geometry'])
    available = target_world.intersection(authorized_body).area
    bounds = dict(deck_inside_body=True,
                  raw_static_slope_exclusion_overlap_m2=target_world.intersection(slope).area,
                  original_target_outside_pre_slope_authority_m2=target_world.difference(shape(reference['allowed_prior'])).area,
                  original_target_outside_current_authorized_body_area_m2=target_world.difference(authorized_body).area,
                  original_target_inside_current_authorized_body_area_m2=available,
                  area_fraction_ceiling=available/target_world.area,
                  footprint_margin_m=mission['footprint_margin_m'],
                  scope='Planar necessary condition, for every heading. The actual body must fit and the deck lies within the body. Being inside the polygon does not guarantee a collision-free connected route, enough time, or tool feedback.')
    return dict(scope='Nominal offline geometry only. No measured area credit or task acceptance.',
                planar_authority_bound=bounds,
                assumptions=['Planar map coordinates in metres; nominal roll and pitch are zero. Real observer applies the full 3D rotation to the tool mount before projecting to XY.',
                             'Each enabled strip is checked for constant yaw, straight forward travel and matching world/map poses. Its exact continuous rectangular sweep is the convex hull of its endpoint footprints. Tool-off turns and gaps between fragments add no area.',
                             'Each sweep is intersected with its own original job polygon before combining all keys. The optional eroded figure applies ledger footprint and target margins, without discrete feedback timing, eligibility checks, or grid quantization.',
                             'Actual tracking, rejoining and terrain orientation can change the sweep. These estimates are neither measured feedback nor an upper bound on reachable coverage.'],
                input_sha256={str(p.relative_to(ROOT)): file_hash(p) for p in paths.values()},
                approved_revision=safe['revision'], world_minus_map_xy_m=mission['map_offset'],
                deck=dict(mount_xyz_m=tool['tool']['mount_xyz'],
                          effective_length_m=tool['tool']['virtual_effective_length_m'],
                          effective_width_m=tool['tool']['virtual_effective_width_m']),
                original_target_union_m2=target_area, original_cluster_area_m2=cluster_area,
                inherited_planned_cut_m2=route['planned_cut_m2'], inherited_planned_fraction=route['planned_fraction'],
                routes=results, earlier_measured_run=dict(run=hist['run'], confirmed_m2=hist['confirmed_area_m2'],
                    residual_m2=hist['original_target_residual_m2'], completed_segments=hist['completed_segments']))


if __name__ == '__main__':
    print(json.dumps(main(), indent=2, allow_nan=False))
