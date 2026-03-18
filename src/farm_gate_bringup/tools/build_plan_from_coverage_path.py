#!/usr/bin/env python3
"""Convert a dense coverage_path.json into a segment-level plan_*.json.

This is a lightweight offline utility (no ROS required).

Input format: farm_gate/coverage_path.json
  {
    "frame_id": "map",
    "count": N,
    "poses": [ {"i":..., "x":..., "y":..., "z":..., "qx":..., "qy":..., "qz":..., "qw":...}, ...]
  }

Output format: plan_*.json
  {
    "frame_id": "map",
    "meta": {...},
    "segments": [ {"id": "seg_000", "poses": [...]}, ...]
  }

Segmentation method:
  - accumulate along-path length
  - split when segment length >= target_length_m

"""

import argparse
import json
import math


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def segment_by_length(poses, target_len_m: float):
    segments = []
    cur = []
    cur_len = 0.0

    def dist(a, b):
        return math.hypot(b['x'] - a['x'], b['y'] - a['y'])

    for p in poses:
        if not cur:
            cur.append(p)
            continue
        cur_len += dist(cur[-1], p)
        cur.append(p)
        if cur_len >= target_len_m:
            segments.append((cur, cur_len))
            cur = [p]
            cur_len = 0.0

    if len(cur) > 1:
        segments.append((cur, cur_len))

    return segments


def add_yaw_quat(poses):
    out = []
    for i, p in enumerate(poses):
        p = dict(p)
        if i < len(poses) - 1:
            nxt = poses[i + 1]
            yaw = math.atan2(nxt['y'] - p['y'], nxt['x'] - p['x'])
        elif i > 0:
            prv = poses[i - 1]
            yaw = math.atan2(p['y'] - prv['y'], p['x'] - prv['x'])
        else:
            yaw = 0.0
        qx, qy, qz, qw = quat_from_yaw(yaw)
        p['qx'] = qx
        p['qy'] = qy
        p['qz'] = qz
        p['qw'] = qw
        # Keep z=0 for navigation
        p['z'] = 0.0
        out.append(p)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', required=True, help='coverage_path.json')
    ap.add_argument('--output', required=True, help='plan_*.json')
    ap.add_argument('--target-length-m', type=float, default=50.0)
    ap.add_argument('--min-seg-len-m', type=float, default=1.0)
    ap.add_argument('--min-seg-poses', type=int, default=3)
    args = ap.parse_args()

    raw = json.load(open(args.input))
    raw_poses = raw['poses']

    segs = segment_by_length(raw_poses, args.target_length_m)

    plan = {
        'frame_id': raw.get('frame_id', 'map'),
        'meta': {
            'source_file': args.input,
            'segmentation_method': 'length_threshold',
            'target_length_m': args.target_length_m,
            'min_segment_length_m': args.min_seg_len_m,
            'min_segment_poses': args.min_seg_poses,
            'raw_pose_count': len(raw_poses),
        },
        'segments': [],
    }

    # Merge tiny segments into previous
    for idx, (poses, seg_len) in enumerate(segs):
        poses = add_yaw_quat(poses)
        if plan['segments'] and (seg_len < args.min_seg_len_m or len(poses) < args.min_seg_poses):
            plan['segments'][-1]['poses'].extend(poses)
            plan['segments'][-1].setdefault('meta', {})['merged_tail'] = True
            continue

        plan['segments'].append(
            {
                'id': f'seg_{idx:03d}',
                'kind': 'coverage',
                'frame_id': plan['frame_id'],
                'poses': poses,
                'meta': {
                    'length_m': float(seg_len),
                    'n_poses': int(len(poses)),
                },
            }
        )

    plan['meta']['segment_count'] = len(plan['segments'])

    with open(args.output, 'w') as f:
        json.dump(plan, f, indent=2)

    print(f'Wrote: {args.output}')
    print(f"Segments: {len(plan['segments'])}")


if __name__ == '__main__':
    main()
