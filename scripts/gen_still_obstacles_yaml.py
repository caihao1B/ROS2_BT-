#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

def parse_segments(s: str):
    s = s.strip()
    if not s:
        return [0]
    if s.startswith('['):
        vals = json.loads(s)
        return [int(v) for v in vals]
    return [int(v) for v in s.split(',') if str(v).strip()]

def load_plan(path: Path):
    with path.open('r', encoding='utf-8') as f:
        return json.load(f)

def seg_pose_points(seg):
    poses = list(seg.get('poses') or [])
    pts = []
    for p in poses:
        try:
            pts.append((float(p['x']), float(p['y'])))
        except Exception:
            continue
    return pts

def point_at_fraction(pts, frac: float):
    if not pts:
        raise ValueError('empty segment poses')
    if len(pts) == 1:
        return pts[0]
    frac = max(0.0, min(1.0, float(frac)))
    lens = []
    total = 0.0
    for a,b in zip(pts[:-1], pts[1:]):
        d = math.hypot(b[0]-a[0], b[1]-a[1])
        lens.append(d)
        total += d
    if total <= 1e-9:
        return pts[0]
    target = frac * total
    acc = 0.0
    for (a,b,d) in zip(pts[:-1], pts[1:], lens):
        if acc + d >= target:
            t = 0.0 if d <= 1e-9 else (target - acc) / d
            return (a[0] + t * (b[0]-a[0]), a[1] + t * (b[1]-a[1]))
        acc += d
    return pts[-1]

def yaml_quote(s: str) -> str:
    return '"' + s.replace('"', '\"') + '"'

def main():
    ap = argparse.ArgumentParser(description='Generate YAML scenario with persistent still obstacles anchored on plan segments.')
    ap.add_argument('--plan', required=True)
    ap.add_argument('--segments', required=True, help='JSON list like [5,6,7] or csv 5,6,7')
    ap.add_argument('--frac', type=float, default=0.5)
    ap.add_argument('--size-profile', default='human')
    ap.add_argument('--name-prefix', default='still')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    plan = load_plan(Path(args.plan))
    segs = list(plan.get('segments') or [])
    ids = parse_segments(args.segments)
    obs = []
    for k, sid in enumerate(ids):
        if sid < 0 or sid >= len(segs):
            continue
        pts = seg_pose_points(segs[sid])
        if not pts:
            continue
        x, y = point_at_fraction(pts, args.frac)
        obs.append({
            'name': f'{args.name_prefix}_{sid:02d}',
            'kind': 'static',
            'behavior': 'static',
            'size_profile': args.size_profile,
            'start': [round(x, 4), round(y, 4)],
            'end': [round(x, 4), round(y, 4)],
            'speed_mps': 0.0,
            'loop': 'stay',
            'pause_start_sec': [0.0, 0.0],
            'pause_end_sec': [9999.0, 9999.0],
        })

    if not obs:
        raise SystemExit('no valid obstacle anchors produced')

    out = []
    out.append('default_speed_mps: 0.0')
    out.append('default_loop: stay')
    out.append('default_pause_sec: [0.0, 0.0]')
    out.append('obstacles:')
    for it in obs:
        out.append(f"  - name: {yaml_quote(it['name'])}")
        out.append(f"    kind: {it['kind']}")
        out.append(f"    behavior: {it['behavior']}")
        out.append(f"    size_profile: {it['size_profile']}")
        out.append(f"    start: [{it['start'][0]:.4f}, {it['start'][1]:.4f}]")
        out.append(f"    end: [{it['end'][0]:.4f}, {it['end'][1]:.4f}]")
        out.append(f"    speed_mps: {it['speed_mps']:.2f}")
        out.append(f"    loop: {it['loop']}")
        out.append(f"    pause_start_sec: [{it['pause_start_sec'][0]:.1f}, {it['pause_start_sec'][1]:.1f}]")
        out.append(f"    pause_end_sec: [{it['pause_end_sec'][0]:.1f}, {it['pause_end_sec'][1]:.1f}]")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text('\n'.join(out) + '\n', encoding='utf-8')
    print(str(out_path))

if __name__ == '__main__':
    main()
