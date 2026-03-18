#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from import_asset_pair import upsert_asset, ws_root


def parse_args() -> argparse.Namespace:
    ws = ws_root()
    p = argparse.ArgumentParser(description='Import multiple geojson/json pairs with matching basenames')
    p.add_argument('--planner', default='custom')
    p.add_argument('--geojson-dir', required=True)
    p.add_argument('--plan-dir', required=True)
    p.add_argument('--summary-dir', default='')
    p.add_argument('--resolution', type=float, default=0.05)
    p.add_argument('--margin-m', type=float, default=1.0)
    p.add_argument('--overwrite', action='store_true')
    p.add_argument('--asset-index', default=str(ws / 'e1_assets' / 'asset_index.json'))
    return p.parse_args()


def main() -> int:
    args = parse_args()
    ws = ws_root()
    geo_dir = Path(args.geojson_dir).resolve()
    plan_dir = Path(args.plan_dir).resolve()
    summary_dir = Path(args.summary_dir).resolve() if args.summary_dir else None
    if not geo_dir.is_dir() or not plan_dir.is_dir():
        raise SystemExit('[ERROR] geojson-dir / plan-dir not found')
    count = 0
    for geo in sorted(geo_dir.glob('*.geojson')):
        map_id = geo.stem
        plan = plan_dir / f'{map_id}.json'
        if not plan.is_file():
            print(f'[SKIP] plan missing for {map_id}: {plan}')
            continue
        summary = (summary_dir / f'{map_id}.summary.json') if summary_dir and (summary_dir / f'{map_id}.summary.json').is_file() else None
        upsert_asset(
            ws=ws,
            map_id=map_id,
            planner=args.planner,
            geojson_path=geo,
            plan_json_path=plan,
            summary_json_path=summary,
            resolution=float(args.resolution),
            margin_m=float(args.margin_m),
            overwrite=bool(args.overwrite),
            index_path=Path(args.asset_index).resolve(),
        )
        count += 1
        print(f'[OK] imported {map_id}')
    print(f'[DONE] imported {count} assets')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
