#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path
from typing import Any, Dict, List, Optional

from e1_prepare_assets import load_geojson_geometry, rasterize_geojson_to_map, generate_collision_model, write_world_file, read_json  # type: ignore


def ws_root() -> Path:
    return Path(__file__).resolve().parent.parent


def ensure_rel(ws: Path, path: Path) -> str:
    try:
        return path.resolve().relative_to(ws.resolve()).as_posix()
    except Exception:
        return path.resolve().as_posix()


def load_index(index_path: Path) -> List[Dict[str, Any]]:
    if not index_path.is_file():
        return []
    data = json.loads(index_path.read_text(encoding='utf-8'))
    return data['assets'] if isinstance(data, dict) else list(data)


def write_index(ws: Path, assets: List[Dict[str, Any]], index_path: Path) -> None:
    assets = sorted(assets, key=lambda a: (str(a.get('map_id', '')), str(a.get('planner', ''))))
    import datetime as _dt
    payload = {'generated_at': _dt.datetime.now(_dt.timezone.utc).isoformat(), 'assets': assets}
    index_path.parent.mkdir(parents=True, exist_ok=True)
    index_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    csv_path = index_path.with_suffix('.csv')
    fieldnames = [
        'map_id', 'planner', 'density', 'size_m', 'seed', 'geojson_path', 'plan_json', 'summary_json',
        'map_yaml', 'map_pgm', 'world_file', 'model_name', 'model_root', 'model_dir',
        'map_resolution', 'map_origin', 'preview_path',
    ]
    with csv_path.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for a in assets:
            row = {k: a.get(k, '') for k in fieldnames}
            row['map_origin'] = json.dumps(a.get('map_origin', []), ensure_ascii=False)
            w.writerow(row)


def build_summary(plan_json: Path, out_summary: Path, map_id: str, planner: str) -> Path:
    data = read_json(plan_json)
    segs = data.get('segments', [])
    total_len = 0.0
    row_count = 0
    turn_count = 0
    for s in segs:
        meta = s.get('meta', {}) or {}
        total_len += float(meta.get('length_m', 0.0) or 0.0)
        kind = str(s.get('kind', '')).strip().lower()
        if kind == 'row':
            row_count += 1
        elif kind == 'turn':
            turn_count += 1
    payload = {
        'map_id': map_id,
        'planner': planner,
        'path_length_m': total_len,
        'segment_count': len(segs),
        'row_count': row_count,
        'turn_count': turn_count,
    }
    out_summary.parent.mkdir(parents=True, exist_ok=True)
    out_summary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    return out_summary


def upsert_asset(
    ws: Path,
    map_id: str,
    planner: str,
    geojson_path: Path,
    plan_json_path: Path,
    summary_json_path: Optional[Path],
    resolution: float,
    margin_m: float,
    overwrite: bool,
    index_path: Optional[Path] = None,
) -> Dict[str, Any]:
    index_path = index_path or (ws / 'e1_assets' / 'asset_index.json')
    imported_geo = ws / 'datasets' / 'imported' / 'geojson' / f'{map_id}.geojson'
    imported_plan = ws / 'datasets' / 'imported' / planner / 'json' / f'{map_id}.json'
    imported_summary = ws / 'datasets' / 'imported' / planner / 'summary' / f'{map_id}.summary.json'
    imported_geo.parent.mkdir(parents=True, exist_ok=True)
    imported_plan.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(geojson_path, imported_geo)
    shutil.copy2(plan_json_path, imported_plan)
    if summary_json_path and summary_json_path.is_file():
        imported_summary.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(summary_json_path, imported_summary)
    else:
        build_summary(imported_plan, imported_summary, map_id=map_id, planner=planner)

    map_yaml = ws / 'e1_assets' / 'maps' / f'{map_id}.yaml'
    preview_png = ws / 'e1_assets' / 'previews' / f'{map_id}.png'
    model_name = f'farm_map_collision_{map_id}'
    world_file = ws / 'e1_assets' / 'worlds' / f'{map_id}.world'

    map_yaml.parent.mkdir(parents=True, exist_ok=True)
    preview_png.parent.mkdir(parents=True, exist_ok=True)
    (ws / 'e1_assets' / 'worlds').mkdir(parents=True, exist_ok=True)
    (ws / 'e1_assets' / 'models').mkdir(parents=True, exist_ok=True)

    if overwrite or not map_yaml.exists():
        map_info = rasterize_geojson_to_map(imported_geo, map_yaml, preview_png, resolution=resolution, margin_m=margin_m)
    else:
        cfg = __import__('yaml').safe_load(map_yaml.read_text(encoding='utf-8'))
        map_info = {
            'map_yaml': str(map_yaml.resolve()),
            'map_pgm': str(map_yaml.with_suffix('.pgm').resolve()),
            'origin': list(cfg.get('origin', [0.0, 0.0, 0.0])),
            'resolution': float(cfg.get('resolution', resolution)),
            'field_meta': {},
        }
    if overwrite or not (ws / 'e1_assets' / 'models' / model_name).exists():
        generate_collision_model(map_yaml, ws / 'e1_assets' / 'models', model_name)
    if overwrite or not world_file.exists():
        write_world_file(world_file, model_name, map_info['origin'])

    _, _, meta = load_geojson_geometry(imported_geo)
    density = str(meta.get('density', '')).strip()
    size_m = float(meta.get('size_m', 0.0) or 0.0)
    seed = int(float(meta.get('seed', 0) or 0))

    asset = {
        'map_id': map_id,
        'planner': planner,
        'density': density,
        'size_m': size_m,
        'seed': seed,
        'geojson_path': ensure_rel(ws, imported_geo),
        'plan_json': ensure_rel(ws, imported_plan),
        'summary_json': ensure_rel(ws, imported_summary),
        'map_yaml': ensure_rel(ws, map_yaml),
        'map_pgm': ensure_rel(ws, map_yaml.with_suffix('.pgm')),
        'world_file': ensure_rel(ws, world_file),
        'model_name': model_name,
        'model_root': 'e1_assets/models',
        'model_dir': f'e1_assets/models/{model_name}',
        'map_resolution': map_info['resolution'],
        'map_origin': map_info['origin'],
        'preview_path': ensure_rel(ws, preview_png),
        'field_meta': map_info.get('field_meta', {}),
    }

    assets = load_index(index_path)
    assets = [a for a in assets if not (str(a.get('map_id', '')) == map_id and str(a.get('planner', '')) == planner)]
    assets.append(asset)
    write_index(ws, assets, index_path)
    return asset


def parse_args() -> argparse.Namespace:
    ws = ws_root()
    p = argparse.ArgumentParser(description='Import one geojson + plan json pair into the clean experiment workspace')
    p.add_argument('--map-id', required=True)
    p.add_argument('--planner', default='custom')
    p.add_argument('--geojson', required=True)
    p.add_argument('--plan-json', required=True)
    p.add_argument('--summary-json', default='')
    p.add_argument('--resolution', type=float, default=0.05)
    p.add_argument('--margin-m', type=float, default=1.0)
    p.add_argument('--overwrite', action='store_true')
    p.add_argument('--asset-index', default=str(ws / 'e1_assets' / 'asset_index.json'))
    return p.parse_args()


def main() -> int:
    args = parse_args()
    ws = ws_root()
    asset = upsert_asset(
        ws=ws,
        map_id=args.map_id,
        planner=args.planner,
        geojson_path=Path(args.geojson).resolve(),
        plan_json_path=Path(args.plan_json).resolve(),
        summary_json_path=Path(args.summary_json).resolve() if args.summary_json else None,
        resolution=float(args.resolution),
        margin_m=float(args.margin_m),
        overwrite=bool(args.overwrite),
        index_path=Path(args.asset_index).resolve(),
    )
    print('[OK] asset imported')
    print(json.dumps(asset, ensure_ascii=False, indent=2))
    print('[NEXT] Resolve it with:')
    print(f"  python3 {ws/'scripts'/'e1_common.py'} resolve --map-id {args.map_id} --planner {args.planner}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
