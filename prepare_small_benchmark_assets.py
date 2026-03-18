#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, io, json, re, shutil, sys, zipfile
from pathlib import Path
from typing import Dict, List, Tuple


def parse_args():
    ap = argparse.ArgumentParser(description='Prepare small benchmark map/plan assets for coverage_ws_farm_gate_e1_static_bt_workspace')
    ap.add_argument('--ws', required=True, help='Workspace root (coverage_ws_farm_gate_e1_static_bt_workspace)')
    ap.add_argument('--geojson-zip', required=True)
    ap.add_argument('--json-zip', required=True)
    ap.add_argument('--dataset-name', default='maps_small_benchmark')
    ap.add_argument('--planner-name', default='f2c_geom')
    ap.add_argument('--overwrite', action='store_true')
    return ap.parse_args()


def ensure_empty_dir(path: Path, overwrite: bool):
    if path.exists():
        if not overwrite:
            raise SystemExit(f'[ERROR] Destination exists: {path} (use --overwrite)')
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def density_sort_key(d: str) -> int:
    order = {'low': 0, 'medium': 1, 'high': 2}
    return order.get(d, 99)


def map_sort_key(map_id: str) -> Tuple[int, str, int, str]:
    m = re.match(r'([a-zA-Z]+)_(\d+)_', map_id)
    if m:
        density = m.group(1).lower()
        idx = int(m.group(2))
        return (density_sort_key(density), density, idx, map_id)
    return (99, '', 9999, map_id)


def read_geojson_props_from_zip(zpath: Path) -> Dict[str, Dict]:
    props_by_map: Dict[str, Dict] = {}
    with zipfile.ZipFile(zpath) as zf:
        for name in zf.namelist():
            if not name.endswith('.geojson'):
                continue
            with zf.open(name) as f:
                data = json.load(io.TextIOWrapper(f, encoding='utf-8'))
            field = None
            for feat in data.get('features', []):
                props = feat.get('properties', {}) or {}
                if str(props.get('kind', '')).lower() == 'field':
                    field = props
                    break
            if not field:
                continue
            map_id = str(field.get('map_id', Path(name).stem)).strip()
            props_by_map[map_id] = dict(field)
    return props_by_map


def read_plan_meta_from_zip(zpath: Path) -> Dict[str, Dict]:
    meta_by_map: Dict[str, Dict] = {}
    with zipfile.ZipFile(zpath) as zf:
        for name in zf.namelist():
            if not name.endswith('.json'):
                continue
            with zf.open(name) as f:
                data = json.load(io.TextIOWrapper(f, encoding='utf-8'))
            map_id = Path(name).stem
            fm = data.get('field_meta', {}) or {}
            meta_by_map[map_id] = {
                'planner': str(fm.get('planner', '')).strip(),
                'row_count': fm.get('row_count', 0),
                'turn_count': fm.get('turn_count', 0),
                'driveable_area_m2': fm.get('driveable_area_m2', 0),
                'coverage_area_m2': fm.get('coverage_area_m2', 0),
                'segment_count': len(data.get('segments', [])),
            }
    return meta_by_map


def main():
    args = parse_args()
    ws = Path(args.ws).resolve()
    geozip = Path(args.geojson_zip).resolve()
    jsonzip = Path(args.json_zip).resolve()
    if not ws.is_dir():
        raise SystemExit(f'[ERROR] Workspace not found: {ws}')
    if not geozip.is_file() or not jsonzip.is_file():
        raise SystemExit('[ERROR] Input zip not found')

    raw = ws / 'datasets' / 'raw'
    maps_root = raw / 'maps_30'
    plans_root = raw / 'maps_30_thesis_ready'
    ensure_empty_dir(maps_root, args.overwrite)
    ensure_empty_dir(plans_root, args.overwrite)

    # unzip raw contents
    with zipfile.ZipFile(geozip) as zf:
        zf.extractall(maps_root)
    with zipfile.ZipFile(jsonzip) as zf:
        zf.extractall(plans_root)

    # normalize expected subdirs
    geo_dir = maps_root / 'geojson'
    json_dir = plans_root / 'f2c_geom' / 'json'
    summary_dir = plans_root / 'f2c_geom' / 'summary'
    json_dir.mkdir(parents=True, exist_ok=True)
    summary_dir.mkdir(parents=True, exist_ok=True)

    # move json/*.json -> f2c_geom/json/
    src_json_dir = plans_root / 'json'
    if src_json_dir.is_dir():
        for p in src_json_dir.glob('*.json'):
            shutil.move(str(p), str(json_dir / p.name))
        shutil.rmtree(src_json_dir)

    # gather metadata
    geo_props = read_geojson_props_from_zip(geozip)
    plan_meta = read_plan_meta_from_zip(jsonzip)
    map_ids = sorted(set(geo_props) | set(plan_meta), key=map_sort_key)
    if not map_ids:
        raise SystemExit('[ERROR] No maps/plans discovered in zips')

    # manifest.csv
    manifest_rows: List[Dict[str, str]] = []
    for map_id in map_ids:
        props = geo_props.get(map_id, {})
        density = str(props.get('density', '')).strip()
        size_m = props.get('size_m', 0)
        seed = props.get('seed', 0)
        layout_mode = str(props.get('layout_mode', '')).strip()
        geo_rel = f'geojson/{map_id}.geojson'
        manifest_rows.append({
            'map_id': map_id,
            'density': density,
            'size_m': f'{float(size_m):.3f}' if size_m not in ('', None) else '0.0',
            'seed': str(int(seed) if str(seed) else 0),
            'layout_mode': layout_mode,
            'geojson': geo_rel,
        })
    with (maps_root / 'manifest.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=['map_id', 'density', 'size_m', 'seed', 'layout_mode', 'geojson'])
        w.writeheader()
        w.writerows(manifest_rows)

    # ros_plan_index.csv + lightweight plan summaries for aggregation
    plan_rows: List[Dict[str, str]] = []
    for map_id in map_ids:
        plan_path = json_dir / f'{map_id}.json'
        if not plan_path.exists():
            continue
        data = json.loads(plan_path.read_text(encoding='utf-8'))
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
        summary_rel = f'f2c_geom/summary/{map_id}.summary.json'
        summary = {
            'map_id': map_id,
            'planner': args.planner_name,
            'path_length_m': total_len,
            'segment_count': len(segs),
            'row_count': row_count,
            'turn_count': turn_count,
        }
        (summary_dir / f'{map_id}.summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        plan_rows.append({
            'map_id': map_id,
            'planner': args.planner_name,
            'plan_json': f'f2c_geom/json/{map_id}.json',
            'summary_json': summary_rel,
        })
    with (plans_root / 'ros_plan_index.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=['map_id', 'planner', 'plan_json', 'summary_json'])
        w.writeheader()
        w.writerows(plan_rows)

    # write meta sidecar for user sanity check
    summary = {
        'dataset_name': args.dataset_name,
        'planner_name': args.planner_name,
        'map_count': len(manifest_rows),
        'plan_count': len(plan_rows),
        'maps_root': str(maps_root),
        'plans_root': str(plans_root),
        'map_ids': map_ids,
        'plan_meta': plan_meta,
    }
    with (raw / 'small_benchmark_import_summary.json').open('w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print('[OK] Imported maps/plans into workspace raw dataset area')
    print(f'  maps_root : {maps_root}')
    print(f'  plans_root: {plans_root}')
    print(f'  map_count : {len(manifest_rows)}')
    print(f'  plan_count: {len(plan_rows)}')
    print('[NEXT] python3 scripts/e1_prepare_assets.py --overwrite')

if __name__ == '__main__':
    main()
