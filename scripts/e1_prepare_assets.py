#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from PIL import Image, ImageDraw
import yaml


@dataclass
class MapMeta:
    map_id: str
    density: str = ''
    size_m: float = 0.0
    seed: int = 0
    geojson_path: Path = Path('.')
    preview_path: Optional[Path] = None


@dataclass
class PlanMeta:
    map_id: str
    planner: str
    plan_json: Path
    summary_json: Optional[Path]


def ws_root() -> Path:
    return Path(__file__).resolve().parent.parent


def norm_relpath(p: str) -> str:
    return p.replace('\\', '/').strip()


def read_json(path: Path) -> Any:
    with path.open('r', encoding='utf-8') as f:
        return json.load(f)


def find_maps_root(path: Path) -> Path:
    p = path.resolve()
    candidates = [p]
    if p.is_dir():
        candidates.extend([x for x in p.iterdir() if x.is_dir()])
    for c in candidates:
        if (c / 'geojson').is_dir() and ((c / 'manifest.csv').is_file() or (c / 'manifest.json').is_file()):
            return c
    raise FileNotFoundError(f'cannot find maps_30 root under: {path}')


def find_plans_root(path: Path) -> Path:
    p = path.resolve()
    candidates = [p]
    if p.is_dir():
        candidates.extend([x for x in p.iterdir() if x.is_dir()])
    for c in candidates:
        if (c / 'ros_plan_index.csv').is_file() or (c / 'ros_plan_index.json').is_file():
            return c
    raise FileNotFoundError(f'cannot find thesis plan root under: {path}')


def load_map_manifest(root: Path) -> List[MapMeta]:
    manifest_csv = root / 'manifest.csv'
    rows: List[Dict[str, Any]] = []
    if manifest_csv.is_file():
        with manifest_csv.open('r', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
    else:
        rows = read_json(root / 'manifest.json')

    out: List[MapMeta] = []
    for r in rows:
        map_id = str(r.get('map_id', '')).strip()
        if not map_id:
            continue
        geo = root / norm_relpath(str(r.get('geojson', f'geojson/{map_id}.geojson')))
        preview_raw = str(r.get('preview', '')).strip()
        preview = (root / norm_relpath(preview_raw)) if preview_raw else None
        out.append(
            MapMeta(
                map_id=map_id,
                density=str(r.get('density', '')).strip(),
                size_m=float(r.get('size_m', 0.0) or 0.0),
                seed=int(float(r.get('seed', 0) or 0)),
                geojson_path=geo.resolve(),
                preview_path=preview.resolve() if preview and preview.exists() else None,
            )
        )
    return out


def load_plan_index(root: Path) -> List[PlanMeta]:
    index_csv = root / 'ros_plan_index.csv'
    rows: List[Dict[str, Any]] = []
    if index_csv.is_file():
        with index_csv.open('r', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
    else:
        rows = read_json(root / 'ros_plan_index.json')

    out: List[PlanMeta] = []
    for r in rows:
        map_id = str(r.get('map_id', '')).strip()
        planner = str(r.get('planner', '')).strip()
        plan_rel = norm_relpath(str(r.get('plan_json', '')))
        summary_rel = norm_relpath(str(r.get('summary_json', '')))
        # tolerate both absolute and relative forms
        if 'maps_30_thesis_ready/' in plan_rel:
            plan_rel = plan_rel.split('maps_30_thesis_ready/', 1)[1]
        if 'maps_30_thesis_ready/' in summary_rel:
            summary_rel = summary_rel.split('maps_30_thesis_ready/', 1)[1]
        # sometimes the original export is prefixed with outputs/
        if plan_rel.startswith('outputs/'):
            plan_rel = plan_rel[len('outputs/'):]
        if summary_rel.startswith('outputs/'):
            summary_rel = summary_rel[len('outputs/'):]
        plan_json = (root / plan_rel).resolve()
        summary_json = (root / summary_rel).resolve() if summary_rel else None
        if not map_id or not planner or not plan_json.exists():
            continue
        out.append(PlanMeta(map_id=map_id, planner=planner, plan_json=plan_json, summary_json=summary_json if summary_json and summary_json.exists() else None))
    return out


def collect_polygons(geometry: Dict[str, Any]) -> List[List[Tuple[float, float]]]:
    gtype = geometry.get('type')
    coords = geometry.get('coordinates', [])
    polys: List[List[Tuple[float, float]]] = []
    if gtype == 'Polygon':
        if coords:
            polys.append([(float(x), float(y)) for x, y in coords[0]])
    elif gtype == 'MultiPolygon':
        for poly in coords:
            if poly:
                polys.append([(float(x), float(y)) for x, y in poly[0]])
    return polys


def load_geojson_geometry(path: Path) -> Tuple[List[List[Tuple[float, float]]], List[List[Tuple[float, float]]], Dict[str, Any]]:
    data = read_json(path)
    field_polys: List[List[Tuple[float, float]]] = []
    obstacle_polys: List[List[Tuple[float, float]]] = []
    meta: Dict[str, Any] = {}
    for feat in data.get('features', []):
        props = feat.get('properties', {}) or {}
        kind = str(props.get('kind', '')).strip().lower()
        if kind == 'field':
            field_polys.extend(collect_polygons(feat.get('geometry', {})))
            # stash useful metadata
            for k in ('map_id', 'dataset', 'size_m', 'density', 'seed', 'realized_obstacle_fraction', 'target_obstacle_fraction', 'gate_side', 'gate_pos_ratio', 'gate_width'):
                if k in props:
                    meta[k] = props[k]
        elif kind == 'obstacle':
            obstacle_polys.extend(collect_polygons(feat.get('geometry', {})))
    if not field_polys:
        raise RuntimeError(f'no field polygon found in {path}')
    return field_polys, obstacle_polys, meta


def world_to_image_point(x: float, y: float, origin_x: float, origin_y: float, resolution: float, height_px: int) -> Tuple[float, float]:
    px = (x - origin_x) / resolution
    py = height_px - ((y - origin_y) / resolution)
    return (px, py)


def rasterize_geojson_to_map(
    geojson_path: Path,
    out_map_yaml: Path,
    out_preview_png: Optional[Path],
    resolution: float,
    margin_m: float,
) -> Dict[str, Any]:
    field_polys, obstacle_polys, meta = load_geojson_geometry(geojson_path)

    xs = [p[0] for poly in field_polys for p in poly]
    ys = [p[1] for poly in field_polys for p in poly]
    minx = min(xs) - margin_m
    miny = min(ys) - margin_m
    maxx = max(xs) + margin_m
    maxy = max(ys) + margin_m

    width_px = max(32, int(math.ceil((maxx - minx) / resolution)))
    height_px = max(32, int(math.ceil((maxy - miny) / resolution)))

    img = Image.new('L', (width_px, height_px), 0)
    draw = ImageDraw.Draw(img)

    for poly in field_polys:
        pts = [world_to_image_point(x, y, minx, miny, resolution, height_px) for x, y in poly]
        draw.polygon(pts, fill=254)

    for poly in obstacle_polys:
        pts = [world_to_image_point(x, y, minx, miny, resolution, height_px) for x, y in poly]
        draw.polygon(pts, fill=0)

    out_map_yaml.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = out_map_yaml.with_suffix('.pgm')
    img.save(pgm_path)

    map_yaml = {
        'image': pgm_path.name,
        'mode': 'trinary',
        'resolution': float(resolution),
        'origin': [float(minx), float(miny), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    with out_map_yaml.open('w', encoding='utf-8') as f:
        yaml.safe_dump(map_yaml, f, sort_keys=False, allow_unicode=True)

    if out_preview_png:
        out_preview_png.parent.mkdir(parents=True, exist_ok=True)
        # upscale small maps for visual checks
        scale = 2 if max(width_px, height_px) < 1024 else 1
        try:
            resample_nn = Image.Resampling.NEAREST
        except Exception:
            resample_nn = Image.NEAREST
        prev = img.resize((width_px * scale, height_px * scale), resample=resample_nn)
        prev.save(out_preview_png)

    return {
        'map_yaml': str(out_map_yaml.resolve()),
        'map_pgm': str(pgm_path.resolve()),
        'origin': [float(minx), float(miny), 0.0],
        'resolution': float(resolution),
        'width_px': int(width_px),
        'height_px': int(height_px),
        'field_meta': meta,
    }


def generate_collision_model(map_yaml: Path, out_model_root: Path, model_name: str) -> Path:
    ws = ws_root()
    script = ws / 'scripts' / 'gen_map_collision_model.py'
    out_model_root.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            'python3', str(script),
            '--map', str(map_yaml),
            '--model-name', model_name,
            '--out-model-dir', str(out_model_root),
            '--wall-height', '1.0',
            '--with-visual', '1',
        ],
        check=True,
    )
    return (out_model_root / model_name).resolve()


def write_world_file(world_path: Path, model_name: str, origin_xyzrpy: List[float]) -> None:
    ox, oy, oyaw = float(origin_xyzrpy[0]), float(origin_xyzrpy[1]), float(origin_xyzrpy[2])
    text = f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
  <world name=\"e1_static_world_{model_name}\">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <include>
      <uri>model://{model_name}</uri>
      <pose>{ox:.6f} {oy:.6f} 0 0 0 {oyaw:.6f}</pose>
    </include>

    <plugin name=\"gazebo_ros_state\" filename=\"libgazebo_ros_state.so\">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
    </plugin>
  </world>
</sdf>
"""
    world_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_text(text, encoding='utf-8')


def make_asset_rows(
    maps_root: Path,
    plans_root: Optional[Path],
    out_root: Path,
    resolution: float,
    margin_m: float,
    overwrite: bool,
    selected_map_ids: Optional[set[str]] = None,
) -> List[Dict[str, Any]]:
    maps = load_map_manifest(maps_root)
    if selected_map_ids:
        maps = [m for m in maps if m.map_id in selected_map_ids]
    plans: List[PlanMeta] = load_plan_index(plans_root) if plans_root else []
    plans_by_map: Dict[str, List[PlanMeta]] = {}
    for p in plans:
        plans_by_map.setdefault(p.map_id, []).append(p)

    out_maps = out_root / 'maps'
    out_worlds = out_root / 'worlds'
    out_models = out_root / 'models'
    out_previews = out_root / 'previews'

    assets: List[Dict[str, Any]] = []
    for m in maps:
        map_yaml = out_maps / f'{m.map_id}.yaml'
        prev_png = out_previews / f'{m.map_id}.png'
        model_name = f'farm_map_collision_{m.map_id}'
        world_file = out_worlds / f'{m.map_id}.world'
        model_dir = out_models / model_name

        if overwrite or (not map_yaml.exists()):
            map_info = rasterize_geojson_to_map(m.geojson_path, map_yaml, prev_png, resolution=resolution, margin_m=margin_m)
        else:
            with map_yaml.open('r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
            map_info = {
                'map_yaml': str(map_yaml.resolve()),
                'map_pgm': str(map_yaml.with_suffix('.pgm').resolve()),
                'origin': list(cfg.get('origin', [0.0, 0.0, 0.0])),
                'resolution': float(cfg.get('resolution', resolution)),
                'field_meta': {},
            }

        if overwrite or (not model_dir.exists()):
            generate_collision_model(map_yaml, out_models, model_name)
        if overwrite or (not world_file.exists()):
            write_world_file(world_file, model_name, map_info['origin'])

        shared = {
            'map_id': m.map_id,
            'density': m.density,
            'size_m': m.size_m,
            'seed': m.seed,
            'geojson_path': str(m.geojson_path.resolve()),
            'preview_path': str((m.preview_path or prev_png).resolve()) if (m.preview_path or prev_png) else '',
            'map_yaml': str(map_yaml.resolve()),
            'map_pgm': str(map_yaml.with_suffix('.pgm').resolve()),
            'world_file': str(world_file.resolve()),
            'model_name': model_name,
            'model_root': str(out_models.resolve()),
            'model_dir': str(model_dir.resolve()),
            'map_resolution': map_info['resolution'],
            'map_origin': map_info['origin'],
            'field_meta': map_info.get('field_meta', {}),
        }

        if plans_by_map.get(m.map_id):
            for p in sorted(plans_by_map[m.map_id], key=lambda x: x.planner):
                summary_path = str(p.summary_json.resolve()) if p.summary_json else ''
                asset = dict(shared)
                asset.update({
                    'planner': p.planner,
                    'plan_json': str(p.plan_json.resolve()),
                    'summary_json': summary_path,
                })
                assets.append(asset)
        else:
            asset = dict(shared)
            asset.update({'planner': 'map_only', 'plan_json': '', 'summary_json': ''})
            assets.append(asset)

    return assets


def write_index(out_root: Path, assets: List[Dict[str, Any]]) -> None:
    out_root.mkdir(parents=True, exist_ok=True)
    index_json = out_root / 'asset_index.json'
    index_csv = out_root / 'asset_index.csv'
    ws = ws_root()

    def _rel(v: Any) -> Any:
        if not isinstance(v, str) or not v:
            return v
        try:
            p = Path(v)
            if p.is_absolute():
                return p.resolve().relative_to(ws).as_posix()
        except Exception:
            return v
        return v

    cooked: List[Dict[str, Any]] = []
    for a in assets:
        row = dict(a)
        for key in ('geojson_path', 'plan_json', 'summary_json', 'map_yaml', 'map_pgm', 'world_file', 'model_root', 'model_dir', 'preview_path'):
            if key in row:
                row[key] = _rel(row.get(key))
        cooked.append(row)

    import datetime as _dt
    payload = {
        'generated_at': _dt.datetime.now(_dt.timezone.utc).isoformat(),
        'assets': cooked,
    }
    with index_json.open('w', encoding='utf-8') as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    fieldnames = [
        'map_id', 'planner', 'density', 'size_m', 'seed', 'geojson_path', 'plan_json', 'summary_json',
        'map_yaml', 'map_pgm', 'world_file', 'model_name', 'model_root', 'model_dir',
        'map_resolution', 'map_origin', 'preview_path',
    ]
    with index_csv.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for a in cooked:
            row = {k: a.get(k, '') for k in fieldnames}
            row['map_origin'] = json.dumps(a.get('map_origin', []), ensure_ascii=False)
            w.writerow(row)


def parse_args() -> argparse.Namespace:
    ws = ws_root()
    p = argparse.ArgumentParser(description='Prepare E1 static benchmark assets from geojson + thesis plans')
    p.add_argument('--maps-root', default=str(ws / 'datasets' / 'raw' / 'maps_30'))
    p.add_argument('--plans-root', default=str(ws / 'datasets' / 'raw' / 'maps_30_thesis_ready'))
    p.add_argument('--out-root', default=str(ws / 'e1_assets'))
    p.add_argument('--resolution', type=float, default=0.05)
    p.add_argument('--margin-m', type=float, default=1.0)
    p.add_argument('--overwrite', action='store_true')
    p.add_argument('--limit', type=int, default=0, help='debug: only prepare first N map ids')
    p.add_argument('--map-id', action='append', default=[], help='prepare only specific map_id (repeatable)')
    return p.parse_args()


def main() -> int:
    args = parse_args()
    maps_root = find_maps_root(Path(args.maps_root))
    plans_root_path: Optional[Path] = None
    try:
        plans_root_path = find_plans_root(Path(args.plans_root)) if args.plans_root else None
    except FileNotFoundError:
        plans_root_path = None

    # Prepare only the requested subset when filters are supplied.
    all_maps = load_map_manifest(maps_root)
    wanted_ids = [str(x) for x in args.map_id] if args.map_id else []
    if wanted_ids:
        all_maps = [m for m in all_maps if m.map_id in set(wanted_ids)]
    if args.limit and args.limit > 0:
        all_maps = all_maps[: int(args.limit)]

    # Temporarily write filtered manifests into memory by monkey-patching helper results.
    # Simpler than duplicating the full generation logic.
    # Rebuild lightweight temporary roots not needed; just use selected ids after generation function returns.
    selected_ids = {m.map_id for m in all_maps}
    assets = make_asset_rows(
        maps_root=maps_root,
        plans_root=plans_root_path,
        out_root=Path(args.out_root).resolve(),
        resolution=float(args.resolution),
        margin_m=float(args.margin_m),
        overwrite=bool(args.overwrite),
        selected_map_ids=selected_ids or None,
    )

    write_index(Path(args.out_root).resolve(), assets)
    print(f'[E1] prepared assets: {len(assets)} entries -> {Path(args.out_root).resolve()/"asset_index.json"}')
    if plans_root_path is None:
        print('[E1] plan index not found; generated map/world assets only.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
