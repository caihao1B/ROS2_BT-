#!/usr/bin/env python3
"""Common helpers for portable experiment asset resolution.

Differences from the older workspace:
- Asset index paths may be relative to the workspace root.
- Resolved output always returns absolute paths.
- Model root is resolved from the asset entry when present.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

JSON = Dict[str, Any]


CORE_PATH_FIELDS = [
    'plan_json', 'summary_json', 'map_yaml', 'map_pgm', 'world_file',
    'geojson_path', 'preview_path', 'model_root', 'model_dir'
]


def ws_root_from_here() -> Path:
    return Path(__file__).resolve().parent.parent


def normalize_relpath(p: str) -> str:
    return p.replace('\\', '/').strip()


def ensure_abs(root: Path, p: Optional[str]) -> Optional[Path]:
    if not p:
        return None
    s = normalize_relpath(str(p))
    path = Path(s)
    if path.is_absolute():
        return path
    return (root / path).resolve()


def load_json(path: Path) -> Any:
    with path.open('r', encoding='utf-8') as f:
        return json.load(f)


def load_asset_index(index_path: Optional[Path] = None) -> List[JSON]:
    ws = ws_root_from_here()
    p = index_path or (ws / 'e1_assets' / 'asset_index.json')
    if not p.is_file():
        raise FileNotFoundError(f'asset index not found: {p}')
    data = load_json(p)
    if isinstance(data, dict):
        assets = data.get('assets', [])
        if isinstance(assets, list):
            return assets
        raise RuntimeError(f'invalid asset index format: {p}')
    if isinstance(data, list):
        return data
    raise RuntimeError(f'invalid asset index format: {p}')


class AssetNotFound(RuntimeError):
    pass


def absolutize_asset_fields(asset: JSON, root: Optional[Path] = None) -> JSON:
    ws = root or ws_root_from_here()
    out = dict(asset)
    for key in CORE_PATH_FIELDS:
        if key in out and out.get(key):
            out[key] = str(ensure_abs(ws, str(out[key])))
    if not out.get('model_root'):
        out['model_root'] = str((ws / 'e1_assets' / 'models').resolve())
    return out


def resolve_asset(
    map_id: Optional[str] = None,
    planner: Optional[str] = None,
    asset_index: Optional[Path] = None,
    plan_file: Optional[str] = None,
    map_yaml: Optional[str] = None,
    world_file: Optional[str] = None,
) -> JSON:
    ws = ws_root_from_here()

    explicit_plan = ensure_abs(ws, plan_file)
    explicit_map = ensure_abs(ws, map_yaml)
    explicit_world = ensure_abs(ws, world_file)

    if explicit_plan or explicit_map or explicit_world:
        return {
            'map_id': map_id or (explicit_plan.stem if explicit_plan else (explicit_map.stem if explicit_map else 'custom')),
            'planner': planner or 'custom',
            'plan_json': str(explicit_plan) if explicit_plan else '',
            'map_yaml': str(explicit_map) if explicit_map else '',
            'world_file': str(explicit_world) if explicit_world else '',
            'model_root': str((ws / 'e1_assets' / 'models').resolve()),
            'source': 'explicit',
        }

    if not map_id:
        raise AssetNotFound('map_id is required when no explicit file paths are given')

    assets = load_asset_index(asset_index)
    planner_norm = planner.strip() if planner else None
    candidates = [a for a in assets if str(a.get('map_id', '')) == str(map_id)]
    if planner_norm:
        candidates = [a for a in candidates if str(a.get('planner', '')) == planner_norm]

    if not candidates:
        want = f'map_id={map_id}, planner={planner_norm or "*"}'
        raise AssetNotFound(f'asset not found in index: {want}')

    def score(a: JSON) -> Tuple[int, int]:
        ok = int(bool(a.get('plan_json'))) + int(bool(a.get('map_yaml'))) + int(bool(a.get('world_file')))
        return (ok, -len(str(a.get('planner', ''))))

    best = sorted(candidates, key=score, reverse=True)[0]
    return absolutize_asset_fields(best, ws)


def cli_resolve(args: argparse.Namespace) -> int:
    asset = resolve_asset(
        map_id=args.map_id,
        planner=args.planner,
        asset_index=Path(args.asset_index).resolve() if args.asset_index else None,
        plan_file=args.plan_file,
        map_yaml=args.map_yaml,
        world_file=args.world_file,
    )
    field = args.field
    if field == 'all':
        print(json.dumps(asset, ensure_ascii=False))
        return 0
    if field not in asset:
        raise SystemExit(f'field not found in asset: {field}')
    print(str(asset.get(field, '')))
    return 0


def cli_list(args: argparse.Namespace) -> int:
    assets = [absolutize_asset_fields(a) for a in load_asset_index(Path(args.asset_index).resolve() if args.asset_index else None)]
    for a in assets:
        map_id = a.get('map_id', '')
        planner = a.get('planner', '')
        print(f"{map_id}\t{planner}\t{a.get('plan_json','')}\t{a.get('map_yaml','')}\t{a.get('world_file','')}")
    return 0


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description='Experiment asset index helper')
    sub = p.add_subparsers(dest='cmd', required=True)

    p_resolve = sub.add_parser('resolve', help='resolve one asset entry')
    p_resolve.add_argument('--asset-index', default='')
    p_resolve.add_argument('--map-id', default='')
    p_resolve.add_argument('--planner', default='')
    p_resolve.add_argument('--plan-file', default='')
    p_resolve.add_argument('--map-yaml', default='')
    p_resolve.add_argument('--world-file', default='')
    p_resolve.add_argument('--field', default='all', choices=['all', 'plan_json', 'summary_json', 'map_yaml', 'world_file', 'map_pgm', 'geojson_path', 'preview_path', 'model_root', 'model_dir', 'map_id', 'planner'])
    p_resolve.set_defaults(func=cli_resolve)

    p_list = sub.add_parser('list', help='list all indexed assets')
    p_list.add_argument('--asset-index', default='')
    p_list.set_defaults(func=cli_list)
    return p


def main() -> int:
    p = build_argparser()
    args = p.parse_args()
    return int(args.func(args))


if __name__ == '__main__':
    raise SystemExit(main())
