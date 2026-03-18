#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def read_json(path: Path) -> Any:
    with path.open('r', encoding='utf-8') as f:
        return json.load(f)


def load_asset_lookup(index_path: Path) -> Dict[Tuple[str, str], Dict[str, Any]]:
    data = read_json(index_path)
    assets = data['assets'] if isinstance(data, dict) else data
    out: Dict[Tuple[str, str], Dict[str, Any]] = {}
    for a in assets:
        out[(str(a.get('map_id', '')), str(a.get('planner', '')))] = a
    return out


def nominal_plan_length(asset: Dict[str, Any], ws_root: Path) -> Optional[float]:
    sp = str(asset.get('summary_json', '') or '').strip()
    if not sp:
        return None
    p = Path(sp)
    if not p.is_absolute():
        p = (ws_root / p).resolve()
    if not p.is_file():
        return None
    try:
        data = read_json(p)
        v = data.get('path_length_m', None)
        return float(v) if v is not None else None
    except Exception:
        return None


def collect_rows(runs_root: Path, asset_lookup: Dict[Tuple[str, str], Dict[str, Any]], ws_root: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for path in runs_root.rglob('summary.json'):
        try:
            s = read_json(path)
        except Exception:
            continue
        map_id = str(s.get('map_id', ''))
        planner = str(s.get('planner', ''))
        method = str(s.get('method', ''))
        scenario_tag = str(s.get('scenario_tag', '') or '')
        asset = asset_lookup.get((map_id, planner), {})
        nominal_len = nominal_plan_length(asset, ws_root) if asset else s.get('nominal_plan_length_m', None)
        cov = s.get('coverage', {}) or {}
        tracking = s.get('tracking_error', {}) or {}
        turn_tracking = s.get('turn_tracking_error', {}) or {}
        g_cost = s.get('global_cost', {}) or {}
        l_cost = s.get('local_cost', {}) or {}
        row = {
            'run_dir': str(path.parent.resolve()),
            'map_id': map_id,
            'planner': planner,
            'method': method,
            'scenario_tag': scenario_tag,
            'success': int(bool(s.get('success', False))),
            'aborted': int(bool(s.get('aborted', False))),
            'duration_sec': s.get('duration_sec', None),
            'path_length_m': s.get('path_length_m', None),
            'coverage_ratio': cov.get('coverage_ratio', None),
            'overlap_ratio': cov.get('overlap_ratio', None),
            'covered_area_m2': cov.get('covered_area_m2', None),
            'uncovered_free_area_m2': cov.get('uncovered_free_area_m2', None),
            'event_count': s.get('event_count', 0),
            'end_reason': s.get('end_reason', ''),
            'nominal_plan_length_m': nominal_len,
            'path_vs_plan_ratio': (float(s['path_length_m']) / float(nominal_len)) if nominal_len and s.get('path_length_m') is not None else None,
            'tracking_p95_m': tracking.get('p95', None),
            'tracking_max_m': tracking.get('max', None),
            'turn_tracking_p95_m': turn_tracking.get('p95', None),
            'turn_tracking_max_m': turn_tracking.get('max', None),
            'global_high_cost_ratio': g_cost.get('high_cost_ratio', None),
            'local_high_cost_ratio': l_cost.get('high_cost_ratio', None),
            'c_refresh': int((s.get('event_counts', {}) or {}).get('c_refresh', 0)),
            'follow_aborted': int((s.get('event_counts', {}) or {}).get('follow_aborted', 0)),
            'segment_done': int((s.get('event_counts', {}) or {}).get('segment_done', 0)),
            'mission_complete': int((s.get('event_counts', {}) or {}).get('mission_complete', 0)),
            'executor_done': int((s.get('event_counts', {}) or {}).get('executor_done', 0)),
            'd1_applied': int((s.get('event_counts', {}) or {}).get('d1_applied', 0)),
            'd2_replace': int((s.get('event_counts', {}) or {}).get('d2_replace', 0)),
        }
        rows.append(row)

    base_lookup: Dict[Tuple[str, str, str], Dict[str, Any]] = {}
    for r in rows:
        if r['method'] == 'nav2_base':
            base_lookup[(r['map_id'], r['planner'], r['scenario_tag'])] = r
    for r in rows:
        b = base_lookup.get((r['map_id'], r['planner'], r['scenario_tag'])) or base_lookup.get((r['map_id'], r['planner'], 'nominal'))
        if b and r['duration_sec'] is not None and b.get('duration_sec') not in (None, 0, 0.0):
            try:
                r['duration_vs_nav2_base'] = float(r['duration_sec']) / float(b['duration_sec'])
            except Exception:
                r['duration_vs_nav2_base'] = None
        else:
            r['duration_vs_nav2_base'] = None
        if b and r['path_length_m'] is not None and b.get('path_length_m') not in (None, 0, 0.0):
            try:
                r['path_vs_nav2_base'] = float(r['path_length_m']) / float(b['path_length_m'])
            except Exception:
                r['path_vs_nav2_base'] = None
        else:
            r['path_vs_nav2_base'] = None
    return rows


def summarize(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    groups: Dict[Tuple[str, str, str], List[Dict[str, Any]]] = {}
    for r in rows:
        groups.setdefault((r['method'], r['planner'], r['scenario_tag']), []).append(r)

    def numvals(key: str, items: List[Dict[str, Any]]) -> List[float]:
        vals = []
        for it in items:
            v = it.get(key, None)
            if isinstance(v, (int, float)):
                vals.append(float(v))
        return vals

    out: Dict[str, Any] = {'groups': []}
    for (method, planner, scenario_tag), items in sorted(groups.items()):
        entry = {'method': method, 'planner': planner, 'scenario_tag': scenario_tag, 'runs': len(items)}
        for key in ['success', 'coverage_ratio', 'duration_sec', 'path_length_m', 'tracking_p95_m', 'turn_tracking_p95_m', 'global_high_cost_ratio', 'local_high_cost_ratio']:
            vals = numvals(key, items)
            if vals:
                prefix = key.replace('_m', '').replace('_ratio', '_ratio')
                entry[f'{key}_mean'] = sum(vals) / len(vals)
                entry[f'{key}_median'] = statistics.median(vals)
            else:
                entry[f'{key}_mean'] = None
                entry[f'{key}_median'] = None
        out['groups'].append(entry)
    return out


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    fieldnames = [
        'run_dir', 'map_id', 'planner', 'method', 'scenario_tag', 'success', 'aborted', 'duration_sec', 'path_length_m',
        'coverage_ratio', 'overlap_ratio', 'covered_area_m2', 'uncovered_free_area_m2', 'event_count', 'end_reason',
        'nominal_plan_length_m', 'path_vs_plan_ratio', 'duration_vs_nav2_base', 'path_vs_nav2_base',
        'tracking_p95_m', 'tracking_max_m', 'turn_tracking_p95_m', 'turn_tracking_max_m',
        'global_high_cost_ratio', 'local_high_cost_ratio',
        'c_refresh', 'follow_aborted', 'segment_done', 'mission_complete', 'executor_done', 'd1_applied', 'd2_replace'
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in fieldnames})


def parse_args() -> argparse.Namespace:
    ws = Path(__file__).resolve().parent.parent
    p = argparse.ArgumentParser(description='Aggregate experiment run summaries')
    p.add_argument('--runs-root', default=str(ws / 'runs' / 'e1'))
    p.add_argument('--asset-index', default=str(ws / 'e1_assets' / 'asset_index.json'))
    p.add_argument('--out-csv', default=str(ws / 'runs' / 'e1' / 'aggregate_runs.csv'))
    p.add_argument('--out-summary-json', default=str(ws / 'runs' / 'e1' / 'aggregate_summary.json'))
    return p.parse_args()


def main() -> int:
    args = parse_args()
    ws_root = Path(__file__).resolve().parent.parent
    asset_lookup = load_asset_lookup(Path(args.asset_index).resolve()) if Path(args.asset_index).is_file() else {}
    rows = collect_rows(Path(args.runs_root).resolve(), asset_lookup, ws_root)
    write_csv(Path(args.out_csv).resolve(), rows)
    summary = summarize(rows)
    with Path(args.out_summary_json).resolve().open('w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(f'[EXP] aggregated {len(rows)} runs -> {Path(args.out_csv).resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
