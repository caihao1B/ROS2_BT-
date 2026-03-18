#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
METHOD="bt_full"
PLANNER="f2c_geom"
ASSET_INDEX="$WS/e1_assets/asset_index.json"
RUNS_ROOT="$WS/runs/e1_batch"
LIMIT=0
WITH_RVIZ=0
SPAWN_SEG_IDX=0
RUN_TIMEOUT_SEC="${RUN_TIMEOUT_SEC:-900}"
FILTER_MAP_ID=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --method) METHOD="$2"; shift 2;;
    --planner) PLANNER="$2"; shift 2;;
    --asset-index) ASSET_INDEX="$2"; shift 2;;
    --runs-root) RUNS_ROOT="$2"; shift 2;;
    --limit) LIMIT="$2"; shift 2;;
    --rviz) WITH_RVIZ="$2"; shift 2;;
    --spawn-seg) SPAWN_SEG_IDX="$2"; shift 2;;
    --timeout-sec) RUN_TIMEOUT_SEC="$2"; shift 2;;
    --map-id) FILTER_MAP_ID="$2"; shift 2;;
    *) echo "[E1][BATCH] unknown arg: $1" >&2; exit 2;;
  esac
done

mkdir -p "$RUNS_ROOT"

readarray -t LINES < <(python3 - <<'PY' "$ASSET_INDEX" "$PLANNER" "$FILTER_MAP_ID" "$LIMIT"
import json,sys
from pathlib import Path
index=Path(sys.argv[1])
planner=sys.argv[2]
filter_map=sys.argv[3]
limit=int(sys.argv[4])
obj=json.load(open(index,'r',encoding='utf-8'))
assets=obj['assets'] if isinstance(obj,dict) else obj
assets=[a for a in assets if a.get('planner')==planner]
if filter_map:
    assets=[a for a in assets if a.get('map_id')==filter_map]
assets=sorted(assets, key=lambda a:(a.get('density',''), float(a.get('size_m',0) or 0), a.get('map_id','')))
seen=[]
for a in assets:
    mid=a.get('map_id','')
    if mid not in seen:
        seen.append(mid)
    if limit and len(seen)>limit:
        break
    print(mid)
PY
)

for MAP_ID in "${LINES[@]}"; do
  [[ -z "$MAP_ID" ]] && continue
  STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
  OUT_DIR="$RUNS_ROOT/$METHOD/$PLANNER/$MAP_ID/$STAMP"
  mkdir -p "$OUT_DIR"

  echo "[E1][BATCH] ===== map=$MAP_ID planner=$PLANNER method=$METHOD ====="
  bash "$WS/scripts/kill_stale_processes.sh" >/dev/null 2>&1 || true

  bash "$WS/scripts/e1_run_t1.sh" --map-id "$MAP_ID" --planner "$PLANNER" --asset-index "$ASSET_INDEX" --rviz "$WITH_RVIZ" --spawn-seg "$SPAWN_SEG_IDX" >"$OUT_DIR/t1.log" 2>&1 &
  T1_PID=$!
  sleep 5
  bash "$WS/scripts/e1_run_t2.sh" --map-id "$MAP_ID" --planner "$PLANNER" --asset-index "$ASSET_INDEX" --spawn-seg "$SPAWN_SEG_IDX" >"$OUT_DIR/t2.log" 2>&1 &
  T2_PID=$!
  sleep 12

  set +e
  bash "$WS/scripts/e1_run_t3_method.sh" --method "$METHOD" --map-id "$MAP_ID" --planner "$PLANNER" --asset-index "$ASSET_INDEX" --out-dir "$OUT_DIR" --timeout-sec "$RUN_TIMEOUT_SEC" >"$OUT_DIR/t3_driver.log" 2>&1
  RC=$?
  set -e
  echo "[E1][BATCH] T3 rc=$RC for $MAP_ID"

  kill "$T2_PID" 2>/dev/null || true
  wait "$T2_PID" 2>/dev/null || true
  kill "$T1_PID" 2>/dev/null || true
  wait "$T1_PID" 2>/dev/null || true
  bash "$WS/scripts/kill_stale_processes.sh" >/dev/null 2>&1 || true
  sleep 3
done

echo "[E1][BATCH] finished. Aggregating..."
python3 "$WS/scripts/e1_aggregate_results.py" --runs-root "$RUNS_ROOT" --asset-index "$ASSET_INDEX" --out-csv "$RUNS_ROOT/aggregate_runs.csv" --out-summary-json "$RUNS_ROOT/aggregate_summary.json"
