#!/usr/bin/env bash
set -eo pipefail

WS="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
RUN_ID="${2:-latest}"
OUTDIR_BASE="${3:-$HOME/t3_debug_bundles}"

mkdir -p "$OUTDIR_BASE"

if [[ "$RUN_ID" == "latest" ]]; then
  RUN_DIR="$(ls -1dt "$HOME"/.ros/ae_events/* 2>/dev/null | head -n1 || true)"
else
  RUN_DIR="$HOME/.ros/ae_events/$RUN_ID"
fi

if [[ -z "${RUN_DIR}" || ! -d "${RUN_DIR}" ]]; then
  echo "[ERROR] Could not find ae_events run dir. Expected under $HOME/.ros/ae_events"
  exit 2
fi

RUN_ID_REAL="$(basename "$RUN_DIR")"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTDIR="$OUTDIR_BASE/${RUN_ID_REAL}_${STAMP}"
mkdir -p "$OUTDIR"

cp -r "$RUN_DIR" "$OUTDIR/ae_events"

mkdir -p "$OUTDIR/ws_snapshot"
for rel in \
  "src/coverage_bt_supervisor/params/supervisor.yaml" \
  "src/coverage_decision/params/profiles/selector_FULL.yaml" \
  "src/coverage_replanner/params/region_state.yaml" \
  "src/coverage_replanner/params/d2_advisor.yaml" \
  "config/cv_predictor_h12.yaml" \
  "src/farm_gate_bringup/plans/plan_safe_v3.json" \
  "src/farm_gate_bringup/plans/plan_micro_gate_trim_safe_leftblock_test_v2.json" \
  "scripts/d2_accept_case_safe_v3.yaml" \
  "scripts/probe_d2_acceptance.py" \
  "scripts/run_t4_d2_accept_case.sh" \
  "scripts/fake_d2_server.py" \
  "scripts/d2_forced_snapshot_proxy.py" \
  "scripts/run_d2_lab_pipeline.sh" \
  "scripts/run_t3_safe_v3_d2lab_fake.sh" \
  "scripts/run_t3_safe_v3_d2lab_fake_auto_bundle.sh" \
  "scripts/run_t3_safe_v3_d2lab_proxy.sh" \
  "scripts/run_t3_safe_v3_d2lab_proxy_auto_bundle.sh" \
  "scripts/run_t4_d2_extreme_static_case.sh" \
  "scripts/d2_extreme_static_late3C.yaml" \
  "scripts/testdata/d2_probe_late3C_snapshot.json" \
  "scripts/testdata/d2_probe_late3C_canned_response.json"; do
  if [[ -f "$WS/$rel" ]]; then
    mkdir -p "$OUTDIR/ws_snapshot/$(dirname "$rel")"
    cp "$WS/$rel" "$OUTDIR/ws_snapshot/$rel"
  fi
done

mkdir -p "$OUTDIR/runtime_logs"
for f in /tmp/nav2_nav_farm_gate.log /tmp/nav2_nav.log /tmp/coverage_bt_supervisor.log; do
  if [[ -f "$f" ]]; then
    cp "$f" "$OUTDIR/runtime_logs/"
  fi
done
LATEST_T3_LOG="$(ls -1t "$HOME"/fgmini_runs/*_t3*.log 2>/dev/null | head -n1 || true)"
if [[ -n "$LATEST_T3_LOG" && -f "$LATEST_T3_LOG" ]]; then
  cp "$LATEST_T3_LOG" "$OUTDIR/runtime_logs/"
fi

LATEST_PROBE_JSONS=$(ls -1t "$HOME"/t3_debug_bundles/d2_probe_*_response.json 2>/dev/null | head -n3 || true)
if [[ -n "$LATEST_PROBE_JSONS" ]]; then
  mkdir -p "$OUTDIR/runtime_logs/d2_probe"
  while IFS= read -r f; do
    [[ -f "$f" ]] && cp "$f" "$OUTDIR/runtime_logs/d2_probe/" || true
    s="${f%_response.json}_snapshot.json"
    [[ -f "$s" ]] && cp "$s" "$OUTDIR/runtime_logs/d2_probe/" || true
  done <<< "$LATEST_PROBE_JSONS"
fi

python3 - "$RUN_DIR/events.jsonl" "$OUTDIR/summary.md" <<'PY'
import json, sys, collections, statistics, pathlib

events_path = pathlib.Path(sys.argv[1])
out_path = pathlib.Path(sys.argv[2])
events = []
with events_path.open() as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        try:
            events.append(json.loads(line))
        except Exception:
            pass

mode_counts = collections.Counter(e.get("mode", "?") for e in events)

def safe_get(e, *keys, default=None):
    cur = e
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur

frontmins, allmins = [], []
for e in events:
    fm = safe_get(e, "snapshots", "scan", "features", "front_min_m")
    am = safe_get(e, "snapshots", "scan", "features", "all_min_m")
    if isinstance(fm, (int, float)):
        frontmins.append(float(fm))
    if isinstance(am, (int, float)):
        allmins.append(float(am))

lines = []
lines.append(f"# T3 Run Summary: {events_path.parent.name}")
lines.append("")
lines.append(f"- events_total: {len(events)}")
for k, v in sorted(mode_counts.items()):
    lines.append(f"- mode_{k}: {v}")
if frontmins:
    lines.append(f"- front_min_m_min: {min(frontmins):.3f}")
    lines.append(f"- front_min_m_median: {statistics.median(frontmins):.3f}")
if allmins:
    lines.append(f"- all_min_m_min: {min(allmins):.3f}")
    lines.append(f"- all_min_m_median: {statistics.median(allmins):.3f}")
lines.append("")
lines.append("## Last 20 events")
for e in events[-20:]:
    lines.append(f"- t={e.get('ros_time','?')} mode={e.get('mode','?')} event_id={e.get('event_id','?')}")
out_path.write_text("\n".join(lines), encoding="utf-8")
PY

python3 - "$OUTDIR/runtime_logs" "$OUTDIR/analysis.md" <<'PY'
import pathlib, re, sys, collections
log_dir = pathlib.Path(sys.argv[1])
out_path = pathlib.Path(sys.argv[2])
log_files = sorted(log_dir.glob('*.log'))
text = ''
for p in log_files:
    try:
        text += f"\n===== {p.name} =====\n" + p.read_text(encoding='utf-8', errors='ignore')
    except Exception:
        pass
patterns = {
    'd2_check': r'D2 check cluster=',
    'd2_replace_accepted': r'D2 replace accepted',
    'd2_replace_false': r'D2 unavailable or replace_plan=false',
    'd2_result': r'D2 result backend=',
    'd2_test_force': r'D2 TEST FORCE',
    'd2_rpc_timeout': r'D2 rpc timeout',
    'watchdog_no_progress': r'detour watchdog fired',
    'close_without_revisit': r'close seg .* without revisit',
    'tiny_residual_close': r'tiny residual .*close without',
    'mission_complete': r'mission complete',
}
counts = {k: len(re.findall(v, text)) for k, v in patterns.items()}
interesting = []
for line in text.splitlines():
    if any(tok in line for tok in ['D2 check', 'D2 TEST FORCE', 'D2 result', 'D2 replace accepted', 'D2 rpc timeout', 'D2 skipped', 'D2 one-shot latch consumed', 'replace_plan=false', 'detour watchdog fired', 'close seg', 'mission complete', 'tiny residual', 'superseded old seg']):
        interesting.append(line)
lines = ['# Runtime Log Analysis', '']
for k, v in counts.items():
    lines.append(f'- {k}: {v}')
lines.append('')
lines.append('## Last 80 interesting log lines')
for line in interesting[-80:]:
    lines.append(f'- {line}')
out_path.write_text('\n'.join(lines), encoding='utf-8')
PY

{
  echo "# Grep: nav2 / recovery / D2"
  grep -R -nE "ABORTED|status=6|detour exhausted|compute_empty_path|compute_path_rejected|turn-prealign timeout|collision|no progress|D2 TEST FORCE|D2 check|D2 result|D2 rpc timeout|D2 replace accepted|D2 skipped|D2 one-shot latch consumed|replace_plan=false|mission complete|tiny residual|superseded old seg" "$OUTDIR/runtime_logs" 2>/dev/null || true
} > "$OUTDIR/grep_runtime.txt"

ZIP_PATH="${OUTDIR}.zip"
(
  cd "$OUTDIR_BASE"
  zip -qr "$(basename "$ZIP_PATH")" "$(basename "$OUTDIR")"
)

echo "[OK] T3 debug bundle written to:"
echo "  $ZIP_PATH"
