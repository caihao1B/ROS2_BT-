#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
source optimization_v2/env.sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
py="$project_root/optimization_v2/venvs/scene/bin/python"
scripts="$project_root/optimization_v2/ws/src/showcase_demo/scripts"
command="${1:-status}"
if (( $# )); then shift; fi
case "$command" in
  check)
    "$py" optimization_v2/ws/src/showcase_demo/tests/monitor_contract.py
    "$py" optimization_v2/ws/src/showcase_demo/tests/dynamic_memory_contract.py
    "$py" optimization_v2/ws/src/showcase_demo/tests/residual_contract.py
    "$py" optimization_v2/ws/src/showcase_demo/tests/inspection_recovery_contract.py
    ;;
  plan)
    if [[ -f optimization_v2/showcase/v1/mission_plan.json && " $* " != *" --out "* ]]; then
      echo 'v1 已冻结。新任务请指定 --out optimization_v2/showcase/v2，不覆盖已录制任务。' >&2; exit 2
    fi
    exec flock -n native_20261108/runs/session.lock "$py" "$scripts/plan_showcase.py" "$@"
    ;;
  run) cluster="${1:?Specify A1 A2 A3 B1 B2 or B3}"; shift; exec "$py" "$scripts/run_showcase.py" --cluster "$cluster" "$@" ;;
  inspect)
    kind="${1:?Specify diff or ackermann}"; shift
    case "$kind" in diff) cluster=A1;; ackermann) cluster=B1;; *) exit 2;; esac
    exec "$py" "$scripts/run_showcase.py" --cluster "$cluster" --operation inspection "$@"
    ;;
  dashboard)
    if (( $# )); then directory="$1"; shift; exec "$py" "$scripts/dashboard.py" --run "$directory" "$@"
    else exec "$py" "$scripts/dashboard.py" --latest; fi
    ;;
  status)
    exec "$py" -c "import json;from pathlib import Path;p=Path('optimization_v2/showcase');print('S17 formal: PARTIAL; S18: NOT_RUN');files=[p/'current.json',p/'recording_budget.json',p/'demo_acceptance.json'];current=p/'current.json';files.extend([Path(json.loads(current.read_text())['run'])/'client/motion_monitor.json'] if current.exists() else []);[(print(f.name),print(json.dumps(json.loads(f.read_text()),ensure_ascii=False,indent=2))) for f in files if f.exists()]"
    ;;
  *) echo 'Usage: showcase.sh {check|plan|run A1|inspect diff|dashboard [run_dir]|status}'; exit 2 ;;
esac
