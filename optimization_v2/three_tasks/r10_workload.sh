#!/usr/bin/env bash
# Bounded R10 experiments use separate result directories and keep the original stage receipts.
set -Eeuo pipefail
task_base="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$task_base/env.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
scripts_dir="$task_base/ws/src/task_platform/scripts"
case "${1:-}" in
  profile) script="$task_base/three_tasks/spray_phase_profile.py" ;;
  spray_travel) script="$scripts_dir/qualify_spray_travel_r10.py" ;;
  mowing_full_46) script="$scripts_dir/qualify_b1_full.py" ;;
  mowing_progress) script="$scripts_dir/qualify_b1_progress.py" ;;
  progress_replay) script="$scripts_dir/replay_b1_progress.py" ;;
  *) echo 'Usage: r10_workload.sh {profile|spray_travel|mowing_full_46|mowing_progress|progress_replay} [arguments]' >&2
     exit 2 ;;
esac
shift
cd -- "$task_base/.."
exec "$task_base/venvs/scene/bin/python" "$script" "$@"
