#!/usr/bin/env bash
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_YAML="${1:-$WS/src/farm_gate_world/maps/map.yaml}"
MODEL_NAME="${2:-farm_map_collision}"
OUT_DIR="${3:-$WS/src/farm_gate_world/models}"
python3 "$WS/scripts/gen_map_collision_model.py"   --map "$MAP_YAML"   --model-name "$MODEL_NAME"   --out-model-dir "$OUT_DIR"   --wall-height 1.0   --with-visual 1
