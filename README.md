# coverage_ws_exp

`coverage_ws_exp` is a ROS 2 experimental workspace for coverage planning, Behavior-Tree-based supervision, dynamic-obstacle experiments, and reproducible runtime logging.

It appears to be derived from an earlier `coverage_ws_farm_gate_e1_static_bt_workspace`, but the current snapshot is no longer just a minimal static-E1 bundle. In its present form, it includes:

- a stable **multi-terminal execution workflow** for simulation and task execution,
- a **dynamic-obstacle stack** (perception, prediction, decision, replanning, BT supervision),
- an **asset import / asset indexing pipeline** for new maps and plans,
- a **runtime logger** that exports experiment summaries and time series,
- a bundled set of **thesis-ready sample maps** and **additional generated benchmark assets**.

The workspace is best understood as a practical experiment bench: it keeps the executable core, the map/plan asset pipeline, and the experiment scripts needed for repeatable evaluation.

---

## Main Capabilities

### 1. Multi-terminal experiment workflow

The workspace keeps a clear terminal split for experiments:

- **T1**: Gazebo + Nav2 + map + world
- **T2**: static `map -> odom` TF
- **T3**: task execution (`bt_full`, `bt_none`, or `nav2_base`) with automatic logging
- **T4** *(optional)*: dynamic obstacle injection

This separation makes debugging easier and avoids coupling obstacle generation to the core simulator bringup.

### 2. Importing new maps and plans

You can import new `geojson + plan json` pairs and automatically generate:

- `map.yaml` + `map.pgm`
- Gazebo collision models
- Gazebo world files
- a portable relative-path asset index at `e1_assets/asset_index.json`

Supported scripts:

- `python3 scripts/import_asset_pair.py ...`
- `python3 scripts/import_asset_bundle.py ...`

### 3. Runtime logging and experiment summaries

The main runtime logger writes the following files into each run directory:

- `summary.json`
- `timeseries.csv`
- `odom_samples.csv`
- `events.jsonl`
- `decisions.jsonl`
- `current_path.jsonl`
- `resources.csv`
- `run_meta.json`

The current logger also records metrics such as:

- covered / uncovered area
- nominal plan length
- actual path length
- path-vs-nominal ratio
- tracking error statistics
- turn-zone tracking error statistics
- global / local costmap statistics
- optional CPU / memory statistics via `psutil`

### 4. Dynamic-obstacle experiments

Obstacle injection is handled independently through:

- `bash scripts/run_obstacles.sh --scenario crossing ...`
- `bash scripts/run_obstacles.sh --scenario blocking ...`
- `bash scripts/run_obstacles.sh --scenario mixed ...`
- `bash scripts/run_obstacles.sh --scenario still ...`

The `still` mode is supported in the current snapshot and should be documented alongside the other obstacle modes.

### 5. Profile switching

Experiment profiles can be applied directly with:

- `bash scripts/apply_profile.sh static`
- `bash scripts/apply_profile.sh dynamic`
- `bash scripts/apply_profile.sh norepair`
- `bash scripts/apply_profile.sh closure_off`

### 6. Batch and aggregation utilities

In addition to manual terminal-based runs, the workspace also includes:

- `bash scripts/e1_batch_headless.sh ...` for headless batch execution
- `python3 scripts/e1_aggregate_results.py --runs-root ...` for aggregation

---

## Workspace Layout

- `src/` — ROS 2 packages
- `config/` — Nav2 and related runtime parameters
- `profiles/` — experiment profiles and obstacle scenario configs
- `e1_assets/` — generated runnable assets (`map`, `world`, collision models, previews, asset index)
- `datasets/` — raw and imported GeoJSON / plan JSON / optional summaries
- `scripts/` — experiment drivers, launch wrappers, asset importers, log tools
- `generated_fc_mppi_closedloop_18/` — generated benchmark asset source bundle included in the current snapshot
- `runs/` — example run outputs included in the current snapshot

---

## ROS 2 Packages in `src/`

The current workspace contains these ROS 2 packages:

- `coverage_msgs` — custom messages and services
- `coverage_metrics` — coverage and task metrics
- `coverage_perception` — scan-based obstacle tracking
- `coverage_prediction` — obstacle prediction
- `coverage_replanner` — local replanning, region state, D1/D2 advisors
- `coverage_bt_supervisor` — BT-based supervisory logic
- `coverage_decision` — strategy selection
- `coverage_manager` — coverage execution and task-level control
- `coverage_sim` — dynamic obstacle simulation utilities
- `farm_gate_world` — map/world/model resources
- `farm_gate_bringup` — bringup / launch support

---

## Bundled Assets in the Current Snapshot

The current uploaded workspace is **not limited to only four example maps**.

At the moment, `e1_assets/asset_index.json` contains **22 runnable assets** under planner `f2c_geom`, including:

- **4 thesis-ready low-density sample maps**
  - `low_01_5p0m_scatter`
  - `low_08_8p2m_ring`
  - `low_09_8p6m_corridor_vertical`
  - `low_11_9p5m_dual_cluster_lr`
- **18 generated benchmark maps**
  - `s05_*` (6 maps)
  - `s07p5_*` (6 maps)
  - `s10_*` (6 maps)

So if this README is meant to describe the current repository state, it should not claim that only four maps remain.

---

## Recommended Environment

This workspace is written for a ROS 2 Humble simulation workflow and expects a Linux environment similar to:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- Nav2
- TurtleBot3 simulation packages
- Python 3

Optional:

- `psutil` for resource monitoring in `resources.csv` and `summary.json`

---

## Build

You can build the workspace either with the helper script:

```bash
bash scripts/build_ws.sh
```

or manually:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Then source the workspace in each terminal:

```bash
source scripts/cov_source.bash
```

The helper `cov_source.bash` also defines convenient aliases such as:

- `covt1`
- `covt2`
- `covt3full`
- `covt3none`
- `covt3base`
- `covobs`
- `covprofile`

---

## Recommended First Experiment

Apply the static profile first:

```bash
bash scripts/apply_profile.sh static
```

Then run the three-terminal workflow.

### Terminal 1

```bash
source scripts/cov_source.bash
bash scripts/run_t1.sh --map-id low_09_8p6m_corridor_vertical --planner f2c_geom --rviz 1
```

### Terminal 2

```bash
source scripts/cov_source.bash
bash scripts/run_t2.sh --map-id low_09_8p6m_corridor_vertical --planner f2c_geom
```

### Terminal 3

```bash
source scripts/cov_source.bash
RUN_TAG=$(date -u +%Y%m%dT%H%M%SZ)
OUT="$PWD/runs/manual_low09_nominal/$RUN_TAG"
mkdir -p "$OUT"

bash scripts/run_t3_bt_full.sh \
  --map-id low_09_8p6m_corridor_vertical \
  --planner f2c_geom \
  --tool-radius 0.425 \
  --scenario-tag nominal \
  --out-dir "$OUT"
```

---

## Dynamic-Obstacle Run (Optional Terminal 4)

Switch to the dynamic profile first:

```bash
bash scripts/apply_profile.sh dynamic
```

Then start obstacles in a fourth terminal:

```bash
source scripts/cov_source.bash
bash scripts/run_obstacles.sh \
  --scenario crossing \
  --map-id low_09_8p6m_corridor_vertical \
  --planner f2c_geom \
  --segments '[8,9,10,11]' \
  --seed 101 \
  --startup-wait 8.0
```

Other supported scenarios in the current script are:

- `crossing`
- `blocking`
- `mixed`
- `still`

---

## Importing Your Own Map / Plan Pair

### Single import

```bash
python3 scripts/import_asset_pair.py \
  --map-id my_map_01 \
  --planner custom \
  --geojson /path/to/my_map_01.geojson \
  --plan-json /path/to/my_map_01.json \
  --overwrite
```

### Batch import

```bash
python3 scripts/import_asset_bundle.py \
  --planner custom \
  --geojson-dir /path/to/geojson_dir \
  --plan-dir /path/to/json_dir \
  --overwrite
```

After import, verify registration with:

```bash
python3 scripts/e1_common.py list
python3 scripts/e1_common.py resolve --map-id my_map_01 --planner custom
```

---

## Result Aggregation

To aggregate run outputs under a run root:

```bash
python3 scripts/e1_aggregate_results.py --runs-root /path/to/runs_root
```

For headless batch execution plus aggregation:

```bash
bash scripts/e1_batch_headless.sh --method bt_full --planner f2c_geom
```

---

## Repository Hygiene Notes

If this workspace is going to be pushed to GitHub as a **source repository**, the current snapshot should be cleaned up or ignored appropriately.

The uploaded archive still contains runtime / build artifacts such as:

- `build/`
- `install/`
- `log/`
- `runs/`
- `__pycache__/`

These directories generally should **not** be version-controlled.

Depending on your goal, you may also choose whether to keep or ignore:

- `generated_fc_mppi_closedloop_18/`

If the repository is meant to be lightweight and source-focused, that directory may also be excluded unless you want to publish the generated benchmark asset source bundle with the code.

---

## Summary

In its current form, `coverage_ws_exp` is not just a stripped-down static demo workspace. It is a broader ROS 2 experiment workspace that combines:

- simulation bringup,
- task execution,
- BT supervision,
- perception / prediction / decision / replanning,
- asset generation and import,
- manual and batch experiment workflows,
- and detailed run logging.

That makes it suitable both for thesis-style controlled experiments and for continued extension with new map/plan assets.
