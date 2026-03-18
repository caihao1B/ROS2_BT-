Files in this bundle:
- executor_e1_static.yaml        : static E1 / nominal profile
- executor_dynamic.yaml          : dynamic E2/E3/E4/E5 profile
- supervisor_e1_static.yaml      : BT-Full static nominal profile
- supervisor_dynamic_full.yaml   : BT-Full dynamic full profile
- supervisor_norepair.yaml       : ablation profile snippet (copy onto supervisor.yaml manually)
- supervisor_closure_off.yaml    : accepted-closure-off ablation snippet
- dynamic_obstacles_small_s1_crossing.yaml
- dynamic_obstacles_small_s2_blocking.yaml
- dynamic_obstacles_small_s3_mixed.yaml

Suggested use:
1) Import maps/plans with prepare_small_benchmark_assets.py
2) Run scripts/e1_prepare_assets.py --overwrite
3) Copy executor/supervisor profile into src/.../params/ before each experiment.
4) Copy dynamic obstacle suite yaml into src/coverage_sim/params/ and adjust suite_segment_indices per map.
