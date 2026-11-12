# GNSS supervisor — S08

S08 session replaces the S07 global TF owner without modifying S01–S07. Actual observation processing and runtime nodes upstream remain reused. See `/home/ceihao/robot_project/docs/optimization_v2/S08/TECHNICAL.md` and the public `optimization_v2/stepctl.sh` interface.

Only `/s08/status.navigation_allowed` together with S05 safety and drive protection permits diagnostic motion. Single-antenna GNSS has no stationary yaw. Site control points are declared simulation survey priors; field deployment requires surveyed correspondences. No truth subscriptions. GNSS is never sent as a strong RTAB prior. Frozen map remains read-only.

The public `--case` selects a documented **test observation fixture**, including injected multipath and map mismatch; it is not RF propagation simulation. Runtime startup is derived from S07 orchestration, estimator implementations are reused. Four serial acceptance captures use 280 simulation seconds plus bounded shutdown margin within a 300-second requested budget.
