# Global pose manager / S07

Run through `/home/ceihao/robot_project/optimization_v2/stepctl.sh`; do not launch a second simulator beside another stage. Production online nodes use ROS domain 119. Simulated raw sensor generation is isolated in 121 and independent truth scoring in 120. Synthetic C++ unit tests use 123 without actuators.

- C++ `global_pose_manager`: sole public `map → odom`, state and admission; S05 retains `odom → base_link`.
- `localization_preprocessor.py`: noisy stereo/lidar + measured local odometry, original-epoch motion alignment; forwards local TF into RTAB's isolated TF input.
- RTAB-Map: read-only candidate localization from the existing 3D database, optimized local scan submaps.
- `map_refinement.py` / `localization_quality.py`: tiny bounded rigid refinement and held-out elevated-surface validation; never oracle localization.
- Runtime/stage/analysis/report: owned processes, actual restart/occlusion, fixed-frame scoring and version-bound receipts.

Map artifacts are protected under `optimization_v2/maps/frozen`. The plan request is explicitly a diagnostic fixture; actual coverage-plan integration is reserved for S12. Synthetic unit tests supplement, and never replace, real simulation captures.

Chinese technical, code, command and result documents: `/home/ceihao/robot_project/docs/optimization_v2/S07`.
