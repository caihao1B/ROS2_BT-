# coverage_sim Dynamic Obstacles (Standalone Module)

This module spawns and drives dynamic obstacles in **Gazebo Classic** via ROS 2 services:

- `SpawnEntity`
- `SetEntityState`
- (optional) `DeleteEntity`

It is designed for the thesis **“Online Coverage Path Planning & Navigation in Agriculture (ROS 2 + Gazebo)”** and includes typical agricultural behaviors:

- **A) Transient / Crossing** (tests WAIT branch)
- **B) Co-moving / Parallel** (tests YIELD branch)
- **C) Persistent / Blocking** (tests DEFER / REPLAN branches)

## Quick start (recommended)

1) Start your simulation (Gazebo + Nav2 + robot), e.g. milestone0:

```bash
ros2 launch farm_gate_bringup milestone0_farm_gate.launch.py
```

2) In another terminal, start the dynamic obstacles:

```bash
ros2 launch coverage_sim dynamic_obstacles.launch.py
```

## Scenario presets

See `coverage_sim/params/`:

- `dynamic_obstacles_agri_suite.yaml`  (mixed A/B/C + random-walk)
- `dynamic_obstacles_case_A_crossing.yaml`
- `dynamic_obstacles_case_B_parallel.yaml`
- `dynamic_obstacles_case_C_blocking.yaml`

You can swap presets by passing `params_file:=...` to the launch file.

## Custom scenario file

There is an example in `coverage_sim/scenarios/example_agri_scenario.yaml`.

Run:

```bash
ros2 launch coverage_sim dynamic_obstacles.launch.py scenario:=yaml \
  scenario_file:=$(ros2 pkg prefix coverage_sim)/share/coverage_sim/scenarios/example_agri_scenario.yaml
```

