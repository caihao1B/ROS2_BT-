# coverage_ws_exp

这是一个基于现有 `coverage_ws_farm_gate_e1_static_bt_workspace` 整理出来的**纯净实验工作空间**，目标是保留真正有用的实验能力，同时把旧工作空间里不再需要的大量补丁、D2/fakeguard 说明文档、历史运行结果、冗余地图资产和 build/install/log 输出去掉。

## 这个新工作空间保留了什么

1. **三终端实验流程**
   - T1：Gazebo + Nav2 + 地图 + world
   - T2：静态 `map -> odom` TF
   - T3：BT Full / BT None / Nav2 Base + 自动记录

2. **可导入新的地图与路径资产**
   - 支持后续导入新的 `geojson + json`。
   - 自动生成：
     - `map.yaml + map.pgm`
     - Gazebo collision model（box 合并）
     - `world` 文件
     - 资产索引 `e1_assets/asset_index.json`

3. **日志与指标记录**
   - 自动输出：
     - `summary.json`
     - `timeseries.csv`
     - `odom_samples.csv`
     - `events.jsonl`
     - `decisions.jsonl`
     - `current_path.jsonl`
     - `resources.csv`
   - `summary.json` 里增加了：
     - 覆盖面积 / 未覆盖面积
     - nominal path 长度
     - 跟踪误差统计（全局 + 转弯区）
     - global/local costmap 高代价占比
     - 可选资源统计（CPU / 内存）

4. **障碍模式分离**
   - `crossing`
   - `blocking`
   - `mixed`
   - 用 `scripts/run_obstacles.sh` 单独启动，便于和 T1/T2/T3 分开控制。

## 这个新工作空间去掉了什么

- 历史 `build / install / log / runs`
- D2 / fakeguard / segwindow / RPC 修补文档与专用脚本
- 大量不再需要的旧 README、patch 报告、实验残留
- 全量 36 张小图资产，只保留 4 张样例图

## 目前保留的样例地图

- `low_01_5p0m_scatter`
- `low_08_8p2m_ring`
- `low_09_8p6m_corridor_vertical`
- `low_11_9p5m_dual_cluster_lr`

这 4 张图分别覆盖了：开放、环形、狭长走廊、双簇结构四种典型布局。

## 目录结构

- `src/`：ROS2 包源码
- `config/`：Nav2 参数
- `profiles/`：静态 / 动态实验配置
- `e1_assets/`：可运行资产（map / world / collision model / asset index）
- `datasets/`：保留的原始 geojson / plan json，以及之后导入的新数据
- `scripts/`：实验主脚本

## 核心脚本

### 资产导入
- `python3 scripts/import_asset_pair.py ...`
- `python3 scripts/import_asset_bundle.py ...`

### 配置切换
- `bash scripts/apply_profile.sh static`
- `bash scripts/apply_profile.sh dynamic`
- `bash scripts/apply_profile.sh norepair`
- `bash scripts/apply_profile.sh closure_off`

### 三终端实验
- `bash scripts/run_t1.sh ...`
- `bash scripts/run_t2.sh ...`
- `bash scripts/run_t3_bt_full.sh ...`
- `bash scripts/run_t3_bt_none.sh ...`
- `bash scripts/run_t3_nav2_base.sh ...`

### 障碍加载
- `bash scripts/run_obstacles.sh --scenario crossing ...`
- `bash scripts/run_obstacles.sh --scenario blocking ...`
- `bash scripts/run_obstacles.sh --scenario mixed ...`

### 结果聚合
- `python3 scripts/e1_aggregate_results.py --runs-root ...`

## 推荐的第一次实验

先切到静态配置：

```bash
bash scripts/apply_profile.sh static
```

然后三终端跑：

### T1
```bash
source scripts/cov_source.bash
bash scripts/run_t1.sh --map-id low_09_8p6m_corridor_vertical --planner f2c_geom --rviz 1
```

### T2
```bash
source scripts/cov_source.bash
bash scripts/run_t2.sh --map-id low_09_8p6m_corridor_vertical --planner f2c_geom
```

### T3
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

如果要做动态障碍，先把 profile 切成 `dynamic`，然后在**第四个终端**启动障碍：

```bash
bash scripts/apply_profile.sh dynamic
```

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

## 后续导入你自己的地图 / 路径

单张导入：

```bash
python3 scripts/import_asset_pair.py \
  --map-id my_map_01 \
  --planner custom \
  --geojson /path/to/my_map_01.geojson \
  --plan-json /path/to/my_map_01.json \
  --overwrite
```

整批导入：

```bash
python3 scripts/import_asset_bundle.py \
  --planner custom \
  --geojson-dir /path/to/geojson_dir \
  --plan-dir /path/to/json_dir \
  --overwrite
```

导入后用下面命令检查是否注册成功：

```bash
python3 scripts/e1_common.py list
python3 scripts/e1_common.py resolve --map-id my_map_01 --planner custom
```
