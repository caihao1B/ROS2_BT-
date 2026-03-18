# New workspace report

## 新工作空间名称

`coverage_ws_exp`

## 主要改动

### 1. 资产索引改为可移植
- `scripts/e1_common.py` 现在支持 **相对路径资产索引**。
- `e1_assets/asset_index.json` 不再依赖固定的 `/home/humble/...` 绝对路径。
- 新工作空间在不同机器和不同目录下移动时，不需要手工重写资产索引。

### 2. 新增资产导入能力
- `scripts/import_asset_pair.py`
- `scripts/import_asset_bundle.py`

支持把新的：
- `geojson`
- `plan json`

导入为：
- `map.yaml + map.pgm`
- `collision model`
- `world`
- `asset_index`

### 3. 三终端流程保留，并增加简化包装
- `scripts/run_t1.sh`
- `scripts/run_t2.sh`
- `scripts/run_t3_bt_full.sh`
- `scripts/run_t3_bt_none.sh`
- `scripts/run_t3_nav2_base.sh`

内部仍沿用稳定的 E1 启动逻辑，减少和旧代码的大面积脱钩。

### 4. logger 增强
`/scripts/e1_runtime_logger.py` 新增：
- nominal path 长度统计
- 覆盖面积 / 未覆盖面积
- 轨迹相对 nominal plan 的跟踪误差统计
- 转弯区跟踪误差统计
- global / local costmap 高代价占比统计
- 可选资源统计（CPU / memory，依赖 `psutil`）

### 5. 障碍加载重整
新增：
- `scripts/run_obstacles.sh`

用于独立启动：
- `crossing`
- `blocking`
- `mixed`

并支持：
- 指定 segment indices
- 指定 seed
- 指定 startup wait
- `plan / map_yaml / manual bbox` 边界模式

### 6. profile 切换更直接
新增：
- `scripts/apply_profile.sh`

直接切换：
- `static`
- `dynamic`
- `norepair`
- `closure_off`

## 保留的样例图
- `low_01_5p0m_scatter`
- `low_08_8p2m_ring`
- `low_09_8p6m_corridor_vertical`
- `low_11_9p5m_dual_cluster_lr`

## 适合的第一个正式实验
推荐先用：
- `low_09_8p6m_corridor_vertical`
- `planner=f2c_geom`
- `method=bt_full`
- `tool_radius=0.425`

先做 nominal，再做 crossing / blocking / mixed。
