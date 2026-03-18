# Workspace notes

## 设计目标

这个工作空间偏向“实验台”而不是“历史归档”。

重点是：
- 三终端稳定运行
- 可导入新的 geojson / plan json
- 自动生成 map/world/collision model
- 自动记录任务层、轨迹层、覆盖层、costmap 层数据
- 障碍模式单独控制

## 当前默认建议

- 静态基线：`bash scripts/apply_profile.sh static`
- 动态测试：`bash scripts/apply_profile.sh dynamic`
- 首图：`low_09_8p6m_corridor_vertical`
- 覆盖半径：`--tool-radius 0.425`
- 障碍 bounds：优先 `plan`

## 资源统计说明

`summary.json` 会尝试汇总：
- system CPU / memory
- Gazebo 进程组
- Nav2 进程组
- mission 层进程组

如果系统里没有 `psutil`，资源文件仍会创建，但不会写入有效统计；这不会影响主实验流程。

## costmap 统计说明

logger 订阅：
- `/global_costmap/costmap_raw`
- `/local_costmap/costmap_raw`

会在每次采样时记录机器人当前位置对应的代价值，便于后续判断：
- 路径是否长期处于高代价区
- 转弯段是否把机器人推到接近 lethal 的位置

## 导入新资产时的建议

如果你后续提供：
- `my_map.geojson`
- `my_map.json`

建议优先用：

```bash
python3 scripts/import_asset_pair.py \
  --map-id my_map \
  --planner custom \
  --geojson /path/to/my_map.geojson \
  --plan-json /path/to/my_map.json \
  --overwrite
```

这样会自动更新：
- `datasets/imported/...`
- `e1_assets/maps/...`
- `e1_assets/worlds/...`
- `e1_assets/models/...`
- `e1_assets/asset_index.json`
