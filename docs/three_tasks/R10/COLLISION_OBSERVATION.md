# 点喷车为什么报告前方碰撞

RPP 在每次计算速度时先检查当前车体轮廓，然后检查拟执行速度在一小段时间内扫过的轮廓。轮廓覆盖到致命或本配置禁止通行的未知栅格时，控制器会记录 `detected collision ahead` 并拒绝这次速度。要理解这个提示，需要知道车身的哪一侧、哪个位置以及哪个时刻出现了对应的代价。参考 [Nav2 Jazzy 碰撞检查实现](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/src/collision_checker.cpp)。

## 已有实测

在旧 A1 运行的 9,538 条预警中，用离线真值轨迹与原场景几何核对，约 9,429 条预警发生时车体中心距静态障碍、坡面、保守的脚本运动范围以及作业边界均超过 2 m。采用录像墙钟与仿真时钟插值对齐，允许采样和消息延迟。该批数据只保存了传感器健康和总格数，单条预警涉及的栅格还需要实测取证。[历史核查数据](../../../validation/R10/spray_collision_warning_archive_20261119.json)

2026-11-19 的三点卡滞案例使用只读观察器。任务完成 3/3。控制器共发出 431 条预警，在其中 30 个相隔至少 0.75 仿真秒的时刻比对了最新已发布的局部地图与当前车体轮廓。7 个时刻的轮廓上存在未知格，全部位于车尾，车体坐标约为横向 ±0.29 m 范围内、纵向 −0.52 至 −0.66 m。其余 23 次当前轮廓未见阻挡格。30 次当前轮廓中未见致命格。影响控制器的还可能是下一瞬间的预测轮廓，定位、控制和地图各自的更新时间也会影响复核结果。记录到的 19 个未知边缘格中，与其同一坐标处较新观测层给出的代价有 13 个为 0、6 个为未知。[本次短例空间对照数据](../../../validation/R10/spray_collision_short_20261119.json)

## 在另一终端观察正在运行的点喷任务

先用项目正常命令启动点喷。下列命令在第二个终端运行。设 `run_dir` 为正在运行的那一次点喷目录。脚本会核对任务类型、会话、ROS 域和现有输出，运行结束自动退出。

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
ls -dt optimization_v2/three_tasks/results/R12/spray_2* | head
read -r -p '粘贴正在运行的点喷目录路径: ' run_dir
out_dir="delivery/observations/$(basename "$run_dir")"
python3 optimization_v2/three_tasks/monitor/collision_observer.py \
  --run "$run_dir" --out "$out_dir" --wall-seconds 3600
```

第三个终端可以查看统计。将 `out_dir` 设为上面实际使用的输出目录。

```bash
python3 -m json.tool "$out_dir/summary.json"
tail -n 1 "$out_dir/footprint_samples.jsonl" | python3 -m json.tool
```

| 记录 | 怎么读 |
|---|---|
| `summary.json` | `observed_warning_lines` 是观察器运行期间读到的控制器预警条数。`sampled_warnings` 是进行空间对照的采样次数。按时间限频后，一串连续预警可能只留下一个样本。 |
| `footprint_samples.jsonl` | 每行包括采样时刻、车体里程计位姿、局部地图时间、观测栅格时间以及当前轮廓的分组。数值 −1 表示未知、100 表示致命。每个异常点带同一里程计坐标位置的观测层代价，两层在时钟和网格位置上有差异。另留裁剪栅格和最近公布的一段预测弧。 |
| `private_map_samples.jsonl` | 新版观察器在预警采样时异步查询 Nav2 的只读 `/local_costmap/get_costmap` 服务，记录请求时刻、返回时刻及地图。请求到达之前控制器可能已进行了下一次内部更新。 |

观察器只订阅在线 ROS 域 119 的任务、里程计、地图、地形与预测弧消息，通过只读服务取快照。它只在当前运行生成的控制器日志出现预警时保存小范围栅格，最多 2,200 次，`Ctrl+C` 结束观察器。传感器真值仅用于上述历史离线核查。割草、巡检、工具和底盘控制不依赖此脚本。

## 完整 A1 尝试的测量

会话 `spray_20261119_081218_417252` 按 56 点原任务运行，到达并确认 22/56。只读侧车共看到 6,378 条控制器碰撞预警，按时间间隔取了 501 次私有地图快照。348 次当前轮廓含未知栅格，153 次当前轮廓未见阻断，当前轮廓致命格出现 0 次。718 个未知格中，按车体朝向分侧有 628 个位于后部，682 个格心位于物理车身与加宽轮廓之间。对应的观测层在 592 个位置仍为未知，126 个位置是零代价。当前轮廓和预测扫掠轮廓应分开读取；异步服务响应也可能晚于发出预警的控制周期。[实测统计及样本](../../../validation/R10/spray_collision_full_20261119.json)

在 479.7–599.8 仿真秒还采集了 S09 独立发布的 69 个局部地形明细帧。将未知轮廓格与地形网格同位置对齐，限制采样时差至 0.16 s 后，12 个格子中 8 个缺少合格地形行，4 个在近时刻的地形明细中测为零代价。缺少合格行可能源于没有回波、回波不足或多帧地面资格未建立。按原安装和车体遮罩投影到理想水平地面，718 个位置在现行俯角下都应有至少三束几何射线，单格中位数 71 束；保持相同射线数、将下限改为 −1.48 rad 后中位数为 63 束。该投影未模拟遮挡、噪声、跨帧运动和地形资格。下一轮传感器改造前应先记录近距实际点云和逐帧地形判断。原碰撞和未知格门限继续使用。[压缩后的原始快照](../../../validation/R10/spray_collision_snapshots_20261119.zip)

在 Linux 项目目录解包并重新计算。ZIP 中保留观测日志、原运行版本和任务配置，分析器还会验证仓库中的 Nav2 配置哈希。

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
snapshot_dir=$(mktemp -d)
python3 -m zipfile -e validation/R10/spray_collision_snapshots_20261119.zip "$snapshot_dir"
python3 optimization_v2/three_tasks/monitor/analyze_collision_observer.py \
  --observer "$snapshot_dir/spray_20261119_081218_417252" \
  --run "$snapshot_dir/run" --out /tmp/A1-private-costmap.json
```
