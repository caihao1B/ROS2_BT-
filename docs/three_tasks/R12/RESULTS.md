# R12 当前真实录像

整体 **PARTIAL**；三任务录像组件：PASS。

|案例|任务检查|录像秒|仿真终时|全部检查|
|---|---|---:|---:|---|
|spray|PASS|376.2|143.396|PASS|
|spray_full_A1|未通过|3069.0|1186.44|未通过|
|inspection|PASS|867.0|331.762|PASS|
|inspection_leader|PASS|316.4|119.812|PASS|
|inspection_blocked|PASS|256.2|74.112|PASS|
|mowing|原回执 PASS；当前版本复验未通过|489.2|185.392|未通过|
|mowing_full_B1|未通过|1129.6|327.93|未通过|
|mowing_startup_perception_loss|PASS|494.0|185.572|PASS|

三点点喷和五段割草是独立小规模演示，不代替完整 A1/B1。原片保留实际等待、重试、失败和终止；精简视频注明倍速。上轮通过的原片仍在原运行目录，未删除或改写。当前版本是否合格以当前回执为准。

## 当前未完成项

- R10 is not PASS
- R11 is not PASS

## 原片

- `spray`：`optimization_v2/three_tasks/results/R12/spray_20261114_194217_549485/video/full.mp4`。
- `spray_full_A1`：`optimization_v2/three_tasks/results/R12/spray_20261114_194849_404114/video/full.mp4`。
- `inspection`：`optimization_v2/three_tasks/results/R12/inspection_20261114_160433_163459/video/full.mp4`。
- `inspection_leader`：`optimization_v2/three_tasks/results/R12/inspection_20261114_161912_321381/video/full.mp4`。
- `inspection_blocked`：`optimization_v2/three_tasks/results/R12/inspection_20261114_181645_878210/video/full.mp4`。
- `mowing`：`optimization_v2/three_tasks/results/R12/mowing_20261114_173824_624692/video/full.mp4`。
- `mowing_full_B1`：`optimization_v2/three_tasks/results/R12/mowing_20261114_205558_700516/video/full.mp4`。
- `mowing_startup_perception_loss`：`optimization_v2/three_tasks/results/R12/mowing_20261114_211518_676200/video/full.mp4`。

本机播放：`xdg-open optimization_v2/three_tasks/results/R12/media/index.html`。录制和退出命令见 MANUAL.md。完整数据库和构建环境保留本机，GitHub 提供源码、配置、精简证据与媒体；尚不等于可移植安装包。
