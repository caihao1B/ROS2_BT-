# R12 真实录像与交付

**PARTIAL**。三任务录像组件：PASS。总体结论依据 R10/R11 和当前录像绑定，不以视频数量替代完整任务验收。

|案例|任务检查|录像时长/秒|仿真终时/秒|当前全部检查|
|---|---|---:|---:|---|
|spray|PASS|397.0|153.178|PASS|
|spray_full_A1|未通过|1315.8|509.824|未通过|
|inspection|PASS|867.0|331.762|PASS|
|inspection_leader|PASS|316.4|119.812|PASS|
|inspection_blocked|PASS|256.2|74.112|PASS|
|mowing|PASS|489.2|185.392|PASS|
|mowing_full_B1|未通过|1388.0|529.17|未通过|

完整录像包括启动、真实运动、等待、任务终止与停车，精简版明确标注每段倍速。小规模点喷仅包含原 A1 前三个点及一次实体工具 JAM/残余补作；割草小规模仅含原 B1 前五段。名称带 full_A1/full_B1 的案例才选择完整原任务，仍须看实际通过状态。

## 查看

```bash
cd /home/ceihao/robot_project
xdg-open optimization_v2/three_tasks/results/R12/media/index.html
bash optimization_v2/three_tasks/advance.sh R12 check
```

## 限制及后续

- R10 is not PASS
- R11 is not PASS
- R11 使用轻量原创面板贴图与材质，仍是简化几何；不是商品机器人精细外观或实物验证。
- 完整数据库、构建环境和逐帧记录保存在本机；GitHub 提供源码、配置、说明、精简证据和展示视频。
- 原 S17/S18 与未运行的簇保持原状态，不自动开始更大规模正式矩阵。

## 原片及验收出处

- `spray`：`optimization_v2/three_tasks/results/R12/spray_20261114_170025_915535/video/full.mp4`；同目录 `recording_acceptance.json` 和对应 `analysis_*.json`。
- `spray_full_A1`：`optimization_v2/three_tasks/results/R12/spray_20261114_170714_616253/video/full.mp4`；同目录 `recording_acceptance.json` 和对应 `analysis_*.json`。
- `inspection`：`optimization_v2/three_tasks/results/R12/inspection_20261114_160433_163459/video/full.mp4`；同目录 `recording_acceptance.json` 和对应 `analysis_*.json`。
- `inspection_leader`：`optimization_v2/three_tasks/results/R12/inspection_20261114_161912_321381/video/full.mp4`；同目录 `recording_acceptance.json` 和对应 `analysis_*.json`。
- `inspection_blocked`：`optimization_v2/three_tasks/results/R12/inspection_20261114_181645_878210/video/full.mp4`；同目录 `recording_acceptance.json` 和对应 `analysis_*.json`。
- `mowing`：`optimization_v2/three_tasks/results/R12/mowing_20261114_173824_624692/video/full.mp4`；同目录 `recording_acceptance.json` 和对应 `analysis_*.json`。
- `mowing_full_B1`：`optimization_v2/three_tasks/results/R12/mowing_20261114_174644_953306/video/full.mp4`；同目录 `recording_acceptance.json` 和对应 `analysis_*.json`。
