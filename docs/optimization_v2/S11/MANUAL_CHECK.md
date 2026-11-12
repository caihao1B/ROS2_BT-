# S11 手工验收

本步完全离线，不需要密码或 Gazebo；一个普通终端即可。执行器加载隔离环境并核对 S02/S03/S10。下列 `--sim-seconds` 为统一接口保留字段，本步明确记录 N/A，不把参数值当作任务耗时。运行时间通常由前置哈希校验占较大部分；演示/测试均有独立墙钟超时。

## 1. 先看正式验收证据

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S11
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S11
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S11
```

应看到自动 PASS、人工 NOT_CHECKED，网页醒目标明 PROVISIONAL 和超预算。check 仅检查已有文件/哈希，不是重新运行算法。

浏览器操作：

1. 点击“查看未分簇整区”，确认两个连续不规则工作区；再点击“显示算法三色分簇”。边界应恰好分三色，无额外空白隔离带。
2. 将视野切到 A/B，切换基础、凹口、含洞；洞应保持白色禁区，不能被颜色填满。
3. 切换 METIS、K-means 和种子 41/42/43；边界来自真实计算结果，不是随机上色。不同种子可能收敛到相同结果。
4. 勾选“簇邻接边”，三簇关系应连通；点击簇，查看单元数、面积和初估负载。
5. A 区勾选“喷洒候选点”；红点表示包络筛查待核验，仍有簇归属。勾选边界带，确认边缘义务仍保留在分簇中。
6. 查看遗漏/重叠分别检查、算法对照与每簇分钟数。当前超 30 分钟是被报告的配置风险，不是隐藏或验收错误；S11 仅验几何分区，S12 才验路线工时。

## 2. 重新运行基础演示

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S11 --case partition_base --seed 41 --sim-seconds 120 --wall-timeout 300
```

产生基础两区的两算法对照，结束自动打开本轮网页。查看完整矩阵应使用正式报告或下一条变体演示。演示输出独立保存，不替换正式候选/回执。

## 3. 变体与不同种子演示

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S11 --case partition_variants --seed 41 --sim-seconds 180 --wall-timeout 420
```

生成三类几何、两区、两算法、种子 41/42/43 的 36 组结果。可将 seed 改为 44，生成 44/45/46；每个固定输入可复现。不需要 GPU。无桌面时加 `--headless`，读取 `optimization_v2/results/S11/last_demo.json` 的 report_path。

## 4. 完整自动复验

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S11 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1200
```

包括实际 C++ 编译、生产代码正常/失败测试、两种教学例、36 组划分及逐组重算、实际 Chrome 控件检查。全部通过才发布候选；中断/失败会更新 PARTIAL/FAIL，不拿旧 PASS 冒充本轮。

## 5. 查看候选与停止

```bash
python3 -m json.tool /home/ceihao/robot_project/optimization_v2/plans/candidate/partition_manifest.json
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S11 --print-path
```

清单应为 `status: PROVISIONAL`、`execution_allowed: false`、`k: 3`、未注册到 SLAM map。GeoJSON 是米制局部坐标，不是经纬度。`zone_a_clusters.geojson`、`zone_b_clusters.geojson` 各有三个 Feature；`*_units.geojson` 包含每个单元的唯一标签，`*_points.json` 包含点与待核验状态。

如需中途停止，在第二终端运行：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S11
```

仅停止本步拥有的计算进程。没有运行会话时返回 NOT_RUNNING。本步从未启动工具/车辆，不会停止其他 ROS 会话。人工检查完成情况请单独记录，不修改已哈希的自动回执。
