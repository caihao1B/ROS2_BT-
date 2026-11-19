2026 年 11 月 19 日的三任务版本已经在同一台 Ubuntu 24.04 主机的独立克隆中还原冻结地图、检查六簇路线并重建原生接口。新机器运行 Gazebo 前还需完成本机的退出库与前置阶段验证。[三任务复现检查](docs/three_tasks/复现检查.md) · [本轮运行与残余记录](docs/three_tasks/收尾复验_20261119.md)

> 2026-11-13 检查点：S17 源码已同步，完整 S17 资格资产仍在开发机，尚不能从本仓库直接复现 S17 全部证据。恢复说明见 docs/optimization_v2/S17/PAUSE_RESUME_20261113.md。

# 构建、资产恢复与演示

本仓库是正在分阶段验收的开发工程。`validation/development_status.json` 记录开发机证据，不能作为新机器的 PASS 回执。S18 已完成部分移植与教学验证，完整新机复现和正式六簇验收仍待完成；原 S01 的 Dino 迁移基线还依赖未随本仓库分发的旧资产。新轻量底盘、场地、源码、地图与规划输入已单独收录。

## 平台与依赖

使用 Ubuntu 24.04 amd64、ROS 2 Jazzy、Gazebo Harmonic。这个组合符合 [Gazebo 官方兼容表](https://gazebosim.org/docs/harmonic/ros_installation/)。先按 [ROS Jazzy 官方安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 配置 ROS 软件源并安装 Jazzy；不要直接在其他 ROS 发行版上混装这些包。

在已经配置好 Jazzy 软件源的机器上，主要依赖如下。命令在普通终端执行；sudo 密码只在本机终端输入。

```bash
sudo apt-get update
sudo apt-get install --no-remove \
  ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-rtabmap-ros \
  ros-jazzy-fields2cover ros-jazzy-ortools-vendor ros-jazzy-ompl \
  ros-jazzy-behaviortree-cpp ros-jazzy-pcl-conversions \
  ros-jazzy-sensor-msgs-py ros-jazzy-rosbag2-py \
  python3-colcon-common-extensions python3-venv \
  python3-numpy python3-scipy python3-yaml python3-opencv \
  python3-matplotlib python3-pil python3-pytest fonts-noto-cjk \
  libmetis-dev libgdal-dev libeigen3-dev libpcl-dev \
  nlohmann-json3-dev libprotobuf-dev build-essential cmake
```

以上是依赖集合，不是将任意最新版视为已验收。开发机的精确版本分别保存在 `optimization_v2/config/dependency_lock.yaml` 和各 `sNN_dependency_lock.json`。例如 mission_bt 的 CMake 当前要求 BehaviorTree.CPP **4.10.0 EXACT**；Fields2Cover 使用 **2.1.1** 接口。若发行版仓库提供不同版本，应先完成兼容性复验，不能删除版本检查来获得 PASS。

## 克隆与恢复

```bash
git clone https://github.com/caihao1B/ROS2_BT-.git
cd ROS2_BT-
python3 scripts/restore_assets.py
```

地图数据库超过 GitHub 普通单文件上限，因此大输入存为 `assets_bundle/` 中的无损分片。恢复脚本逐片验证 SHA-256，再检查整个归档及每个输出文件。它拒绝路径越界、符号链接与覆盖已修改文件。网络下载不发生在恢复过程中；所需分片与源码在同一提交中。

`SOURCE_INVENTORY.json` 记录原 S14–S17 快照的逻辑输入文件。三任务当前版本的构建、地图和输入绑定见[独立克隆复验](docs/three_tasks/复现检查.md)，`assets_bundle/manifest.json` 记录分片及原始文件大小、哈希。不要直接把恢复后的大文件再次 `git add -f`；`.gitignore` 已排除这些展开副本。

## Python 环境与编译

规划和报告使用可读取系统 ROS 包的虚拟环境。几何计算使用 Shapely 2.1.2，R12 录像使用 imageio-ffmpeg 0.8.1 与 Noto CJK 字体。环境内的 pip 可直接以普通用户运行。

```bash
python3 -m venv --system-site-packages optimization_v2/venvs/scene
optimization_v2/venvs/scene/bin/python -m pip install 'shapely==2.1.2' 'imageio-ffmpeg==0.8.1'
source optimization_v2/env.sh
cd optimization_v2/ws
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ../..
source optimization_v2/env.sh
```

学习型预测使用自己的 CPU 隔离环境，不能把模型依赖安装进 ROS 的系统 Python。以下脚本安装固定的 Python 3.12 / Linux x86_64 wheel，并校验逐个 SHA-256；不可与仿真并行安装。

```bash
bash optimization_v2/scripts/install_s15_dependencies.sh
```

官方 Trajectron++ 权重与 MIT 许可随受控资产分发，原始文件哈希和兼容性适配见 `optimization_v2/config/model_lock.yaml`。不需要下载 ETH 原始训练数据或大型 nuScenes 数据集。

## 不依赖开发机回执的场景预览

下面入口调用实际场景生成器与底盘模型，在 `preview_runs/` 生成临时世界，修正资源到当前克隆路径；不会修改冻结地图，也不会生成验收回执。

```bash
source optimization_v2/env.sh
optimization_v2/venvs/scene/bin/python scripts/preview_world.py --vehicle diff --seconds 90
optimization_v2/venvs/scene/bin/python scripts/preview_world.py --vehicle ackermann --seconds 90
```

两条命令顺序运行。可看到完整场地、连接路、场景运动物体和静止车辆。该入口是场景/模型预览，未执行导航、喷洒或割草；物体使用模型规定的脚本运动，不能视为商用行人行为模拟。`--seconds` 是预览的现实秒数。Ctrl+C 或关闭窗口后只清理本次预览进程。无桌面可加 `--headless`。

## 离线运行证据

恢复资产后，可直接打开 `validation/s14/index.html` 查看发布时的真实案例回放。时间轴为原记录抽样，状态切换与首尾记录保留，任务分母、反馈完成量和失败原因不改写。`export_manifest.json` 标明源运行与原始结果哈希；这是开发机证据展示，不是新机器的验收回执。

```bash
xdg-open validation/s14/index.html
xdg-open validation/s15/index.html
```

S15 归档包含实际多模态输出、逐窗口评分和 15 个闭环/故障实验。模型是否被选为默认使用，以报告中的 selection 为准；实验接口允许显式测试未启用候选，不能据此声称它已通过收益门槛。

## 正式分步验收

```bash
bash optimization_v2/stepctl.sh list
bash optimization_v2/stepctl.sh status
bash optimization_v2/stepctl.sh prompt S14
```

`test` 执行仿真和检查；`demo` 生成独立演示；`check` 只核对已有证据链。新克隆没有 `results/SNN/latest.json`，因此 `check` 显示 NOT_RUN 是预期行为。不要复制开发机回执、编辑哈希或跳过前置检查。现有严格验收链包含历史 S01 基线，尚不能宣称此源码快照在任意新机上一条命令即可得到 S14/S15 PASS。开发环境的阶段运行方法、详细前置与当前限制见各阶段的 `MANUAL_CHECK.md` 和 `RESULTS.md`。

目录和生成的 SDF 中可能包含开发机历史绝对路径；预览入口重新生成自己的资源路径。冻结地图与计划不批量字符串替换，因为这种修改会改变版本绑定。正式换机的路径、版本及地图重绑定需由后续移植验收负责。
