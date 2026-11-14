from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent;APP=ROOT/'optimization_v2/three_tasks'
status={}
for s in ['R08','R09','R10','R11','R12']:
 p=APP/'results'/s/'latest.json';status[s]=json.loads(p.read_text()).get('automated_status') if p.exists() else ('IN_PROGRESS（未封存）' if s in ['R09','R10','R11','R12'] and list((APP/'results'/s).glob('*_20*')) else 'NOT_RUN')
rows='\n'.join(f"|{s}|{status[s]}|[结果](docs/three_tasks/{s}/RESULTS.md) · [命令](docs/three_tasks/{s}/MANUAL.md) · [原理](docs/three_tasks/{s}/TECHNICAL.md) · [代码](docs/three_tasks/{s}/CODE.md)|" for s in status)
r12file=APP/'results/R12/latest.json'
r12=json.loads(r12file.read_text()).get('summary',{}) if r12file.exists() else {}
demo_note='\n\n**三任务演示录像组件已通过。** 点喷为原 A1 前三个点及工具故障补作；巡迹包括完整连接路和独立跟车停止/恢复；割草为原 B1 前五段及关刀倒车换行。完整 A1/B1 另行评分，不能由这些演示替代。\n' if r12.get('recording_components_passed') else ''
plots='\n'.join('!['+label+'](media/'+stage.lower()+'_'+name+')' for stage,name,label in [('R08','inspection.png','巡迹真实轨迹与交互案例'),('R09','mowing.png','割草真实轨迹与反馈面积')] if (APP/'results'/stage/name).exists())
text='''# ROS 2 农田自主作业机器人

同一张三维农田地图，三个分别运行的作业任务：**差速点喷、道路巡迹、阿克曼条带割草**。两个连续不规则工作区通过曲线道路连接，每个工作区通过工作量加权算法划分三个连通簇。车辆先使用与几何契约一致的轻量物理模型，采用保持碰撞、传感器位置和动力学契约的轻量材质方案，三任务可以分别进行真实观察相机录像。

项目把带误差的模拟传感器、定位与障碍感知、规划、可解释行为树、车辆控制和工具反馈连成可检查的闭环。目标是可落地、能解释失败和保留残余任务，不能用路径走完代替作业完成。喷洒与割草执行器是带反馈的抽象动作；机械臂、近景视觉与 VLA 可作为独立项目接入。

![原场地、任务与算法分簇](media/r03_task_map.png)

## 三个独立任务

|任务|车辆几何参考|主要导航与作业方式|
|---|---|---|
|点喷|Husky A200 尺寸参考的差速底盘|TSP 点序，导航到点、精停、抽象喷洒动作、计量反馈和残余重试|
|巡迹|Jackal 尺寸参考的差速底盘|FollowPath 跟踪原 88.48 m 连接路；前车跟随、停车恢复、观测绕行、受阻上报|
|割草|项目 Dino 前阿克曼底盘|Fields2Cover 条带，满足整车轮廓的 Reeds–Shepp/Dubins 地头连接；关闭工具后换向或转场|

这些是几何参考和实际仿真动力学，不宣称与对应商品整机等价。Dino 后轴最小转弯半径约 3 m，整车外侧转圈直径约 9.82 m；规划同时检查车体轮廓，不能只检查中心线。割草有效幅宽 1.6 m，正常前进指令 0.35 m/s、倒车上限 0.25 m/s。

## 技术结构

```mermaid
flowchart LR
  SIM[Gazebo 物理与带误差传感器] --> ODOM[RTAB 激光里程计 + 轮速/IMU EKF]
  SIM --> MAP[观测点云地图匹配 + GNSS 监督]
  SIM --> PER[地形/障碍点云 + 目标跟踪]
  ODOM --> NAV[Nav2 规划与 RPP / MPPI 控制]
  MAP --> NAV
  PER --> BT[可解释 BT 与有界恢复]
  BT --> NAV
  NAV --> GATE[唯一底盘出口与停车确认]
  GATE --> SIM
  BT --> TOOL[ToolTask 抽象工作动作]
  TOOL --> LEDGER[反馈账本 / 确认量 / 残余]
  LEDGER --> BT
```

- **环境与传感器：** Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic；三维激光、双目、IMU、轮速、GNSS 和实际近地扫描通道。R08/R09 使用独立版本的 128×2048 主雷达采样及会话专用 DDS 大点云缓冲；旧 R05 配置不变。时间延迟、噪声、偏差、丢点及安装外参均显式配置。
- **定位：** RTAB-Map 原生局部激光里程计与 `robot_localization` EKF；全局定位使用冻结的观测点云区域配准和独立 GNSS 创新监督。原生 RTAB 跨车辆高度的全局视觉匹配未通过，不能把当前结果称为该功能已验收。真值只进入隔离评估域。
- **通行代价：** PCL 点云、地面统计、坡度可观测性、低矮障碍和整车轮廓约束。未知区域保守处理，候选全局路径不等于已观测可通行路径。
- **离线规划：** 工作量加权 METIS 连通分簇；OR-Tools TSP / 2-opt 点服务顺序；Fields2Cover 与 OMPL 生成条带和满足转弯半径的地头段。
- **在线导航：** Nav2 Smac 规划器、RPP 跟踪，关刀末端姿态恢复使用有界 MPPI Ackermann 候选；阿克曼方向、速度和曲率由唯一控制出口约束。任务行为树使用实际 BehaviorTree.CPP，记录决策原因、输入证据、门限和未选择的候选动作。
- **动态交互：** 当前点云跟踪提供同向跟随、短时横穿保护与观测活动区绕行。使用 CV 未来协方差及有限时长的活动位置记忆，不宣称学会行人意图。历史 S15 学习型预测候选未达收益门槛，继续禁用。
- **有效作业：** 实际模拟工具电机/流量反馈驱动事务账本；面积按保守工具足迹求交、并集，重复经过不重复计数。原始任务分母与残余同时保存。R09/R10 的工具物理反馈采样为 50 Hz，独立桥接并按原采集时间配对，反馈与停车门限保持不变。

## 当前可验证范围

|阶段|状态|证据与学习材料|
|---|---|---|
'''+rows+demo_note+'\n\n'+plots+'''

R01–R06 已分别完成启动退出、几何、离线路径、动力学、短程定位和 BT 协议验收。R07 PASS 限定原 A1 前三个点及工具故障恢复；R08 必须包含完整连接路及三个交互案例；R09 组件案例限定原 B1 前五段；R10 另外检查完整 A1、连接路和 B1 路线及声明的故障案例。具体完成量以对应结果文件为准，已准备代码不等于已通过。R09 持续障碍用例在完成定位后放置实体障碍；启动即被大障碍遮挡的重定位尚未通过。割草搜索若找到替代路线但尚需覆盖重排，会保留残余并明确上报，不把候选路径记成已经绕行。

**原 S17/S18 继续保持 PARTIAL。** 不执行六十次正式矩阵，不把未运行的 A2/A3/B2/B3 推定为通过。条带段抵达与工具有效面积是两个指标；原 95% 标准保留，未确认面积不会清零。原 B 区可用 1120 m²，本版内侧任务 586.40 m²，其余 533.60 m² 保留为未计入的地头及边缘义务。

## 本机使用

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh status
bash optimization_v2/three_tasks/advance.sh R08 check
bash optimization_v2/three_tasks/advance.sh R08 open
bash optimization_v2/three_tasks/advance.sh R09 open
bash optimization_v2/three_tasks/advance.sh R10 open
```

实际复验命令、选择范围和停止条件见各阶段手工说明。每次运行一辆车，等有序退出后再启动下一个任务。原地图、正式任务与旧回执保留，新的配置、结果和学习材料使用独立版本。

```text
optimization_v2/three_tasks/       独立任务入口、配置版本和本机结果
optimization_v2/ws/src/task_platform/  任务、导航适配、BT 与评估源码
optimization_v2/ws/src/           定位、感知、工具和账本等共享包
docs/three_tasks/                分步任务书、命令、技术与核心代码说明
validation/                     发布的精简实测证据
media/                          图、截图及历史独立演示
```

本仓库同步源码、配置、说明和精简证据。完整冻结数据库、构建产物及逐帧记录仍绑定开发机，**尚不是 R12 的可移植发行包**。历史媒体按原阶段说明理解；本轮媒体统一以 r11_ / r12_ 前缀区分。R11 材质对照和 R12 实际录像的范围、完成状态与限制见当前回执。完整录像包含真实等待和失败，不等于完成全部作业。
'''
text+='\n## 本轮材质与录像\n\n'
for task in ['spray','inspection','mowing']:
 if (APP/'results/R11'/task/'material/preview.png').exists():text+='!['+task+' 材质实拍](media/r11_'+task+'_material.png)\n\n'
manifest=APP/'results/R12/media/manifest.json'
if manifest.exists():
 text+='[核心行为片段](media/r12_behavior_highlights.mp4) · [观看本轮精简录像](media/r12_three_tasks_short.mp4)（每段明确标注倍速；完整原片保留）。\n\n'
 for e in json.loads(manifest.read_text())['episodes']:
  video=ROOT/e['full_path'];label=e['case']+'：'+('本例通过' if e['task_passed'] else '未完成，保留实际停止记录')
  if video.stat().st_size<24_000_000:text+='- ['+label+'](media/r12_'+e['case']+'_full.mp4)\n'
  else:text+='- '+label+'；完整原片见本机 `'+e['full_path']+'`。\n'
text+='\n[本轮推进与后续命令](docs/three_tasks/本轮推进结果_R08-R12_20261114.md) · [R11 手工对照](docs/three_tasks/R11/MANUAL.md) · [R12 录像命令](docs/three_tasks/R12/MANUAL.md)\n'
(OUT/'README.md').write_text(text)
print(status)
