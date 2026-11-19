# B1 四挡整车复验

这次检验的是原 B1 第六条连接路。在两个相邻割草条带间，seed877 离线路径依次要求前进 1.415 m、倒车 2.196 m、前进 21.853 m、倒车 2.475 m。每个挡位单独提交给 Nav2 FollowPath，完成后要停在原计划终点并对正车头。刀盘全程关闭。全簇档案和作业面积分母继续使用各自的原版本。

## 本轮整车结果

**四个原挡 4/4 通过独立物理验收。** 任务用时 228.312 仿真秒，实际运行约 8 分 23 秒。在线估计对隔离物理轨迹的 XY 均方根误差为 2.62 cm。刀盘整程停止，账本新增作业量为零，原车体足迹全程位于批准区域，无异常接触，全部运行进程正常退出。

| 挡位 | 停车窗最大位置误差 | 停车窗最大航向误差 | 沿车头 / 向左的终点偏差 | 原挡行驶中最大横向偏离 |
| --- | ---: | ---: | ---: | ---: |
| 第 1 挡，前进 | 4.87 cm | 0.80° | −4.86 / +0.33 cm | 0.4 cm |
| 第 2 挡，倒车 | 4.90 cm | 0.55° | −3.64 / −3.27 cm | 4.9 cm |
| 第 3 挡，前进 | 9.38 cm | 0.63° | +4.56 / −8.20 cm | 9.4 cm |
| 第 4 挡，倒车 | 3.57 cm | 1.43° | +0.94 / −3.44 cm | 9.4 cm |

第三挡离 10 cm 门限只有约 0.62 cm 的余量，主要误差在横向。第四挡直接下发了包含全部 64 个原计划采样的倒挡 FollowPath。需要反复等待和重发命令的情况仍真实存在。四段共收到 1、5、14、3 次原 FollowPath，最终均保持各自的计划挡位。更换三个挡位时，在驱动轮零速状态分别采到 35、35、27 帧舵机调角，详见 `steering_audit.json`。

`physical_connector_report.json` 为最终逐挡评分，`parking_samples.json` 为同一停车窗内的真值、估计和闸状态。时间戳使用仿真钟。前三轮分开保存在 `initial/`、`capture_time/`、`stopped_steering/`，用于重现从冻结地图定位、地形配对、实际调舵到 BT 上下文的修复顺序。

## 手工复现

在安装过本项目 ROS 2 Jazzy 和 Gazebo Harmonic 的机器中打开一个新终端，进入项目根目录。第一次命令组只生成临时场景，检查地图、四挡路线、车辆模型和装载入口，终端打印四个目标的 `[x, y, yaw]` 与方向。

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
DRIVER=optimization_v2/ws/src/task_platform/scripts/qualify_b1_context.py
PY=optimization_v2/venvs/scene/bin/python
"$PY" "$DRIVER" prepare
```

第二组命令启动独占整车仿真，保留 Nav2、BT、安全闸、真实 Gazebo 底盘、刀盘反馈、摄像录制和独立物理评测。驾驶与评测使用隔离的 ROS 域。若另一台任务仍在本机运行，非阻塞会话锁会拒绝启动第二个 Gazebo。

```bash
"$PY" "$DRIVER" run --rtf 0.55 --seconds 390
```

`--rtf` 是仿真计划的实时倍率。`--seconds` 是该四段试验的仿真钟上限。一次粗初值在 8 s 后到达，以后所有定位来自传感器。每个会话独立保存，下列命令可在整车退出后重新读取对应回执；若已开始下一次独立运行，请将 `RUN` 指向希望查看的会话。

```bash
OUT=optimization_v2/three_tasks/results/experiments/b1_connector
RUN=$("$PY" -c 'import json,sys; print(json.load(open(sys.argv[1]))["path"])' "$OUT/latest.json")
"$PY" optimization_v2/ws/src/task_platform/scripts/score_b1_connector.py "$RUN"
ls -lh "$RUN"/video
```

可实时查阅 `$OUT/mowing_<本次时间>/live.json` 和 `mission_events.jsonl`。每一行 `SEGMENT_GOAL` 代表 Nav2 已接受实际动作。单次 `SEGMENT_ARRIVAL` 要和后续稳定停车窗、任务回执与独立物理评测一起读取。一次任务会尝试有界重规划和停车，遇到无法满足条件的情况会记录人工接管原因。

## 停车误差怎样读

每个目标用车体最终朝向作为 x 轴，向车体左侧为 y 轴。回执 `trajectory` 始终有四行。`physical` 是隔离评测域中的 Gazebo 位姿，`estimated` 是机器人在同一时刻可用的全局定位。两者共有以下含义。

| 字段 | 含义 |
| --- | --- |
| `longitudinal_m` | 沿目标车头方向的有符号误差 |
| `lateral_left_m` | 向目标车头左侧的有符号误差 |
| `overshoot_in_motion_direction_m` | 纵向误差乘以挡位方向。正值表示沿实际行进方向越过终点，倒车同样适用 |
| `yaw_error_signed_rad` | 实际车头减目标车头，折回 ±π；正值为逆时针 |
| `parking_window` | 至少 15 个定位观测跨 0.6 s，需要保持 10 cm、0.12 rad，逐次核对该时窗内物理位置、车速、角速度、刀盘转速与安全闸状态 |
| `state` | `NOT_STARTED` 待测，`ATTEMPTED` 已下发，`PARKING_OR_TRACKING_REJECTED` 任务结束但独立核验拒收，`PHYSICALLY_VERIFIED` 当挡验收通过 |

## 先解决定位的依据

旧局部启动会话采集的静止扫描中，激光点云对冻结地图的点面拟合出现了持续的平移偏差。有些拟合面位于带误差的地图边缘，旧算法仍让这些较远的点参与求解。新候选保留拟合集中距离最近的 60% 对应点，绝对残差超过 4 cm 后按 Huber 权重降权。未参与求解的另一半点继续独立检查表面重合率和 P95，七个初始航向假设必须相互收敛，原 GNSS 15 cm 与 NIS 9.21 检查继续生效。七个初值共享同一激光帧。姿态计算与选优均不读取真实车体位置。

| 真实历史扫描 | 原算法物理位置误差中位数 → 调整后 | 原算法 GNSS 15 cm 内 → 调整后 |
| --- | --- | --- |
| 原失败静止会话 14 个不同时间快照 | 13.23 → 4.77 cm | 11/14 → 14/14 |
| 另一次完整 B1 执行期间保留的 67 个快照 | 13.98 → 2.08 cm | 35/67 → 66/67 |

`registration_replay.json` 逐帧记录了冻结地图哈希、粗初值、七个偏角、选优规则和配对结果。保留集包含通过其他区域和进出原后段连接路时的移动扫描。这里的改善是历史回放结果；本次四挡实际驾驶评测单列于 `physical_connector_report.json`。

```bash
"$PY" optimization_v2/ws/src/task_platform/scripts/replay_connector_registration.py /tmp/b1-registration-check.json
```

该回放命令需要本机保留原始两组实际传感器数据；仓库里的小回执可直接阅读。


## 第一轮为什么停车

初次整车运行在 19 s 左右取得激光与 GNSS 一致的定位。第一挡的 Nav2 导航接受了 7 次实际请求，前进 0.697 m 后，任务达到原有重试上限，进入人工处理并有序停机。车体与刀盘在最终反馈中均已停止。四个停车点的通过数为 0。复查发现旧地形节点把激光帧与回调到达时的最近里程计强行比较，全程出现 34 条里程计时间不匹配记录，进而触发 BT 的感知等待和取消。

下一轮从原 Dino 专用 R09 地形 C++ 源码派生。在短有界缓冲中按激光采集时间选择里程计，超过原时间差或缺少支持时仍拒绝。保留 Dino 自身轮组、刀具和车身遮罩、近障细节，以及 B1 原 terrain.json 参数。复现首轮可改用 `qualify_b1_connector.py run --terrain arrival-order --rtf 0.7`。原失败小回执保存在本目录 `initial/`。


## 为什么新版本要停车调舵

第二次整车使用按采集时间配对的地形。第一挡停车达标。第二挡实际从前进切换到倒车时，Nav2 请求曲率 0.334 m⁻¹，前 9.9 cm 里物理曲率只有 0.051 m⁻¹。舵机还在换侧，底盘已经沿纵向移动。接近目标时，车的纵向误差已约 2 cm，车尾运动方向上的横偏仍达 20.5 cm。恢复动作后来抵达了终点，但有额外换挡并离开了原轨迹。第二轮独立只计 1/4，记录保留在 `capture_time/`。

第三版保留真实轴距（1.638 m）、轮距（1.8 m）、舵机速率（0.6 rad/s）和 3 s 跟踪故障限时，使用现有物理底盘的停车调舵能力。左右前轮任一与目标差超过 0.05 rad 时，驱动轮目标速度为零，舵机继续转到目标角度。Nav2 仍处理 FollowPath、速度和碰撞预测。独立 `r10::DirectedPathCurvature` 插件从 Reeds–Shepp 路径求参考曲率，同时校正横向和车头方向误差。符号由实测车体朝向与计划挡位决定。最后用原 Nav2 碰撞检查器核对实际输出曲率。内部控制目标 2.5 cm 与 0.04 rad 为停稳留出余量，项目验收门限继续使用 10 cm 与 0.12 rad。

连续重试仅在原路径上又测得 ≥0.8 m 净推进时可重新计数。判断使用带原始时间戳的有效估计位姿和 DRIVE 闸状态，避免把在同一位置反复晃动计为前进。控制器同时保持原连续失败上限，另有每段共 18 次中断上限、重试窗口限时及整次任务 390 s 总限时。安全停机和持续封堵的人工接管由原 BT、安全闸负责。

无须操控车辆，可以在第二个已加载 ROS 环境的终端保存实际前轮角日志。先在第一个终端开车，再将下面的 `RUN` 填成已输出的会话目录。在同一试验已经存在该采样文件时会拒绝覆盖。

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
RUN=$(ls -td optimization_v2/three_tasks/results/experiments/b1_connector/mowing_* | head -1)
/usr/bin/python3 optimization_v2/ws/src/task_platform/scripts/capture_b1_steering.py "$RUN"
```

仿真退出后，下面的命令核对实际装载的 SDF、车体配置、Nav2 插件、BT 哈希及换挡转向样本。它为每次换挡给出驱动轮零速期间的左右前轮实测角。

```bash
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/task_platform/scripts/audit_b1_steering.py "$RUN" /tmp/b1-steering-review.json
```


## 为什么换挡时要等待本段的 BT 证据

第三版真实走完了前三个原挡并停在各自终点。第三段末端的 REPLAN 来自第三段的 BT 上下文。第四段才进入 READY，新阶段立刻读到了上一段的指令，在 0.06 s 内转入备选路径。最终确实停到第四目标，停车窗独立位置误差 1.58 cm，航向 0.0181 rad，但期间额外换了挡，原单倒挡动作的接受数为 0。查阅 `stopped_steering/alternate_fourth.json` 可看真实替代行驶，`stopped_steering/physical_connector_report.json` 记录严格 3/4。

第四版将段号、挡位方向和实际激光采集时刻加入 BT 上下文。任务节点消费 EXECUTE 或 REPLAN 前，检查该决定属于本段，方向一致且感知采集发生在换段之后。HUMAN、STOP 和定位恢复仍立即生效。若原导航已派发，车已按计划经过终点，随后因 BT 取消或 Nav2 停车，任务节点可在原 15 样本、0.6 s、10 cm 和 0.12 rad 的停车窗内确认到达，下一挡仍需重新取证。每段现在独立检查仿真钟硬时限，整次仍限 390 s。

段号比较位于 `qualify_b1_context.py` 的 `disposition()`。运动控制实现为 `r10_connector_nav/controller.cpp` 的 `computeVelocityCommands()`。物理运动与在线定位间没有真值通道；真实车体输入只进入独立评分器 `score_b1_connector.py`。


## 完整 46 段版本留给整簇复验

`mission_46.json.gz` 解压后是本次成功整车会话实际加载的完整任务 JSON。`full_plan_preflight.json` 保存从固定项目源重建并用已有安全规划校验器复核的结果。完整 B1 连接长度共 227.653 m，45 个相邻段接缝连续，首末位姿与批准的 47 段版本相同。离线预计 1240.61 s（约 20 分 41 秒）；整簇原限时 1800 s。完整执行时间、覆盖面积与刀盘反馈仍待实测。

三个历史数字的含义不同。原 34 段版本包含 9 个不可缩减的割草工作义务，其目标并集 188.3918016929564 m²，整个簇的区域为 195.99438340335297 m²。经已批准的安全裁剪，8 个原义务对应 10 个可行工作片段，原第 16、29 段各拆出两个；第 33 段没有长度至少 1 m 的安全片段，继续保留残余。新 46 段版逐项保留所有 10 个执行片段、9 个原始义务及其分母，只将旧的 5 个刀盘关闭连接段换成新 4 个挡位段。

以下命令在任意包含本仓库的 Linux 系统上读取压缩包和列出完整 B1 段数。Python 标准库即可运行。

```bash
python3 - <<'PYTHON'
import gzip, json
from pathlib import Path
path = Path('validation/r10_continuation_20261119/b1/mission_46.json.gz')
with gzip.open(path, 'rt', encoding='utf-8') as stream:
    mission = json.load(stream)
b1 = next(x for x in mission['clusters'] if x['cluster'] == 'B1')
print('版本', mission['version'])
print('执行段', len(b1['segments']), '开启刀盘段', sum(s['tool_on'] for s in b1['segments']))
print('未获得安全片段的原义务', [s['segment'] for s in b1['omitted_original_work']])
PYTHON
```

在保留本次整车会话和原几何依赖的仿真主机，可用前述 `RUN` 离线复核整份文件。它复用既有 `late_connector.compare` 和 `safe_plan_r10.validate`，再比较实际装载任务与重建版本的每个字段。

```bash
"$PY" optimization_v2/ws/src/task_platform/scripts/audit_b1_full_plan.py "$RUN" \
  --output /tmp/b1-46-preflight.json --export /tmp/b1-46-mission.json
sha256sum /tmp/b1-46-mission.json
```

完整 JSON 的 SHA-256 为 `3ea2fd1565ee1313114e0bf2ed77032f65081eb30e60f270ed553404057f4df3`。当前专用驾驶入口从索引 22 起跑，只接受四个刀盘关闭的连接段。下一次正式完整 B1 需把这份版本绑定到索引 0 开始的整簇运行入口，运行全部 46 段并核查原 9 个作业义务的物理工具反馈、原面积分母和 1800 s 时限；旧入口的 47 段回执属于前版。
