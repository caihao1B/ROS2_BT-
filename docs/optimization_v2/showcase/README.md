# 独立小规模演示与运动监视

本目录对应用户调整后的演示目标。原地图、S12/S17 正式任务、六十次矩阵和故障消融标准保留。演示结果单独记录，不改变 S17 的 PARTIAL 状态，不自动执行 S18。

## 本轮实际结果

独立演示总状态：**PARTIAL**。六簇作业通过；两种底盘的完整巡检尚未全部通过。S17 仍为 PARTIAL，S18 未执行。

| 任务 | 结果 | 实际反馈或进度 | 累计仿真秒（含恢复） |
|---|---|---|---:|
| A1 | PASS | 2/2 点 | 168.0 |
| A2 | PASS | 2/2 点 | 179.4 |
| A3 | PASS | 2/2 点 | 145.1 |
| B1 | PASS | 98.22% | 586.0 |
| B2 | PASS | 98.72% | 578.6 |
| B3 | PASS | 97.22% | 1248.0 |
| road_diff | FAIL | 13/15 子段；完整巡检未完成 | 1447.9 |
| road_ackermann | PARTIAL | 13/21 子段；完整巡检未完成 | 925.7 |

全部计入本轮预算的尝试共 179.24 分钟墙钟、91.63 分钟仿真时钟，包含失败尝试。所有作业簇累计仿真时间均未超过 1800 秒。

差速巡检在末端出现绝对航向来源过期，航向 P95 超过原门限，结果为 FAIL。阿克曼已执行部分安全、定位合格，但未在本轮墙钟余量内完成全路线，结果为 PARTIAL；时间上限停止没有生成新的终态监视停车快照，网页明确标记最后快照不能当作最终停车证明。运行收尾已关闭工具并清理所有本次仿真进程。

录像为真实运行的章节拼接：完整版本 **2:42:31**，精简版本 **4:14**，后者包含标注的 53 倍速总览和实速关键事件。B3 补作和差速巡检的显式恢复保留前后两段，未用重启清零进度或时间。

[GitHub 精简视频](https://github.com/caihao1B/ROS2_BT-/blob/main/media/showcase_short.mp4) · [GitHub 完整录像](https://github.com/caihao1B/ROS2_BT-/blob/main/media/full_demo.mp4)

本机查看：

```bash
cd /home/ceihao/robot_project
xdg-open optimization_v2/showcase/media/showcase_short.mp4
xdg-open optimization_v2/showcase/media/full_demo.mp4
```

## 当前可核查的运动监视功能

监视网页仅监听本机，数据超过 5 秒墙钟没有更新时会明确提示连接或执行状态未知，不能继续显示为正常运行。底盘原有租约看门狗仍独立执行停车保护。

机器人先按原行为树执行有限等待和重规划。持续没有位置进展、耗尽恢复次数、定位/感知无法恢复或工具反馈不确定时，由原任务控制者取消导航、停止工具、保存剩余任务。页面只显示状态，不发布运动指令，没有强制继续按钮。

- 同一导航义务连续 20 秒位置变化不足 0.10 m，且航向变化不足 0.05 rad：显示等待/重新规划及诊断。
- 连续 60 秒无位置或航向进展：锁存 `SUPERVISOR_NO_PROGRESS_60S_MANUAL_CHECK`，通过原有 BT 安全停止链执行停车。差速车正常原地对准航向属于运动进展。
- 差速点服务的正常驻留不计入停滞；阿克曼割草不豁免停滞判断。
- 当前地图或定位不满足条件时，不给出“通道封闭”结论。
- 新鲜准入栅格的宽松八邻域点搜索仍不连通，才标记 `OBSERVED_GRID_DISCONNECTED`。这只表示当前观测栅格不连通，不证明真实世界永久无路。
- 点可达而规划失败时，标记 `POINT_CONNECTED_KINEMATICS_UNRESOLVED`。车辆轮廓、转弯半径和搜索预算仍可能导致失败。
- 停车确认同时要求新鲜的底盘实测静止与工具关闭反馈；报警本身不等于停车成功。

真实试运行 `showcase_repair_ackermann_20261113_181900` 在约 151.07 秒生成了人工接管记录：无进展 60.07 秒，停车与工具关闭已确认。独立评估无碰撞、定位合格，导航目标仍未完成，结果为 PARTIAL。此记录是当次源码快照的证据；后续版本仍需各自运行验收。

## 小规模任务

地图保持原有两个不规则作业区和曲线连接路。独立演示对工作义务缩放，运动授权区不缩小，也不更改正式分母。

| 作业 | 独立演示义务 | 分区方式 |
|---|---:|---|
| 差速喷洒 | 连续 27.0336 m² 区域中的 6 个点 | 工作量加权 METIS + 连通性修复，3 簇，每簇 2 点 |
| 阿克曼割草 | 连续约 4.104 m² 区域 | 同样计算 3 个连通簇，保留各簇完整面积分母 |
| 连接路巡检 | 原有完整连接路线 | 工具始终关闭；实际端点和车体授权边界独立核验 |

任务模型的预计工作时间约 1970.9 仿真秒，不含两次连接路巡检、启动和现场等待。每簇硬上限仍为 1800 仿真秒；正式演示批次累计墙钟预算为 10800 秒。修复探针必须启动前显式声明 `--probe`，不允许事后把慢的正式运行改成探针以规避预算。预计工时不是实际成功证据。

`optimization_v2/showcase/v1/plan_overview.png` 显示原地图、自动分簇、点目标和条带。`mission_plan.json`、`manifest.json` 与每次运行的 `spec.json` 绑定任务版本。计划尚未运行时状态为 `NOT_RUN`。

## 手工命令

先进入项目目录：

```bash
cd /home/ceihao/robot_project
optimization_v2/venvs/scene/bin/pip install -r optimization_v2/config/showcase_requirements.txt
bash optimization_v2/scripts/build_showcase.sh  # 只在没有仿真运行时构建
bash optimization_v2/showcase.sh check
bash optimization_v2/showcase.sh status
```

检查命令验证停滞/驻留区分、人工接管锁存、旧反馈不能确认停车、栅格诊断边界，以及移动历史不能污染静态记忆等约束。它不替代真实任务运行。

逐个运行簇；同一时间只运行一个仿真实例：

```bash
bash optimization_v2/showcase.sh run A1
bash optimization_v2/showcase.sh run A2
bash optimization_v2/showcase.sh run A3
bash optimization_v2/showcase.sh run B1
bash optimization_v2/showcase.sh run B2
bash optimization_v2/showcase.sh run B3 --seconds 1800
```

分别运行两种底盘的原连接路巡检：

```bash
bash optimization_v2/showcase.sh inspect diff
bash optimization_v2/showcase.sh inspect ackermann
```

在另一个终端打开当前运行监视页：

```bash
cd /home/ceihao/robot_project
bash optimization_v2/showcase.sh dashboard
# 浏览器打开 http://127.0.0.1:8768
```

查看已验证的受阻记录：

```bash
bash optimization_v2/showcase.sh dashboard \
  optimization_v2/results/S17/showcase_repair_ackermann_20261113_181900
```

检查运行目录中的这些文件：

- `client/motion_monitor.json`：实时状态、当前目标、BT 原因、重试次数和停车反馈。
- `client/manual_handoff.json`：需要人工检查的锁存记录；没有此文件不等于任务成功。
- `client/mission.json`、`residual.geojson`：工具反馈确认的作业量、未完成点和残余面积。
- `showcase_result.json`：独立演示实际结果；PASS 需要完成、安全、定位、任务分母和版本一致。
- `video/full.mp4`、`video/frames.jsonl`：真实 Gazebo 观察相机的完整录像和帧时间线。相机只进入隔离的评估域 120，不进入定位、感知或控制。
- `cleanup14.json`：本次拥有进程的清理情况。

完整展示视频按任务章节拼接，各章来自独立真实运行；两种固定底盘分别运行。这不是单车不中断地完成跨区全部作业的证明。精简版在画面中明确标注播放倍速，原始仿真时间仍可见。

B3 的复现命令预留原有 1800 秒簇上限，供真实残余补作使用，不改变覆盖率或安全门槛。本轮预算文件不会自动清零，余额不足时录制命令会拒绝启动。之后单独进行手工开发验证，可在启动前明确加 `--probe`，例如 `bash optimization_v2/showcase.sh run B3 --seconds 1800 --probe`；这类运行单独标记，不补记为本轮正式录制成绩。

不要直接编辑分母、删除残余账本或把 PARTIAL 改为 PASS。解决障碍后应显式启动经版本检查的恢复/重跑；本页面不提供无人确认自动解除停车。

如需在本轮剩余墙钟预算内尝试阿克曼完整巡检，可显式使用 `bash optimization_v2/showcase.sh inspect ackermann --bounded-attempt`。此参数只允许在保守工时估计超出余量时仍作有界尝试；路线、终点、工具关闭、安全与完成标准均不变，墙钟上限也不增加。它不保证完成，超时后保留实际 PARTIAL/FAIL，不能补记成功。普通运行仍会在预算估计不足时拒绝启动。

本轮 B3 在初次三条工作带后确认覆盖 92.42%，补作过程中触发 `BUDGET_STOP` 并确认停车。SQLite 中的实际足迹并集已与任务账本逐项核对。检查后的显式恢复示例为：

```bash
bash optimization_v2/showcase.sh run B3 --seconds 600 \
  --resume-from optimization_v2/results/S17/showcase_B3_work_20261113_194812
```

恢复使用最后一次传感器估计位姿、原任务版本和已确认账本，不使用仿真真值。`showcase_resume_proof.json`、`resume_source.json` 和 `client/restored17.json` 记录来源哈希和恢复结果。前段的 PARTIAL 保留；恢复是否完成由后续实测决定。验收和完整视频必须同时包含初次运行与恢复运行，累计时间不能因重启清零。

## 核心实现与算法

`ws/src/showcase_demo/scripts/motion_monitor.py` 用估计定位的滑动轨迹窗口判断停滞，用栅格连通域提供有限诊断。它只设置任务控制者的安全标志，底盘/工具动作出口仍各有唯一所有者。`dashboard.py` 提供仅监听本机的只读 HTTP 页面。

`plan_showcase.py` 复用 S11 的加权图划分。单元权重近似表示行驶、喷洒驻留、转弯和地形工时，METIS 降低跨簇边界代价，再通过连通增长和边界单元交换修复连通与工作量平衡。喷洒路径比较 NN + 2-opt 与 OR-Tools TSP；割草用 Fields2Cover 生成条带，用带车体安全裕量的实测地图检验有向 Dubins 连线，再在有限方向图上用 Held–Karp 选择闭合路线。离线覆盖不直接记入工具账本。

`src/ReactiveActors.cc` 是独立演示场景插件。保留原演员、运动意图和物理尺寸，对靠近车辆的演员加入局部停让；停让时冻结其意图时间，避免恢复时追赶跳跃。它读取仿真车辆位置仅用于演员自身行为，该信息不传给机器人算法。这是简单局部停让模型，不是经过商用验证的人群模型。原不避让场景及其失败记录继续保留。

`dynamic_terrain_memory.py` 将已确认移动目标的观测历史从补充静态地形记忆中排除。当前雷达占用和冻结地图占用始终优先，未知数据不能凭空清除静态障碍。历史运动轨迹作为软代价影响路线偏好，当前障碍仍是硬约束。此修改只在独立演示适配层生效，需要真实通行复验。

本次录制的 `S17_PREDICTION=disabled`：导航采用当前观测和动态历史，未启用预测附加代价或经验缓存对照。S15/S16 的既有代码与证据保留，本次演示不新增其预测准确率或算力收益结论。演员停让与机器人自身的等待/重规划分别记录，不能混为同一算法的效果。

`showcase_binding.py` 检查父地图/正式计划版本以及演示任务哈希，防止运行期间偷换地图或任务；`evaluate_showcase.py` 复用原有独立真值评估，但明确使用演示自身已声明的义务分母。Gazebo 真值仅在任务后用于评估，不反馈到在线导航。

`residual_showcase.py` 修复了补作连接段只检查运动区域、未排除观测障碍的问题。它保存已准入的实时栅格快照，禁止未知格和高占用格，先检查六类前进 Dubins 连接；直接连接不可用时，在有限的 1 m 位置格点和 8 个航向上搜索经一个中间位姿的连接，30 秒内无结果就返回不可补作。整个车体及安全裕量必须通过栅格检查，再检查原运动区域和车辆运动约束。合格连接以分段 `FollowPath` 执行，实时碰撞检查、定位准入和工具联锁仍有效。这是有限候选搜索，不声称找到了全局最优路径，也不把找不到候选等同于证明永久无路。

补作只重复原任务中仍有可确认收益的工作带，不更改目标区域或完成门槛。B3 的第一次恢复确实触发了新版 60 秒无进展停车；该失败保留在开发记录和耗时预算中。修正后的第二次显式恢复 `showcase_B3_work_20261113_203412` 实测通过，覆盖率由 92.42% 增至 97.22%，恢复链、安全和定位检查均通过。补作后在工作带末端安全停车；原计划的返回停车点动作已在补作之前执行，不宣称补作后又返回了一次。

演示适配层将差速 DWB 的角速度采样从 21 档增至 41 档，并将角度离散步长细化到 0.01 rad，便于接近严格停靠航向。目标位置/航向容差仍为 0.025 m / 0.025 rad，工具准入和停车门限未放宽。启动前收到过期 BT 上下文时，只允许在原有 90 秒初始化期限内保持停车等待；初始化完成后的过期停止规则不变。

## 巡检断点与恢复计数

连接路较长，独立演示把恢复额度限定在当前尚未完成的子段：只有导航成功并保存一个新的已完成子段，才开始下一窗口的 12 次观测恢复、2 次重新规划额度。`inspection_recovery.py` 把各窗口用量和总用量写入 `inspection_recovery_window_closed` 事件。失败、时间流逝、重复完成旧子段都不能清零。原 60 秒无进展报警、传感器时效、定位和工具停车门限，以及整次运行的时间上限仍有效。此策略只属于独立演示适配层，正式 S17 策略未改动。

检查后显式恢复差速巡检的示例：

```bash
cd /home/ceihao/robot_project
bash optimization_v2/showcase.sh inspect diff --seconds 900 \
  --resume-from optimization_v2/results/S17/showcase_A1_inspection_20261113_204730
```

恢复校验原路线、完成事件、剩余后缀、源文件哈希和已确认停车；使用最后一次传感器估计作为重启先验。重启不重置累计计时或删除原失败记录。完整巡检还必须在离线评估中检查两次实际轨迹按顺序到达原始端点，且车体始终位于授权区域。视频按实际运行片段拼接，不填补停机间的虚构运动。

检查最近运行的报警和作业量，避免直接输出很大的路径 JSON：

```bash
python3 - <<'PY_CHECK'
import json
from pathlib import Path
run = Path(json.loads(Path('optimization_v2/showcase/current.json').read_text())['run'])
for name, keys in [
    ('client/motion_monitor.json', ['state','reason','bt_reason','requires_manual','route_diagnosis','stop_confirmed']),
    ('client/mission.json', ['state','reason','confirmed_points','required_points','coverage_fraction','uncertainty_pending']),
    ('showcase_result.json', ['status','complete','safety','localization_pass','execution_episodes'])
]:
    p = run / name
    if p.exists():
        d = json.loads(p.read_text())
        print(name, json.dumps({k:d.get(k) for k in keys}, ensure_ascii=False, indent=2))
PY_CHECK
```

`MANUAL_CHECK_REQUIRED` 表示需要检查，不等于已证明永久无路。先核对 `stop_confirmed`，再检查障碍、定位和感知原因；不可只清除报警后继续。真正无法解决时保留 PARTIAL 和剩余任务，停止本轮任务供人工处理。

## 已测出的巡检限制

差速完整路试 `showcase_A1_inspection_20261113_212812` 从已核验断点继续，完成进度到 13/15 个子段，随后停止。跨两次实际轨迹检查没有车体越界；本次位置误差 P95 为 0.03876 m，但航向误差 P95 为 0.20357 rad，超过 0.05 rad 要求。末端出现 `ABSOLUTE_HEADING_EXPIRED`，观测恢复等待超过原有 45 秒限制，BT 报 `BLOCK_RETRY_EXHAUSTED`，车辆与工具停止确认。结果是 FAIL，不是巡检完成，也没有证据证明障碍导致永久无路。

这说明仅调整恢复次数不能解决长路段的绝对航向来源失效。该失败断点不满足 `showcase_resume.py` 的定位合格前提，禁止直接恢复。后续应先核查绝对航向观测、地图匹配质量与时间戳，再在同一路段做有界定位复验；不能放宽 0.05 rad 门槛或把定位无效解释成道路封闭。

巡检默认起点覆盖恢复先验的适配问题已修正，并用实际生成的 Gazebo SDF 检查位置和航向一致。Python 模块遮蔽也已改为唯一模块名并检查实际导入文件。上述失败录像保留，测试通过不改写旧运行的 FAIL。

## 阶段边界

六十次正式矩阵和全部故障消融暂缓。演示通过单独记录。S17 原标准未满足时保持 PARTIAL，因此暂不能据此宣布 S17 PASS 或进入 S18。

原始 B1 录像中，分段转场进入下一子段后，监视叠层曾继续显示首个子段的橙色目标框；真实导航动作目标以 `client/events.jsonl` 为准。这一只读显示问题已在后续运行中修正为跟随活动动作，未改动路径、停靠门限或进度判定。录像保留原始画面，没有事后替换为修正界面。
