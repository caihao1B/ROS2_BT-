# A1 连续点喷精停与工时

本轮短程实验选取原 A1 顺序中第 22–25 个点。四点依次向西，相隔 3 m，场景在第一个目标处冷启动。该设计能比较停车与连续点间导航，也意味着不能把第一点的短工时当成 56 点的平均工时。原计划的 56 个目标、每点 3 单位剂量、地形与通行门限均保留。场景记录使用物理工具、唯一底盘与工具请求出口、独立剂量计和 Gazebo 摄像机。

## 同条件实测

| 原顺序四点 | 实际完成 | 结束仿真秒 | 已见时间错配 | BT 感知取消 | 实际喷洒中最大物理误差 |
|---|---:|---:|---:|---:|---:|
| 本轮冷启动原版 | 3/4 | 156.448 | 18 | 15 | 49.9 mm |
| 第一候选 | 4/4 | 128.164 | 0 | 0 | 36.5 mm |
| 最终候选，独立冷启动 | 4/4 | 120.242 | 0 | 0 | 37.1 mm |

两次候选均通过独立物理剂量、SQLite 账本、重投不增流、唯一执行出口、全程录像和最终已停检查。只对原、候选都完成的前三点配对，三点合计从第一次导航请求到成功反馈的时间分别节省 20.620 和 20.904 仿真秒。最终候选还记录到 17 个有意等待后成功配对的不同源点云，实际最大采集时间差 50 ms。最终候选的全局重新接近分支在这四点没有触发。两次候选构建物不同，第二次含显式时钟回退处理、队列观测和上述后备动作。两个短程运行支持同范围的结论，不能代替全簇实测。三次运行均保留原逻辑身份、完整记录和各自结果，误差的最大值只在实际有流量的时间计算。

## 原顺序 56 点整车选测

另一次从正式起点冷启动的单次运行选入全部 56 点，在 978.792 仿真秒内连续确认 26 点，其中包括原第 22–25 点四个难点。第 27 点导航中达到本轮墙钟截止，执行器收到 SIGINT 后取消导航、确认车辆与工具均已停稳。无新增未完成的工具账项，剩余 30 个原目标留待复验。全部 26 个实际作业点都通过原物理配准/剂量门限，作业中最大物理平面偏差 55.73 mm，定位 RMSE 为 48.30 mm。摄像机 7,959 帧且连续性通过，导航图与工具执行出口检查通过。整簇判定仍为 PARTIAL。

从首次发出 Nav2 目标到原第 22 点确认，本次为 783.436 仿真秒，同日历史原序为 869.566 秒。历史运行设置 RTF 0.50，本次 0.85；这组 86.130 秒差值是现场记录，未控制调度差异。本次全程识别到 2,682 个经过等待再正确配对的源点云，仍有 9 个被观察到的时间错配和 9 个原 BT 感知取消。公开的原始评分见 `full_selection.json`，历史逐里程碑、原位关停与逐点物理核验见 `full-56.json`。

## 为什么改动

原程序收到一帧点云就立即在已经投递的里程计消息中找最近样本。点云先到而对应里程计仍排队时，原版会判定时差超过 50 ms，触发地形无效、BT 取消导航或喷洒。在本轮原版中，第三个实际确认的点之后，第四站的 Nav2 已报告到达，停稳后的全局目标距离却约 0.165 m，比局部纠偏允许的 0.15 m 更远。现场保留余量并停机请求人工处理。

最终候选对一个尚待配对的点云最多等原配对时差 50 ms，收到里程计后重跑原始时间、坐标系和几何判定。等待不产生新的有效地形。队列满时及时结算旧帧，过期输出无效，新版本对等待期间的仿真时钟回退锁存故障。每次原位判定同时积累机械停车时长和定位观测时长。观测只收双停确认之后获取的不同真实定位时间戳。定位代次更换、间隙、无效观测或运动都会重置窗口。门限仍为不少于 20 个观测、跨度至少 1.8 s、最大间隙 0.15 s、中位数误差最多 35 mm、末样本最多 45 mm、散布 P95 最多 15 mm，另需机械双停至少 1.5 s。

对接近目标时连续处在原 25 mm 导航半径内的低速观测，候选可先请求停车，再重新做完整全局判定。从未把该请求作为已到达或已喷洒的证据。若当前可信稳定偏移在 0.15–0.25 m，可重新向 Nav2 发送一次原目标，并计入同一轮最多两次实际纠偏。所有成功反馈仍来自原 ToolTask、账本与物理剂量。缺少可行运动或观测会保持残余。

## 手工执行

在已安装和搭建原 ROS 2 Jazzy 工作区的主目录运行。下列两项是独立测试，二者顺序执行以共享同一个仿真进程锁。

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
py=optimization_v2/venvs/scene/bin/python
scripts=optimization_v2/ws/src/task_platform/scripts

# 构建独立候选执行器
"$py" "$scripts/spray_precision_session_r10.py" prepare

# 重现四个困难目标。索引从 0 开始，21 对应第 22 个目标
"$py" "$scripts/spray_precision_session_r10.py" run --first-index 21 --count 4 --seconds 300 --rtf .55

# 独立选择全 56 个原序目标和原 1800 仿真秒预算
"$py" "$scripts/spray_precision_session_r10.py" run --first-index 0 --count 56 --seconds 1800 --rtf .55
```

每次自然场景运行写入新的 `optimization_v2/three_tasks/results/experiments/spray_precision_session/spray_defer_*` 文件夹。打开 `mission_result.json` 看选点数、已确认列表、失败和剩余，`video/full.mp4` 记录真实场景，`mission_events.jsonl` 记录各次停车与重试，`risk/risk.jsonl` 的 `terrain_health` 可核验点云源时间和配对差。下列命令可重新算量化结果，生成的 `/tmp/a1-comparison.json` 为本机临时报告。

```bash
base=optimization_v2/three_tasks/results/experiments
"$py" "$scripts/analyze_spray_precision_r10.py" \
  --baseline "$base/spray_operator_handoff/spray_defer_20261119_121141_482729" \
  --candidate "$base/spray_precision/spray_defer_20261119_121833_942724" \
  --candidate "$base/spray_precision_session/spray_defer_20261119_122402_750899" \
  --out /tmp/a1-comparison.json
"$py" "$scripts/test_spray_precision_r10.py"
"$py" "$scripts/replay_spray_precision_r10.py" \
  --handoff "$base/spray_operator_handoff/spray_defer_20261119_121141_482729" \
  --candidate "$base/spray_precision_session/spray_defer_20261119_122402_750899" \
  --out /tmp/a1-negative-replay.json
```

只下载了小型验收凭证的设备可直接打开同目录 `comparison.json`。重新计算需要对应原始日志、Gazebo 真值与物理剂量文件。录像连续性按原摄像机门限检查，目标几何与定位真值只在离线评分使用。

离线负例见 `negative-replay.json`。从已封存的实际人工接管回执回放，阀仍打开、非零流量或转速、过龄、跨代次、底盘未停、缺停机确认等八项改动都不能通过停机检查。64 个种子模拟了采集抖动、运动和 32 cm 定位跳变，在稳定观测重新建立前不得准入。对封存执行器中的实际 `service()` 回放 STOP、HUMAN、WAIT、CANCEL，均只取消一次并保留未作业剂量。这些结果属于离线验证。

## 源码阅读

- `spray_precision_r10.py` 中 `StoppedWindow` 按采集时刻和定位代次积累原位证据，最终调用原 `docking_r10.stable_dock`。`LowSpeedCapture` 仅生成停车请求。
- `r10_backend/terrain_pairing.inc` 管理单槽点云队列，原 R08 地形处理函数始终重新核对原时间和配准门限。`spray_precision_session_r10.py` 带有将最终版的回退锁停与等待标志插入独立构建物的逻辑。
- `spray_precision_session_r10.py` 绑定执行器、地形源码、生成物与原任务版本，支持任意连续原任务选点与原预算。历史原版和第一候选仍可分别复现。
- `analyze_spray_precision_r10.py` 把 SQLite 账本和独立物理真值比对，按每点第一次发送导航到第一次成功反馈计算周期。中止点的周期保留为空。源文件与原始录制路径及 SHA-256 在 `comparison.json` 和发布的版本索引中。
