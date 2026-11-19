# 机器人实时任务观测台

这是与任务控制分离的本机只读 GUI。它支持自动跟随点喷、巡检、割草运行，以及查看历史运行。页面显示实际记录和传感器消息，不根据画面正常推断任务通过，不发送底盘、TF、导航或工具指令。

## 离线查看场景中的某一时刻

Windows 或 Linux 用户可下载随发布附带的 HTML，在现代浏览器中双击打开。选择点喷、割草或巡检运行，用时间条和事件列表查看同期定位、BT、控制及轮速等记录。采样包含每 5 仿真秒和作业事件时刻，各通道按照自己的观测时间与新鲜度筛选。关刀、退出与最终验收单独显示。Linux 实测使用 Chrome 的本地文件模式。实时 GUI 同时列出独立候选运行，在页首标明“独立候选试验”，并在地图上弱化本例未选择的作业段或候选点。[离线回放](../../../validation/closeout_20261119/replay.html) · [浏览器验证](../../../validation/closeout_20261119/replay_browser.json)

## 启动与退出

终端一启动 GUI：

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/gui.sh
```

浏览器打开 **http://127.0.0.1:8088**。可用 `--port 8090` 换端口；不需要新的仿真。没有 ROS 环境或只需要查看记录时，可直接执行：

```bash
python3 optimization_v2/three_tasks/monitor/server.py --no-ros
```

终端二启动一个任务，例如：

```bash
cd /home/ceihao/robot_project
# 三点喷洒，包含真实工具故障及残余重试。
bash optimization_v2/three_tasks/advance.sh R12 record spray
```

其他独立案例：

```bash
# 各行分别运行；同一时刻只允许一套 Gazebo。
bash optimization_v2/three_tasks/advance.sh R12 record inspection
bash optimization_v2/three_tasks/advance.sh R12 record inspection_leader
bash optimization_v2/three_tasks/advance.sh R12 record inspection_blocked
bash optimization_v2/three_tasks/advance.sh R12 record mowing_startup_perception_loss
```

已有任务正在运行时，只启动 GUI 即可。关闭浏览器不影响任务。GUI 终端按 Ctrl+C，只结束 GUI 及其自己的传感器观察进程；任务终端的 Ctrl+C 才会请求任务停车收尾。页面“暂停界面刷新”只冻结显示，不暂停机器人。

## 建议按这条顺序学习

1. **任务总览**：确认运行目录、任务类型、仿真时钟和任务上限。地图区显示计划、已确认点和打开界面之后的传感器定位轨迹；滚轮缩放、拖动平移。“适配地图”恢复视角。
2. **决策与执行链**：查看当前动作、原因码、候选行为资格、阻挡累计、重规划次数。对照事件时间线找到 NAV_GOAL、取消确认、停车、定位准备/提交、工具成功等实际事件。
3. **传感器与定位**：对比配置频率和实际消息频率，检查数据年龄、IMU 偏置、轮速滑移、ICP 配准质量、GNSS 创新量。点云显示雷达坐标系下最多 512 个实际采样点的俯视/侧视图；空白不代表自由空间。车载左目来自 `/s04/perception/left`，与总览的外部仿真观察相机分开。
4. **作业账本与残余**：区分当前作业键的确认量、任务总体确认数、原目标面积和未作业面积。导航成功不等于工具作业成功。完整面积分析在任务关闭并完成评估后才出现。
5. **原始证据**：查看完整字段和版本哈希，下载当前快照用于学习笔记或问题报告。快照是查看时刻的数据，不会变更任务验收。

## 数据链路与时间语义

|页面内容|实际来源|解释|
|---|---|---|
|任务阶段、目标序号|运行目录 `live.json`|当前目标序号不是全部作业成功数|
|BT 决策和候选行为|`live.json.bt`、`decisions.jsonl`|显示原始原因、资格与选中状态|
|定位与 GNSS 监督|`reference/live.json`|地图姿态、创新、跳变锁存、恢复 epoch|
|地形与通行|`risk/live.json`|实际感知有效性、未知/危险栅格与限速|
|控制与停车|`control/gate.jsonl`|原始 Nav2 速度、门控后速度、实测停车标志|
|工具反馈|`ledger/live.json`、`mission_events.jsonl`|幂等工作键、确认量、残余、不确定状态|
|IMU / 轮速处理|`inputs/inputs.jsonl`|传感器处理后的偏置、方差、滑移证据|
|点云 / ICP|`mapping_inputs/frames.jsonl`、`icp_gate/quality.jsonl`|采集时间对齐与里程计质量|
|原始传感器预览|ROS domain 119 的只读订阅|IMU、轮速、GNSS、雷达、相机、局部 odom；不连接评估真值域 120|
|验收结论|保存的 `analysis_*.json` 与阶段 `latest.json`|只显示已有结果，不自行升级为 PASS|

仿真数据年龄用消息时间与任务 `/clock` 比较；文件停更则单独用现实时间判断。`FRESH` 仅表示记录较新，不能替代数据里的 `valid` 或正式传感器准入。GUI 的显示阈值不会改变控制门限。历史运行始终标为历史，即使最后一条数据在其仿真时钟下是新鲜的。

任务暂停或退出时，页面保留最后记录并明确显示停更/历史状态。相机、点云和原始消息预览只属于正在运行且目录匹配的任务；切换到历史记录时不会混入另一台车的实时消息。未保存的历史相机数据标为未收到，不用外部观察视频冒充车载图像。

## 核心代码

- `data.py`：限定在已知运行目录读取；JSONL 从文件末尾有界读取，跳过尚未写完的行；按消息时间区分新鲜、过期、未来时间与缺失。全部读取只读，不加载 `evaluation/truth.jsonl`。
- `server.py`：Python 标准库 HTTP 服务，绑定 `127.0.0.1`；只提供 GET 接口。可选 ROS 子进程由服务器拥有，退出时有界收尾。
- `ros_observer.py`：只订阅传感器与 odom。每 0.5 秒保存一次小型状态；相机约 1 Hz 缩略预览，点云最多 512 点。消息频率按原始仿真时间估计，不能当作现实时间吞吐率。
- `index.html`：本地 HTML/CSS/JavaScript，无外部 CDN。1 Hz 拉取数据，Canvas 绘制轨迹、点云与曲线；下载按钮仅下载浏览器快照。
- 现有 C++ BT、定位、底盘和工具节点负责实际控制。GUI 读取这些节点的输出，不再建立第二个控制器。

当前 GUI 不提供远程控制、不回放完整原始点云、不宣称完成视觉语义识别。长任务的轨迹曲线保留最近 1200 个界面采样；完整记录仍以原运行目录为准。阶段状态可能比刚结束的运行滞后，需执行阶段 finalize 后读取新的封存回执。

## 手工检查

```bash
cd /home/ceihao/robot_project
python3 -m unittest discover -s optimization_v2/three_tasks/monitor -p 'test_monitor.py' -v
python3 - <<'PY'
import json, urllib.request
s = json.load(urllib.request.urlopen('http://127.0.0.1:8088/api/state'))
print(s.get('run'))
print('sim_t:', s.get('sim_t'))
print('ROS channels:', list(s.get('observer', {}).get('channels', {})))
PY
```

浏览器逐项检查：任务时钟前进；选择历史运行后出现“历史记录”；暂停界面刷新后机器人仍按任务执行；恢复刷新后追上当前状态；查看工具反馈和停车事件；GUI Ctrl+C 后任务继续、端口关闭。不要为检查 GUI 同时启动第二套仿真。

离线选片中，先选择带仿真时间的章节，再点“查看这次事件的 BT、定位与控制”，回放会自动选中同一次运行，并定位到事件时刻或其之前最近的采样。实时 GUI 的原始证据菜单另提供现场人工接管通知，包含原点位、剩余剂量、锁存原因和停机核验。[事件选片](../../../media/closeout_20261119/index.html) · [双通知界面检查](../../../validation/closeout_20261119/gui_operator_browser.json)
