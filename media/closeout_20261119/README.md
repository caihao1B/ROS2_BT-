# 三个真实仿真任务的 78 秒演示

下载并解压仓库后，双击 [按事件选片播放](index.html)。可以选整簇或专项原片，点击事件会跳到实际播放器时间。也可直接打开 [三任务精简视频](three_tasks_highlights.mp4)。三个案例均通过各自的短例验收。点喷和割草来自 2026 年 11 月 19 日的新版本，巡检使用 11 月 14 日已通过的前车跟随案例。画面上方保留仿真时间、行为树原因码和速度，下方字幕与实际事件时间对齐。倍速相对于原始录制的现实时间，原片内的仿真时钟保持原值。Windows 可直接下载 MP4 后播放。

| 精简视频时间 | 案例 | 实际观察 |
|---|---|---|
| 00:00–00:22.3 | 点喷工具卡滞，2 倍速 | 第二个点卡滞后，账本保留已计量的 0.022 单位，工具重新执行剩余的 2.978 单位，最终累计确认 3.052 单位。所选 3 个点全部完成，原 A1 有 56 点。[单独片段](spray_excerpt.mp4) · [完整原片](spray_pilot_full.mp4) |
| 00:22.3–00:52.8 | 巡检跟随与停车，3 倍速 | 本次选择约 20 m 的跟随路线。前车停下后，跟车在仿真 64–67 秒内实测位移为 0，恢复后在 72–85 秒内前进 1.45 m。案例最小实测跟随距离为 3.06 m。[单独片段](inspection_excerpt.mp4) · [完整原片](../r12_inspection_leader_full.mp4) |
| 00:52.8–01:18 | 阿克曼割草启动中断，原速 | 感知中断触发取消。0.588 仿真秒后收到车体与工具均已停止的反馈，随后恢复原条带。此次选取安全 B1 路径前 5 段并完成 5 段，按反馈确认作业 26.41 m²，所选目标为 27.95 m²。该版完整执行路径有 47 段。[单独片段](mowing_excerpt.mp4) · [完整原片](mowing_pilot_full.mp4) |

原片和底部字幕各使用独立的数据来源。原片来自 Gazebo 的观测相机，字幕取自任务日志或运行后的物理测量。完整任务的 A1 点喷、B1 割草及正式 S17 六簇矩阵分别评分。各原片 SHA-256、选段起止、字幕时标、独立物理验证与时间线检查均可核对。[素材与验收清单](../../validation/closeout_20261119/media_manifest.json)

保有这三次运行的原始观测目录时，可在机器人主机的项目根目录复算并导出本组合集。脚本使用系统字体、工作区已有的 FFmpeg、原项目的事件选段函数及单线程软件编码。执行前应备份希望另存的同名本轮导出件。

```bash
source optimization_v2/env.sh
optimization_v2/venvs/scene/bin/python media/closeout_20261119/build.py
```

## 两次完整选择的实速原片

- **A1 点喷**。A1 完整簇选取，22/56 个原喷洒点取得作业确认。任务终态 INCOMPLETE，录像与任务联合验收 PARTIAL。 [完整实速录像](spray_20261119_081218_417252_A1_partial_full.mp4)
- **B1 割草**。B1 完整簇选取，25/47 段获准安全执行路径通过端点验收。任务终态 HUMAN_REQUIRED，录像与任务联合验收 PARTIAL。 原作业区分母为 195.99 m²，选中目标上经反馈确认的并集为 74.01 m²。 [完整实速录像](mowing_20261119_072044_764057_B1_partial_full.mp4)

逐事件的播放器时间与仿真时钟可查阅录像清单；离线回放还可以查看同一时刻的 BT 和定位数据。[录像校验和跳转时间](../../validation/closeout_20261119/media_full_runs.json) · [离线事件回放](../../validation/closeout_20261119/replay.html)

点 10_20 长期受阻的独立四点案例能按同一个 BT 事件对照视频和传感器。停车确认、保留剂量与后续点真实喷洒均可单独查看。[47 秒精简片](spray_defer_highlight.mp4) · [347 秒原片](spray_defer_full.mp4) · [逐帧时标与作业账本](../../validation/closeout_20261119/spray_defer_media.json)

无注入自然四点记录展示原故障点在重新定位后完成喷洒。点 9_20 的工具作业取消后，未达标剂量仍留账。[31.6 秒恢复过程](spray_natural_relocalization.mp4) · [402.2 秒原片](spray_natural_full.mp4) · [原始时标与账本](../../validation/closeout_20261119/spray_natural_media.json)

最后一点被设备占用时，独立案例展示实际报警、双停和正常退出。录像中的同一采样帧包含两个相隔 0.012 仿真秒的事件，原始消息分别保存。[27.4 秒交接过程](spray_last_blocked_handoff.mp4) · [141.6 秒原片](spray_last_blocked_full.mp4) · [修复前 149.6 秒原片](spray_last_blocked_initial_full.mp4) · [消息与时标](../../validation/closeout_20261119/spray_last_blocked_media.json)
