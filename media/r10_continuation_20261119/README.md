# R10 现场录像与事件对照

原片来自车外 Gazebo 相机。短片保留一段连续画面，加速倍数写在画面上沿。使用 Windows 浏览器或本地播放器打开 MP4 即可观看，下载后也可离线逐帧查看。

录像时间指播放器中的位置。仿真时刻来自 ROS `/clock`，倍速与实时因子会让两个计时值不同。`mission_result.json` 给出任务终态和已确认工作，`runtime.json` 记录执行进程退出与清理。正式验收见 [R10 状态](../../validation/R10/latest.json)。

## A1 原控制器连续四点对照

[连续节选（01:02，4 倍速）](a1_control_short.mp4) · [完整原片（05:22）](a1_control_full.mp4) · [全部时间锚点与视频校验值](../../validation/r10_continuation_20261119/films/a1_control/media.json) · [任务结果](../../validation/r10_continuation_20261119/films/a1_control/mission_result.json) · [运行配置与签名](../../validation/r10_continuation_20261119/films/a1_control/spec.json) · [该会话源码索引](../../validation/r10_continuation_20261119/versions/a1_control.json)

会话 `spray_defer_20261119_121141_482729`。仿真终态时刻 156.448 秒。
本次选择 4 个点，任务反馈确认 3 个。原 A1 共 56 个点。
独立 SQLite 账本复核确认 3 个点，与任务成功点名单一致。
[原始物理计量、SQLite 作业账本、定位轨迹和传感器校验值](../../validation/r10_continuation_20261119/films/a1_control/work/index.json)。压缩账本可用 [SQLite 文件](../../validation/r10_continuation_20261119/films/a1_control/work/ledger/ledger.sqlite.gz) 下载并解压，jobs 表记录每点实际量、完成状态和不确定标志。
终态为 `HUMAN_REQUIRED`。
任务记录的结束原因是 `RuntimeError('NO_QUALIFIED_DEFER_OR_ALTERNATIVE')`。

| 行为 | 点目标或路径段 | 仿真时刻（秒） | 原片位置 | 短片位置 |
|---|---|---:|---:|---:|
| 下发点目标 | point_11_20 | 35.688 | 01:13 | 00:00 |
| 精停检查 | point_11_20 | 39.030 | 01:20 | 00:02 |
| 工具反馈 | point_11_20 | 43.106 | 01:28 | 00:04 |
| 下发点目标 | point_10_20 | 43.154 | 01:28 | 00:04 |
| 精停检查 | point_10_20 | 68.638 | 02:22 | 00:17 |
| 工具反馈 | point_10_20 | 76.966 | 02:37 | 00:21 |
| 下发点目标 | point_9_20 | 84.454 | 02:52 | 00:25 |
| 精停检查 | point_9_20 | 114.360 | 03:51 | 00:40 |
| 工具反馈 | point_9_20 | 118.458 | 03:59 | 00:42 |
| 下发点目标 | point_8_20 | 118.504 | 03:59 | 00:42 |
| 精停检查 | point_8_20 | 143.640 | 04:49 | 00:54 |
| 工具最终停止 |  | 156.448 | 05:14 | 01:00 |

## A1 配对点云与闭环精停

[连续节选（01:06，3 倍速）](a1_first_short.mp4) · [完整原片（04:30）](a1_first_full.mp4) · [全部时间锚点与视频校验值](../../validation/r10_continuation_20261119/films/a1_first/media.json) · [任务结果](../../validation/r10_continuation_20261119/films/a1_first/mission_result.json) · [运行配置与签名](../../validation/r10_continuation_20261119/films/a1_first/spec.json) · [该会话源码索引](../../validation/r10_continuation_20261119/versions/a1_first.json)

会话 `spray_defer_20261119_121833_942724`。仿真终态时刻 128.164 秒。
本次选择 4 个点，任务反馈确认 4 个。原 A1 共 56 个点。
独立 SQLite 账本复核确认 4 个点，与任务成功点名单一致。
[原始物理计量、SQLite 作业账本、定位轨迹和传感器校验值](../../validation/r10_continuation_20261119/films/a1_first/work/index.json)。压缩账本可用 [SQLite 文件](../../validation/r10_continuation_20261119/films/a1_first/work/ledger/ledger.sqlite.gz) 下载并解压，jobs 表记录每点实际量、完成状态和不确定标志。

| 行为 | 点目标或路径段 | 仿真时刻（秒） | 原片位置 | 短片位置 |
|---|---|---:|---:|---:|
| 下发点目标 | point_11_20 | 34.832 | 01:13 | 00:01 |
| 精停检查 | point_11_20 | 36.784 | 01:17 | 00:02 |
| 工具反馈 | point_11_20 | 40.900 | 01:25 | 00:05 |
| 下发点目标 | point_10_20 | 40.924 | 01:25 | 00:05 |
| 精停检查 | point_10_20 | 63.240 | 02:10 | 00:20 |
| 工具反馈 | point_10_20 | 67.352 | 02:18 | 00:22 |
| 下发点目标 | point_9_20 | 67.380 | 02:19 | 00:23 |
| 精停检查 | point_9_20 | 92.850 | 03:11 | 00:40 |
| 工具反馈 | point_9_20 | 96.948 | 03:19 | 00:43 |
| 下发点目标 | point_8_20 | 96.990 | 03:19 | 00:43 |
| 精停检查 | point_8_20 | 122.040 | 04:11 | 01:00 |
| 工具反馈 | point_8_20 | 126.112 | 04:19 | 01:03 |
| 工具最终停止 |  | 128.164 | 04:24 | 01:04 |

## A1 独立场景连续四点复验

[连续节选（00:57，3 倍速）](a1_repeat_short.mp4) · [完整原片（04:04）](a1_repeat_full.mp4) · [全部时间锚点与视频校验值](../../validation/r10_continuation_20261119/films/a1_repeat/media.json) · [任务结果](../../validation/r10_continuation_20261119/films/a1_repeat/mission_result.json) · [运行配置与签名](../../validation/r10_continuation_20261119/films/a1_repeat/spec.json) · [该会话源码索引](../../validation/r10_continuation_20261119/versions/a1_repeat.json)

会话 `spray_defer_20261119_122402_750899`。仿真终态时刻 120.242 秒。
本次选择 4 个点，任务反馈确认 4 个。原 A1 共 56 个点。
独立 SQLite 账本复核确认 4 个点，与任务成功点名单一致。
[原始物理计量、SQLite 作业账本、定位轨迹和传感器校验值](../../validation/r10_continuation_20261119/films/a1_repeat/work/index.json)。压缩账本可用 [SQLite 文件](../../validation/r10_continuation_20261119/films/a1_repeat/work/ledger/ledger.sqlite.gz) 下载并解压，jobs 表记录每点实际量、完成状态和不确定标志。

| 行为 | 点目标或路径段 | 仿真时刻（秒） | 原片位置 | 短片位置 |
|---|---|---:|---:|---:|
| 下发点目标 | point_11_20 | 36.402 | 01:11 | 00:01 |
| 精停检查 | point_11_20 | 38.340 | 01:15 | 00:02 |
| 工具反馈 | point_11_20 | 42.454 | 01:24 | 00:05 |
| 下发点目标 | point_10_20 | 42.502 | 01:24 | 00:05 |
| 精停检查 | point_10_20 | 63.210 | 02:07 | 00:19 |
| 工具反馈 | point_10_20 | 67.278 | 02:15 | 00:22 |
| 下发点目标 | point_9_20 | 67.320 | 02:15 | 00:22 |
| 精停检查 | point_9_20 | 94.170 | 03:07 | 00:39 |
| 工具反馈 | point_9_20 | 98.272 | 03:15 | 00:42 |
| 下发点目标 | point_8_20 | 98.310 | 03:15 | 00:42 |
| 精停检查 | point_8_20 | 114.120 | 03:46 | 00:52 |
| 工具反馈 | point_8_20 | 118.196 | 03:54 | 00:55 |
| 工具最终停止 |  | 120.242 | 03:58 | 00:56 |

## B1 首次整车复验与停车交接

[连续节选（00:25，1 倍速）](b1_initial_short.mp4) · [完整原片（01:09）](b1_initial_full.mp4) · [全部时间锚点与视频校验值](../../validation/r10_continuation_20261119/films/b1_initial/media.json) · [任务结果](../../validation/r10_continuation_20261119/films/b1_initial/mission_result.json) · [运行配置与签名](../../validation/r10_continuation_20261119/films/b1_initial/spec.json) · [该会话源码索引](../../validation/r10_continuation_20261119/versions/b1_initial.json)

会话 `mowing_20261119_122854_238212`。仿真终态时刻 31.624 秒。
本次选择 4 个换向分段，记录导航请求 7 次。停止位姿确认涉及 0 段，任务到达事件涉及 0 段。
独立复核将位置、航向、实际换挡和原轨迹一并计入，本次合格 0/4 段。[逐挡实测与排除原因](../../validation/r10_continuation_20261119/b1/initial/physical_connector_report.json)
终态为 `HUMAN_REQUIRED`。
任务记录的结束原因是 `RuntimeError('SEGMENT_RETRY_BOUND')`。

| 行为 | 点目标或路径段 | 仿真时刻（秒） | 原片位置 | 短片位置 |
|---|---|---:|---:|---:|
| 下发当前挡位轨迹 | B1:offline_connection:6:0 | 22.170 | 00:42 | 00:03 |
| 任务停止 |  | 31.486 | 01:00 | 00:20 |

## B1 时间配对修复与四挡整车复验

[连续节选（01:10，6 倍速）](b1_paired_short.mp4) · [完整原片（07:52）](b1_paired_full.mp4) · [全部时间锚点与视频校验值](../../validation/r10_continuation_20261119/films/b1_paired/media.json) · [任务结果](../../validation/r10_continuation_20261119/films/b1_paired/mission_result.json) · [运行配置与签名](../../validation/r10_continuation_20261119/films/b1_paired/spec.json) · [该会话源码索引](../../validation/r10_continuation_20261119/versions/b1_paired.json)

会话 `mowing_20261119_123242_987678`。仿真终态时刻 207.902 秒。
本次选择 4 个换向分段，记录导航请求 10 次。停止位姿确认涉及 2 段，任务到达事件涉及 2 段。
独立复核将位置、航向、实际换挡和原轨迹一并计入，本次合格 1/4 段。[逐挡实测与排除原因](../../validation/r10_continuation_20261119/b1/capture_time/physical_connector_report.json)
终态为 `HUMAN_REQUIRED`。
任务记录的结束原因是 `RuntimeError('SEGMENT_RETRY_BOUND')`。

| 行为 | 点目标或路径段 | 仿真时刻（秒） | 原片位置 | 短片位置 |
|---|---|---:|---:|---:|
| 下发当前挡位轨迹 | B1:offline_connection:6:0 | 22.350 | 00:48 | 00:00 |
| 记录分段到达 | B1:offline_connection:6:0 | 32.360 | 01:10 | 00:04 |
| 停车位姿确认 | B1:offline_connection:6:0 | 32.972 | 01:12 | 00:04 |
| 下发当前挡位轨迹 | B1:offline_connection:6:1 | 33.152 | 01:12 | 00:04 |
| 停车位姿确认 | B1:offline_connection:6:1 | 144.750 | 05:21 | 00:45 |
| 记录分段到达 | B1:offline_connection:6:1 | 144.882 | 05:21 | 00:45 |
| 下发当前挡位轨迹 | B1:offline_connection:6:2 | 145.642 | 05:23 | 00:46 |
| 任务停止 |  | 207.762 | 07:44 | 01:09 |

## B1 转向控制修复与逐挡停车复验

[连续节选（01:13，8 倍速）](b1_third_short.mp4) · [完整原片（10:50）](b1_third_full.mp4) · [全部时间锚点与视频校验值](../../validation/r10_continuation_20261119/films/b1_third/media.json) · [任务结果](../../validation/r10_continuation_20261119/films/b1_third/mission_result.json) · [运行配置与签名](../../validation/r10_continuation_20261119/films/b1_third/spec.json) · [该会话源码索引](../../validation/r10_continuation_20261119/versions/b1_third.json)

会话 `mowing_20261119_124201_958601`。仿真终态时刻 279.382 秒。
本次选择 4 个换向分段，记录导航请求 9 次。停止位姿确认涉及 4 段，任务到达事件涉及 4 段。
独立复核将位置、航向、实际换挡和原轨迹一并计入，本次合格 3/4 段。[逐挡实测与排除原因](../../validation/r10_continuation_20261119/b1/stopped_steering/physical_connector_report.json)
终态为 `COMPLETED`。

| 行为 | 点目标或路径段 | 仿真时刻（秒） | 原片位置 | 短片位置 |
|---|---|---:|---:|---:|
| 下发当前挡位轨迹 | B1:offline_connection:6:0 | 23.970 | 00:58 | 00:00 |
| 记录分段到达 | B1:offline_connection:6:0 | 39.210 | 01:32 | 00:04 |
| 停车位姿确认 | B1:offline_connection:6:0 | 39.780 | 01:34 | 00:04 |
| 下发当前挡位轨迹 | B1:offline_connection:6:1 | 39.962 | 01:34 | 00:04 |
| 记录分段到达 | B1:offline_connection:6:1 | 58.524 | 02:16 | 00:10 |
| 停车位姿确认 | B1:offline_connection:6:1 | 59.100 | 02:17 | 00:10 |
| 下发当前挡位轨迹 | B1:offline_connection:6:2 | 59.312 | 02:17 | 00:10 |
| 停车位姿确认 | B1:offline_connection:6:2 | 209.040 | 07:59 | 00:53 |
| 记录分段到达 | B1:offline_connection:6:2 | 209.168 | 08:00 | 00:53 |
| 记录分段到达 | B1:offline_connection:6:3 | 276.652 | 10:36 | 01:12 |
| 停车位姿确认 | B1:offline_connection:6:3 | 277.230 | 10:37 | 01:12 |

## B1 四挡实车停车与航向逐段复核

[连续节选（01:13，6 倍速）](b1_bound_short.mp4) · [完整原片（08:10）](b1_bound_full.mp4) · [全部时间锚点与视频校验值](../../validation/r10_continuation_20261119/films/b1_bound/media.json) · [任务结果](../../validation/r10_continuation_20261119/films/b1_bound/mission_result.json) · [运行配置与签名](../../validation/r10_continuation_20261119/films/b1_bound/spec.json) · [该会话源码索引](../../validation/r10_continuation_20261119/versions/b1_bound.json)

会话 `mowing_20261119_125335_539235`。仿真终态时刻 228.312 秒。
本次选择 4 个换向分段，记录导航请求 23 次。停止位姿确认涉及 4 段，任务到达事件涉及 4 段。
独立复核将位置、航向、实际换挡和原轨迹一并计入，本次合格 4/4 段。[逐挡实测与排除原因](../../validation/r10_continuation_20261119/b1/physical_connector_report.json)
终态为 `COMPLETED`。

| 行为 | 点目标或路径段 | 仿真时刻（秒） | 原片位置 | 短片位置 |
|---|---|---:|---:|---:|
| 下发当前挡位轨迹 | B1:offline_connection:6:0 | 22.158 | 00:46 | 00:00 |
| 记录分段到达 | B1:offline_connection:6:0 | 34.024 | 01:11 | 00:04 |
| 停车位姿确认 | B1:offline_connection:6:0 | 34.652 | 01:13 | 00:04 |
| 下发当前挡位轨迹 | B1:offline_connection:6:1 | 35.264 | 01:14 | 00:05 |
| 记录分段到达 | B1:offline_connection:6:1 | 53.764 | 01:53 | 00:11 |
| 停车位姿确认 | B1:offline_connection:6:1 | 54.362 | 01:54 | 00:11 |
| 下发当前挡位轨迹 | B1:offline_connection:6:2 | 55.040 | 01:56 | 00:12 |
| 停车位姿确认 | B1:offline_connection:6:2 | 201.990 | 07:08 | 01:04 |
| 记录分段到达 | B1:offline_connection:6:2 | 202.126 | 07:08 | 01:04 |
| 下发当前挡位轨迹 | B1:offline_connection:6:3 | 203.382 | 07:11 | 01:04 |
| 停车位姿确认 | B1:offline_connection:6:3 | 225.450 | 07:57 | 01:12 |
| 记录分段到达 | B1:offline_connection:6:3 | 225.592 | 07:57 | 01:12 |

下表把逐挡验收的停车窗口与原片对齐。误差取窗口内物理测量的最大值。

| 挡位 | 最大位置误差 | 最大航向误差 | 停车窗口（仿真秒） | 原片位置 |
|---|---:|---:|---:|---:|
| 1 · 前进 | 4.87 cm | 0.80° | 34.022–34.652 | 01:11 |
| 2 · 倒车 | 4.90 cm | 0.55° | 53.762–54.362 | 01:53 |
| 3 · 前进 | 9.38 cm | 0.63° | 202.108–202.710 | 07:08 |
| 4 · 倒车 | 3.57 cm | 1.43° | 225.570–226.172 | 07:57 |

## A1 原序全选五十六点现场实测

[连续节选（01:14，21 倍速）](a1_long_short.mp4) · [完整原片（26:31）](a1_long_full.mp4) · [全部时间锚点与视频校验值](../../validation/r10_continuation_20261119/films/a1_long/media.json) · [任务结果](../../validation/r10_continuation_20261119/films/a1_long/mission_result.json) · [运行配置与签名](../../validation/r10_continuation_20261119/films/a1_long/spec.json) · [该会话源码索引](../../validation/r10_continuation_20261119/versions/a1_long.json)

会话 `spray_defer_20261119_130223_269769`。仿真终态时刻 978.792 秒。
本次选择 56 个点，任务反馈确认 26 个。原 A1 共 56 个点。
独立 SQLite 账本复核确认 26 个点，与任务成功点名单一致。
[原始物理计量、SQLite 作业账本、定位轨迹和传感器校验值](../../validation/r10_continuation_20261119/films/a1_long/work/index.json)。压缩账本可用 [SQLite 文件](../../validation/r10_continuation_20261119/films/a1_long/work/ledger/ledger.sqlite.gz) 下载并解压，jobs 表记录每点实际量、完成状态和不确定标志。
任务记录的结束原因是 `TimeoutError('TASK_BOUND_OR_STOP_REQUEST')`。

| 行为 | 点目标或路径段 | 仿真时刻（秒） | 原片位置 | 短片位置 |
|---|---|---:|---:|---:|
| 下发点目标 | point_7_14 | 20.366 | 00:29 | 00:00 |
| 精停检查 | point_7_14 | 22.292 | 00:33 | 00:00 |
| 工具反馈 | point_7_14 | 26.672 | 00:40 | 00:00 |
| 下发点目标 | point_8_14 | 26.756 | 00:41 | 00:00 |
| 精停检查 | point_8_14 | 43.712 | 01:08 | 00:01 |
| 工具反馈 | point_8_14 | 47.834 | 01:15 | 00:02 |
| 下发点目标 | point_8_15 | 47.886 | 01:15 | 00:02 |
| 精停检查 | point_8_15 | 80.790 | 02:11 | 00:04 |
| 工具反馈 | point_8_15 | 84.888 | 02:18 | 00:05 |
| 下发点目标 | point_9_15 | 84.932 | 02:18 | 00:05 |
| 精停检查 | point_9_15 | 133.352 | 03:35 | 00:09 |
| 工具反馈 | point_9_15 | 135.634 | 03:39 | 00:09 |
| 下发点目标 | point_9_14 | 140.494 | 03:47 | 00:09 |
| 精停检查 | point_9_14 | 184.532 | 04:59 | 00:12 |
| 工具反馈 | point_9_14 | 188.670 | 05:05 | 00:13 |
| 下发点目标 | point_10_14 | 188.730 | 05:05 | 00:13 |
| 精停检查 | point_10_14 | 222.692 | 05:59 | 00:15 |
| 工具反馈 | point_10_14 | 226.774 | 06:06 | 00:16 |
| 下发点目标 | point_10_15 | 226.802 | 06:06 | 00:16 |
| 精停检查 | point_10_15 | 258.840 | 06:57 | 00:18 |
| 工具反馈 | point_10_15 | 262.954 | 07:04 | 00:18 |
| 下发点目标 | point_10_16 | 262.990 | 07:04 | 00:18 |
| 精停检查 | point_10_16 | 291.718 | 07:50 | 00:21 |
| 工具反馈 | point_10_16 | 295.872 | 07:56 | 00:21 |
| 下发点目标 | point_9_16 | 295.918 | 07:57 | 00:21 |
| 精停检查 | point_9_16 | 327.426 | 08:49 | 00:23 |
| 工具反馈 | point_9_16 | 331.650 | 08:55 | 00:24 |
| 下发点目标 | point_8_16 | 331.684 | 08:55 | 00:24 |
| 精停检查 | point_8_16 | 359.496 | 09:42 | 00:26 |
| 工具反馈 | point_8_16 | 363.586 | 09:48 | 00:26 |
| 下发点目标 | point_8_17 | 363.630 | 09:48 | 00:26 |
| 精停检查 | point_8_17 | 393.270 | 10:38 | 00:29 |
| 工具反馈 | point_8_17 | 393.302 | 10:38 | 00:29 |
| 下发点目标 | point_9_17 | 399.422 | 10:48 | 00:29 |
| 精停检查 | point_9_17 | 432.960 | 11:43 | 00:32 |
| 工具反馈 | point_9_17 | 437.094 | 11:50 | 00:32 |
| 下发点目标 | point_10_17 | 437.132 | 11:50 | 00:32 |
| 精停检查 | point_10_17 | 464.580 | 12:34 | 00:34 |
| 工具反馈 | point_10_17 | 468.716 | 12:41 | 00:35 |
| 下发点目标 | point_10_18 | 468.754 | 12:41 | 00:35 |
| 精停检查 | point_10_18 | 507.180 | 13:44 | 00:37 |
| 工具反馈 | point_10_18 | 511.260 | 13:51 | 00:38 |
| 下发点目标 | point_9_18 | 511.286 | 13:51 | 00:38 |
| 精停检查 | point_9_18 | 553.564 | 15:02 | 00:41 |
| 工具反馈 | point_9_18 | 557.650 | 15:09 | 00:42 |
| 下发点目标 | point_8_18 | 557.702 | 15:09 | 00:42 |
| 精停检查 | point_8_18 | 582.420 | 15:51 | 00:44 |
| 工具反馈 | point_8_18 | 586.590 | 15:57 | 00:44 |
| 下发点目标 | point_8_19 | 586.624 | 15:57 | 00:44 |
| 精停检查 | point_8_19 | 616.680 | 16:46 | 00:46 |
| 工具反馈 | point_8_19 | 620.766 | 16:53 | 00:46 |
| 下发点目标 | point_9_19 | 620.792 | 16:53 | 00:46 |
| 精停检查 | point_9_19 | 651.570 | 17:42 | 00:49 |
| 工具反馈 | point_9_19 | 655.686 | 17:49 | 00:49 |
| 下发点目标 | point_10_19 | 655.746 | 17:49 | 00:49 |
| 精停检查 | point_10_19 | 682.110 | 18:31 | 00:51 |
| 工具反馈 | point_10_19 | 686.188 | 18:38 | 00:52 |
| 下发点目标 | point_11_19 | 686.222 | 18:38 | 00:52 |
| 精停检查 | point_11_19 | 708.242 | 19:14 | 00:53 |
| 工具反馈 | point_11_19 | 712.320 | 19:20 | 00:53 |
| 下发点目标 | point_12_20 | 712.360 | 19:20 | 00:53 |
| 精停检查 | point_12_20 | 764.912 | 20:44 | 00:57 |
| 工具反馈 | point_12_20 | 769.020 | 20:51 | 00:58 |
| 下发点目标 | point_11_20 | 769.052 | 20:51 | 00:58 |
| 精停检查 | point_11_20 | 799.674 | 21:40 | 01:00 |
| 工具反馈 | point_11_20 | 803.802 | 21:47 | 01:00 |
| 下发点目标 | point_10_20 | 803.854 | 21:47 | 01:00 |
| 精停检查 | point_10_20 | 841.978 | 22:48 | 01:03 |
| 工具反馈 | point_10_20 | 846.336 | 22:55 | 01:04 |
| 下发点目标 | point_9_20 | 846.390 | 22:55 | 01:04 |
| 精停检查 | point_9_20 | 886.082 | 23:58 | 01:07 |
| 工具反馈 | point_9_20 | 890.192 | 24:04 | 01:07 |
| 下发点目标 | point_8_20 | 890.224 | 24:04 | 01:07 |
| 精停检查 | point_8_20 | 912.332 | 24:39 | 01:09 |
| 工具反馈 | point_8_20 | 916.478 | 24:46 | 01:09 |
| 下发点目标 | point_8_21 | 916.534 | 24:46 | 01:09 |
| 精停检查 | point_8_21 | 952.412 | 25:43 | 01:12 |
| 工具反馈 | point_8_21 | 956.570 | 25:50 | 01:12 |
| 下发点目标 | point_9_21 | 956.614 | 25:50 | 01:12 |
| 任务停止 |  | 978.306 | 26:25 | 01:14 |
| 工具最终停止 |  | 978.792 | 26:25 | 01:14 |

## 逐条核对

从源码复查历史录像时，打开该会话源码索引。与基线相同的依赖可在索引给出的 Git 提交中找到。source_archive 将其余源文件映射到可解压的精确版本，路径以 versions 目录为起点。generated_scene_inputs 指向该次冻结的世界或运行器登记的场景配置。现场构建物的 SHA-256 由运行配置中的 implementation_bindings 记录。

同一目标多次触发精停检查时，上表只列第一次，`media.json` 保留全部动作时间。相机逐帧索引在 `video/frames.jsonl.gz`，原始决策事件在 `mission_events.jsonl.gz`。以下示例在仓库根目录执行。

```bash
python3 -m json.tool validation/r10_continuation_20261119/films/a1_control/mission_result.json
gzip -cd validation/r10_continuation_20261119/films/a1_control/mission_events.jsonl.gz | head -n 5
sha256sum media/r10_continuation_20261119/a1_control_full.mp4
```

Python 命令按字段显示任务原始回执。`gzip -cd` 把压缩的事件流输出到终端，`head` 截取前五条。`sha256sum` 可与对应 `media.json` 的 `raw.sha256` 核对。
