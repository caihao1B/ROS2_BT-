# 喷洒点受阻后延期并选择后续原点

此次实验让差速车在同一张农田地图上处理 A1 中连续的 4 个原点。Gazebo 在第二个点 `point_10_20` 放置有实体碰撞形状的设备箱，在线节点经真实仿真传感器和 Nav2 获取环境信息。目标选择仍取自原 56 点任务。实验生成独立任务版本、账本与录像目录。

## 现场发生了什么

- `point_11_20` 完成作业并得到计量反馈。面对设备箱，机器人三次向原目标派发导航，均收到失败结果，期间原生行为树两次选择 `REPLAN`。
- 仿真 80.944 秒，原生行为树发出编号 14 的 `DEFER`，原因是 `TRY_OTHER_OBLIGATION_RETAIN_RESIDUAL`。此时原目标仍有 3.0 剂量待完成，账本和独立物理喷流均为零，车身与工具已停稳。
- 机器人绕过设备箱，到达下一原点 `point_9_20`。在 141.844 仿真秒，账本确认 3.078 剂量，独立物理流量积分为 3.229，该次作业最大物理位置误差为 0.0381 m。第四点 `point_8_20` 也有独立作业反馈。178.150 秒结束时，车身与工具均停稳。
- 本次实际执行结果为 3 个作业完成、1 个保留残余。粗导航延期场景专项通过。正式 A1 完整簇验收保留 22/56 和 PARTIAL。

完整录像和精简录像的帧时、播放倍速、关键事件与哈希可在独立素材清单中查阅。[素材清单](../../validation/closeout_20261119/spray_defer_media.json) · [完整录像](../../media/closeout_20261119/spray_defer_full.mp4) · [精简录像](../../media/closeout_20261119/spray_defer_highlight.mp4)

## 为什么同时有精停专项回放

正式 A1 在同一处遇到另一种失败。全局接近后，局部控制器 `DockRPP` 执行了两次独立微调，各因碰撞检查持续拒绝而终止。最后一个停稳观察窗中位误差为 0.03730 m，原门限为 0.035 m。此时还有 855.712 仿真秒预算。候选分派器只接受已经发出、收到失败且末次观察仍无法准许作业的真实微调记录。定位丢失、感知失效、工具状态不确定、车身仍在运动或缺少后续目标时都不能将点延期后继续。

独立 DDS 域原生回放复算了两次正式微调轨迹，再将设备箱实测例的 195 条原始决策上下文逐条送进 BehaviorTree.CPP 监督器。输入文件与回执保存了准确数量和原始记录摘要。两段证据分别对应精停失败识别、真实 `WAIT → REPLAN → DEFER`。同一次 Gazebo 运行中的精停失败转延期仍待测试。[正式中断证据](../../validation/closeout_20261119/spray_full_diagnosis.json) · [原生回放回执](../../validation/closeout_20261119/spray_defer_native_replay.json)

## 在 Linux 上运行

在已按仓库主页建好 R10/R12 仿真运行环境的工程根目录执行。`source` 将当前 shell 的 ROS、Gazebo 和本工程工作空间加入搜索路径。`prepare` 生成实验专用分派器。`export ROS_DOMAIN_ID=151` 只在子 shell 内对原生回放生效，使测试消息留在独立域。

```bash
source optimization_v2/env.sh
python3 optimization_v2/ws/src/task_platform/tests/test_spray_defer_candidate.py
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/task_platform/scripts/spray_defer_fixture_r10.py check
(
  export ROS_DOMAIN_ID=151 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/task_platform/scripts/probe_spray_defer_native_r10.py \
    --bundle validation/closeout_20261119/spray_defer_replay_inputs.json.gz \
    --out /tmp/spray_defer_native_replay.json
)
```

模拟器空闲时，可以启动单独的真实四点任务。这里的 330 秒是单例仿真时钟上限。终端将打印包含 `candidate_acceptance.json` 的新会话目录。检查其中的 `behavior_result`、`coarse_defer_demonstrated`、`precision_defer_demonstrated`、`deferred[].next_point` 和 `physical_jobs`。更换成同点自然环境时在命令末尾添加 `--natural`，每次实际遇到的事件以该次回执为准。

```bash
source optimization_v2/env.sh
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/task_platform/scripts/spray_defer_candidate_r10.py run --seconds 330 --rtf .55
```
