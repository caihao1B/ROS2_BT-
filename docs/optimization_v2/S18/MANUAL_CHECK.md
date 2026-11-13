# S18 人工检查

先运行快速开始中的 `--help`、`doctor` 和 `status`。应看到两种底盘、六簇编号、当前地图计划版本；S17 与 S18 的 PARTIAL 状态应清楚可见。人工未完成时不修改为已通过。

```bash
bash /home/ceihao/robot_project/optimization_v2/run.sh audit
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S18
```

离线审计返回 0 仅表示审计正常完成，不表示 S18 PASS。报告应有版本核查、文件体积、学习链接、尚未通过的条件。可用 `stepctl.sh check S18` 查看部分回执。

运行任务需两个终端：第一个按[快速开始](/home/ceihao/robot_project/docs/optimization_v2/快速开始.md)启动一个有界案例；第二个使用 `run.sh stop`。停止后检查输出目录 `runtime.json` 的工具关闭、`cleanup14.json` 的空 survivors，以及 `metrics17.json` 的安全、定位与实际完成量。无有效评分时不能恢复或判通过。

存储与跟随的独立检查：

```bash
cd /home/ceihao/robot_project
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/release_tools/tests/contracts18.py
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/release_tools/tests/follow_contract18.py
```

9 项入口/存储检查、9 项离线跟随检查应通过。它们使用临时载荷或人工构造的观测输入，不代表 Gazebo 动态跟随已经运行。

完整 S18 执行器保留 `delivery_audit`、`release_smoke` 和 `test --suite acceptance` 接口；上层 stepctl 会核对前置 S17。用户授权的提前交付从独立 `run.sh audit` 进入，不绕过全项目通过条件。


外观预览（一次只运行一种，通常约 10 秒，最多 85 秒；不下发运动命令）：

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/release_tools/tests/visual_contract18.py
optimization_v2/venvs/scene/bin/python optimization_v2/ws/src/release_tools/scripts/preview18.py --vehicle ackermann
xdg-open /home/ceihao/robot_project/optimization_v2/showcase/presentation_v1/ackermann/preview.png
# 差速外观：把 --vehicle ackermann 改为 --vehicle diff。
```

比对 `skin.visual.json` 的 `nonvisual_unchanged=true`；这是去掉 visual 后全部 SDF 元素相等，不能解释为视觉传感器数据不变。`preview.sdf` 在皮肤之外额外加了只读摄像头，静态演员夹具和 S03 车辆用于外观检查，并非 S17 在线传感器全套或行为录像。新外观没有替换正式任务默认模型。

短测预算不能机械缩到 60–90 秒：BT 会比较下一动作预计时间与停车预留，可能在定位准入前合理拒绝继续。已有 90 秒 B1 例就是这种情况。建议使用 180 秒以上；全簇仍需按原工时安排。Gazebo 收尾出现 -11 已记录，不能把画面正常等同进程全通过。
