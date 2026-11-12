# S12 手工验收

一个普通终端即可；仅在运行时中断需要第二终端。执行器自行装载隔离环境。本步纯离线，不启动 Gazebo、ROS 节点或工具，无 `/clock`。`--sim-seconds` 保留统一接口但不消耗仿真时钟；页面显示的任务时间来自路线模型，不是实际仿真耗时。首次正式结果已由本步执行器生成，以下演示读取有效版本并真实重跑失败检查。

1. 确认有效回执及下一步条件：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S12
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S12
```

应为 PASS；若 STALE，说明输入、源码或前置回执发生变化，先重新 test，不要手工改哈希。

2. 查看路线动画（通常数秒完成并打开浏览器，现实上限 540 s）：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S12 --case plan_animation --seed 41 --sim-seconds 240 --wall-timeout 540
```

选择 A1/A2/A3，拖动时间轴或播放：服务点按顺序变化，经过旋转、转场、驻留；66 个点属于本版新任务。选择 B1/B2/B3，看到条带、橙色工具关闭转弯、紫色规划覆盖、车体/工具位置；查看停车转向事件和各簇 24 min 预算。取消“放大任务区”，看到原整区仍有大量灰色未完成范围，不应把缩小任务称为完成了原区域。再看四条连接路：两车分别双向巡检，无作业计数。

3. 查看失败用例：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S12 --case infeasible_and_stale --seed 41 --sim-seconds 120 --wall-timeout 300
```

案例菜单选“原整区超预算”：必须 INFEASIBLE；“转弯空间不足”：红色路线/失败采样点超出绿框，两类实际 F2C 曲线均拒绝；“旧版本拒绝”：地图、车辆、工具、配置为 STALE_INPUT_VERSION，篡改计划正文为 PLAN_HASH_MISMATCH。测试只改内存副本，权威文件不变。

4. 从原始输入完整重算验收：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S12 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S12
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S12
```

最多 1800 现实秒；不是 30 分钟仿真作业。终端打印每次尺度/反馈迭代与三个工时，较大 B 任务出现 INFEASIBLE 属于搜索中的真实结果；最终必须六簇通过且 S12-C01…C04 全 PASS。正式测试失败或中断应更新 FAIL/PARTIAL，不能沿用上一轮 PASS。

5. 查看报告路径、计划或停止：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S12 --print-path
cat /home/ceihao/robot_project/optimization_v2/plans/validated/time_budget.csv
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S12
```

`open` 默认打开最近正式报告，演示独立保存在 `results/S12/last_demo.json` 指向的目录。`stop` 只停匹配本次 PID/创建时间/命令的 S12 计算；空闲返回 NOT_RUNNING。已停止的报告动画可直接关浏览器。中断后再次 test 从头有界重算，不会控制其他 ROS 会话。

6. 学习例（见 README，C++ 调用真实库，Python 调用生产图与 TSP）：

```bash
bash /home/ceihao/robot_project/optimization_v2/scripts/build_s12.sh
source /home/ceihao/robot_project/optimization_v2/env.sh
/home/ceihao/robot_project/optimization_v2/ws/build/coverage_planner/planner_lesson_s12
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/lessons/S12/lesson.py
```

C++ 应打印 F2C 条带/转弯并拒绝非法宽度；Python 应打印绕障距离大于直线距离、两个访问顺序，并拒绝不可达点。教学例不是完整任务验收。

人工看过后可以单独记入 `manual_observation.json`，不要修改既有回执造成下游失效。默认 NOT_CHECKED，不由自动截图替你确认。

为控制体积，后台候选请求/曲线及各迭代大 JSON 使用 gzip 无损保存，映射及原始哈希在本轮 `compression_manifest.json`。可用 `gzip -dc 路径.json.gz` 查看；最终选中计划保持普通 JSON。
