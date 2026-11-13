# S17 手工验收命令

实际项目位于 `/home/ceihao/robot_project`。请串行执行；不要同时启动两辆车或两个 Gazebo。完整正式矩阵尚未通过，当前运行结果及缺项见 RESULTS.md。

## 查看已有实测证据

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S17
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S17
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S17 --print-path
```

`check` 展示当前阶段状态；当前公共索引对非 PASS 会提前返回，所以 PARTIAL 的证据完整性另用下面的专用命令核查。两者都不会运行 60 次仿真。PARTIAL/FAIL 返回非零是实际状态。页面显示六簇固定清单、缺项和开发案例；开发测试不能变成正式绿色成绩。选择失败案例查看轨迹、原始残余与原因；筛选簇、种子、控制器、结果。图中的测站与规划路线不是已完成作业。

```bash
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/ws/src/acceptance_runner/scripts/audit17.py
```

预期同时出现 `integrity_pass: true` 与 `acceptance_status: PARTIAL`：文件证据一致不等于任务通过。此命令递归核查前置回执和实际文件哈希，可能需要数十秒。

## 短演示与真实恢复

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S17 --case acceptance_preview --seed 41 --sim-seconds 300 --wall-timeout 720 --headless --rtf 1
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S17 --case resume_partial --seed 41 --sim-seconds 300 --wall-timeout 900 --headless --rtf 1
```

第一条保留 A2 原始 22 点分母，运行有界预览；第二条实际停止、关闭工具、恢复 SQLite 账本并继续新任务。观察 `resume_proof.json` 的 `passed`、恢复点数和 `duplicate_goals`，不能只看进程退出码。短演示不承诺完成全簇。默认无 GUI，可在总览检查轨迹；去掉 `--headless` 会打开当前环境支持的仿真显示。依赖回执哈希验证也消耗现实时间。

## 正式分批验收

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S17 --suite formal --seed 41 --sim-seconds 1800 --wall-timeout 21600 --shard 1/10 --resume --rtf 1
```

单分片最多六个正式簇，单例包含停止在内最多 1800 仿真秒。执行器遇到未完成或失败例即保留证据并停止当前批次，必须先诊断；不能为了收集更多成功种子跳过失败。单分片退出 0 仍不等于 S17 PASS。

```bash
for shard in 2 3 4 5 6 7 8 9 10; do
  bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S17 --suite formal --seed 41 --sim-seconds 1800 --wall-timeout 21600 --shard "$shard/10" --resume --rtf 1 || break
done
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S17 --suite ablations --seed 41 --sim-seconds 1800 --wall-timeout 21600 --resume --rtf 1
```

同一版本使用 `--resume` 跳过已提交案例；不同地图、控制或评分版本不能复用旧成绩。消融套件包含配对实验及连接路/故障/重启检查；未完成的几何变体资格仍是独立缺项，不能仅因一批命令结束就声明通过。

## 组件与核心代码学习

```bash
bash /home/ceihao/robot_project/optimization_v2/scripts/build_s17.sh
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/ws/src/acceptance_runner/scripts/verify17.py
cat /home/ceihao/robot_project/optimization_v2/lessons/S17/README.md
```

组件检查覆盖检查点防篡改、空矩阵、时间连接、路径后缀、真实跳变、视觉增量门控及候选代价不清除障碍；它不会启动完整任务，也不产生正式完成数。教学说明提供 C++/Python 单独编译运行命令。

## 停止与继续检查

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S17
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S17
```

停止命令仅处理登记为 S17 且创建身份匹配的本会话进程；`NOT_RUNNING` 表示没有匹配会话，不会全局杀死 ROS/Gazebo。S18 的前置是有效 S17 PASS；未人工打开页面不会新增权限门槛，但实际失败和缺项不能忽略。
