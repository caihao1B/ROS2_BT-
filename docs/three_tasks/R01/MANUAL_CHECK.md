# R01 手工验收

以下在当前开发机执行。不要同时打开旧项目仿真。--gui 沿用原入口打开 RViz，查看现有底盘、定位和导航数据；本机有退出缺陷的 ImageDisplay 面板默认不加载，摄像头与识别话题仍工作；正式新模型在 R02 处理。

## 1. 查看三个任务和版本

```bash
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh doctor
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh list
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh status
```

doctor 应返回 INPUTS_VALID；三个 task_id、配置版本和结果根目录不同，但地图 ID、计划版本相同。inspection 的 A1 是定位启动锚点，不代表执行点喷。

## 2. 逐个直观检查启动和自动停止

```bash
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh smoke spray --gui
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh smoke inspection --gui
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh smoke mowing --gui
```

依次执行。默认 60 仿真秒提出停止，随后停车、工具确认停止、关闭模拟器。看见 RViz 窗口不等于验收；以结果中的 admission、server_exit=0、tool_shutdown.state=STOPPED、survivors=[] 和生命周期状态为依据。短测结束时 mission_status 为 PARTIAL 是允许的，因为没有要求跑完整个簇。

## 3. 主动停止与互斥

终端一：
```bash
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh run inspection --gui --sim-seconds 300
```
终端二：
```bash
# 应拒绝，不应停止巡迹
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh stop mowing
# 应拒绝启动第二个仿真
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh smoke spray
# 正确任务，触发有序停车退出
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh stop inspection
```

拒绝操作返回非零是预期行为。取消未完成任务时 run 返回 2，不能把它解释为簇完成；stop 返回值表示停止请求是否完成，还须检查内部 lifecycle_status。Ctrl+C 也走同一条有序退出链路。

## 4. 查看证据

```bash
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh check
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh open
```

check 检查 R01 回执绑定的文件；open 打开本地只读报告。每次运行单独保存到 `optimization_v2/three_tasks/results/<task>/r01_<task>_<UTC时间>/`，包括 task_config、spec、runtime、原评分、stop_request、result 和日志。任务下的 last_run 仅是最新运行指针，不覆盖历史运行目录。

原 S01–S18 状态仍用原命令读取：
```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh status
```

S17/S18 应保持 PARTIAL。R01 通过仅允许推进 R02，不意味着新车辆或全任务通过。
