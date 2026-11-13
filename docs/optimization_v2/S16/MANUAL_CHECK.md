# S16 手工验收

所有命令在普通终端串行运行，不并行开两辆仿真。自动实验与用户人工观看分开记录；用户观看前为 NOT_CHECKED。

## 先看已完成证据（不重新仿真）

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S16
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S16
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S16 --print-path
```

check 验证源码、输入、前置和结果哈希；它不重新跑实验。浏览器页面切换 test/val，看三组 CPU/墙钟、查询与维护耗时；选择场景并拖动滑条，核对命中/拒绝原因与实时预测；下方看每个两点任务的完成、停止和数据库状态。default disabled 不等于本步失败。回放的 CPU、墙钟和调用次数是三次重复的合计；准确性仅统计第一轮可评分窗口，不能把重复命中当成独立样本。

## 实际车辆对照

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S16 --case memory_three_way --seed 41 --sim-seconds 300 --wall-timeout 900
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S16 --case memory_wrong_match --seed 41 --sim-seconds 180 --wall-timeout 600
```

第一条串行运行 turning 的 full/event/cache 三组，各最多 100 仿真秒；同一车每组重启，最多共 300 仿真秒。第二条运行一个最多 100 仿真秒的版本失配注入案例，报告中的组件试验另展示同形反向运动拒绝；18–28 仿真秒内地图版本不匹配，候选命中必须被取消并重新预测。GUI 默认打开 RViz 和结束后的报告；无桌面可加 `--headless`。输入读取、前置哈希、真实模型回放需要现实时间，不使用仿真钟。原 prompt 420 秒较紧，慢机建议这里的 600 秒，车辆停止和门限没有放宽。

## 正式验收与断点

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S16 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 4200
```

共四种场景 × 三组，加一个版本失配案例，仿真串行执行。每案例上限 100 仿真秒；整批现实预算 4200 秒不会改变单簇 1800 秒要求。2400 秒也可使用，预算不足应 PARTIAL 而非 PASS。

如果中断，按终端输出的 run_id 代入以下命令；只有源码、输入、库快照及案例证据全部匹配的检查点可以复用：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S16 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 4200 --resume-from /home/ceihao/robot_project/optimization_v2/results/S16/实际运行目录
```

停止放第二终端：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S16
```

只停止本步匹配 PID、启动时间和用户身份的会话。正在工作时先关闭工具、停车、保存进度。不得同时全局 kill ROS/Gazebo。异常应看 result.json、exception.log、各案例 prediction/worker.log 和 commands.jsonl；查询故障必须回退，不能据过期命中强行通过。

## 学习材料

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S16/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S16/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S16/README.md
```

## 查验完整检索记录

浏览器每个场景最多展示 24 个有未来观测标签的抽样窗口，不等于全部查询。完整线上记录在每个案例的 `prediction/forecasts.jsonl`，其中 `memory.decisions` 包含实际特征、事件原因、命中原型、拒绝理由和最终模型选择。以下命令显示本轮 turning/cache 案例的前 10 次实际查询；去掉末尾 `[:10]` 可查看全部。

```bash
cd /home/ceihao/robot_project
python3 - <<'PY'
import json
from pathlib import Path
receipt = json.loads(Path('optimization_v2/results/S16/latest.json').read_text())
run = Path(receipt['report_path']).parent
path = run / 'turning_cache_none/prediction/forecasts.jsonl'
queries = []
for line in path.open():
    packet = json.loads(line)
    for decision in packet.get('memory', {}).get('decisions', []):
        if decision['lookup']['reason'] != 'NOT_QUERIED':
            queries.append(decision)
print('实际查询数:', len(queries))
for decision in queries[:10]:
    print(json.dumps(decision, ensure_ascii=False))
PY
```

原始记录中的 ID 仅为本次跟踪索引；库匹配依据 context 和版本，不依赖这个 ID 表示永久物体身份。
