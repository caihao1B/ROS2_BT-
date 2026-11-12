# S04 手工验收

普通桌面终端即可，公开执行器自动加载 Jazzy 和本项目环境。所有仿真逐条运行，不并行。S03 前置必须有效。无需新增 sudo 安装。

## 1. 正常传感器面板（一个终端）

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S04 --case sensor_panel --seed 41 --sim-seconds 180 --wall-timeout 420
```

启动后约 15–25 现实秒会出现 RViz 和浏览器实时面板。RViz 左右图像、地面激光环和墙面/障碍高度点应该真实可见；浏览器同页显示左右图像、三维点云投影、IMU、轮速和 GNSS 曲线。车辆在安全短道慢速前后行驶。正常案例内部运行 60 仿真秒；180 是给定上限，420 是外层现实预算。结束后实时服务退出、RViz 关闭，离线证据仍保留。

如浏览器未自动前置，可在运行期间另开一个终端重开本轮实时面板：

```bash
python3 - <<'PY'
import json,pathlib,subprocess
b=pathlib.Path('/home/ceihao/robot_project/optimization_v2')
r=json.loads((b/'runtime/S04.json').read_text())['run_id']
for p in (b/'results/S04'/r).glob('*/online/live_url.txt'):
    subprocess.run(['xdg-open',p.read_text().strip()],check=True)
PY
```

它只在本次仿真运行时有效。结束后实时页停在最后画面，提示等待观测/运行结束；不要把这个状态误判成传感器故障。

## 2. 故障注入演示（上一演示退出后）

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S04 --case sensor_faults --seed 41 --sim-seconds 180 --wall-timeout 420
```

内部运行 90 仿真秒，完整覆盖七种故障。看面板的仿真秒：12–20 延迟，22–30 图像掉帧，32–40 轮式里程计比例失配，42–50 GNSS float，52–60 GNSS 位置中断，62–72 GNSS 坐标偏移且标签仍 fixed，74–82 陀螺 z 偏置。延迟恢复后的 late 丢包是为拒绝倒序时间戳而做的处理，会记录在生成器结果中。

两种演示的离线报告独立于正式验收。运行完可打开“最近一次演示”报告：

```bash
python3 - <<'PY'
import json,pathlib,subprocess
r=pathlib.Path('/home/ceihao/robot_project')
p=json.loads((r/'optimization_v2/results/S04/last_demo.json').read_text())['report_path']
subprocess.run(['xdg-open',str(r/p)],check=True)
PY
```

## 3. 正式状态、交互报告和完整复验

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S04
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S04
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next S04
```

`check` 应显示 PASS 且文件哈希有效；它不重新跑仿真。交互报告可切换四个案例、拖动观测时间片、旋转真实点云；下方为实测曲线、双目图像、RViz 截图和频率表。`next` 必须依据有效 S04/S03 回执给出 S05 就绪；不会执行 S05。人工状态为 NOT_CHECKED，需你自己观察，不影响已授权阶段的准入判断。

重新跑四案完整验收（约 5–8 分钟现实时间，受负载影响；总预算 1800 秒）：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S04 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800 --gui
```

300 是单案最大仿真预算，实际正常案各 50 s、故障案各 90 s；GUI 只在差速案例打开，阿克曼同样真实仿真和评分。无桌面可省略 `--gui`。非零退出/FAIL 就不能沿用本轮 PASS；查看新结果中的 error、metrics 和日志。参数错误或已有其他仿真锁时会拒绝启动。

## 4. 在线话题检查（演示期间的第二终端）

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
export ROS_DOMAIN_ID=119
ros2 topic list
timeout 8s ros2 topic hz /s04/imu/data --use-sim-time
timeout 8s ros2 topic hz /s04/wheel/twist --use-sim-time
ros2 topic echo /s04/stereo/right/camera_info --once --qos-durability transient_local --qos-reliability reliable
ros2 topic echo /s04/gnss/quality --once --qos-reliability best_effort
ros2 topic echo /s04/diagnostics --once
```

应存在五类观测，右 CameraInfo 的 P[3] 为负数且 −P[3]/P[0]=0.12。timeout 的 124 表示按时结束采样；`ros2 topic hz` 额外消耗 CPU，不替代正式采集器的频率结果。故障窗内降频/延迟是预期现象。在线图不应有 `/s04/raw/*`、`/s03/evaluation/*`、`/world/*`。

## 5. 回看有界录包（仿真结束后）

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
S04_BAG=$(python3 - <<'PY'
import json,pathlib
b=pathlib.Path('/home/ceihao/robot_project/optimization_v2')
r=json.loads((b/'results/S04/latest.json').read_text())['run_id']
print(b/'results/S04'/r/'diff_nominal/online/sample_bag')
PY
)
ros2 bag info "$S04_BAG"
ROS_DOMAIN_ID=122 ros2 bag play "$S04_BAG" --clock --qos-profile-overrides-path /home/ceihao/robot_project/optimization_v2/config/sensor_replay_qos.yaml
```

应看到约 2 s 的录制、五类测量、两路 CameraInfo 与 TF static；整包不超过 64 MiB。回放使用 122，避免混入在线 119。录包保存了消息及类型，但未复制 DDS discovery 的全部 QoS 元数据，因此显式加载回放配置，恢复 TF/CameraInfo 的 transient_local 语义。需要观看时，在另一个终端加载环境并设置 `ROS_DOMAIN_ID=122`，启动 `rviz2 -d /home/ceihao/robot_project/optimization_v2/config/rviz/sensors.rviz --ros-args -p use_sim_time:=true`，然后再播放。固定坐标 base_link 的局部传感器查看不需要世界真值。默认录包窗是正常段；故障完整时间序列在离线报告及 JSONL 中。

## 6. 停止与学习

主终端 Ctrl+C 或第二终端执行：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S04
```

只停止本步拥有的会话；先请求停车再关闭，结果记 PARTIAL，不冒充 PASS。重复 stop 应返回 NOT_RUNNING。全部实际命令、退出码、PID 和域在本轮 `commands.jsonl` / 各案 `runtime.json`。

技术与代码入口：

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S04/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S04/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/docs/optimization_v2/S04/RESULTS.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S04/README.md
```
