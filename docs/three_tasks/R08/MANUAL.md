# R08 手工检查

只读检查：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R08 check
bash optimization_v2/three_tasks/advance.sh R08 open
```
`check` 核对实际回执及文件哈希；`open` 打开真实轨迹与每项检查。若仍为 PARTIAL，不能因组件测试通过就认为路线通过。

分别复验，每次等前一个进程完整退出：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R08 test nominal 0
bash optimization_v2/three_tasks/advance.sh R08 test leader 15
bash optimization_v2/three_tasks/advance.sh R08 test detour 15
bash optimization_v2/three_tasks/advance.sh R08 test blocked 20
```
`nominal 0` 保留原完整连接路；其他三个只选原路线前缀，余下距离仍记入残余。检查跟随间距、真实前车停下/恢复、自车响应、规划补丁及重接、受阻后的有限查询和人工原因。受阻案例的通过表示确认停车和保留残余，不表示完成路线。

运行中查看最近会话：
```bash
cd /home/ceihao/robot_project
python3 - <<'PY'
from pathlib import Path
import json
p=max(Path('optimization_v2/three_tasks/results/R08').glob('inspection_20*'),key=lambda x:x.name)
d=json.loads((p/'live.json').read_text())
print(p)
print('仿真秒',d['t'],'阶段',d['phase'],'原路线点序',d['index'])
print('BT',d.get('bt',{}).get('reason_code'))
print('定位',d.get('loc',{}).get('reason'))
PY
```

构建和组件检查：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R08 build
source optimization_v2/env.sh
optimization_v2/venvs/scene/bin/python -m unittest discover -s optimization_v2/ws/src/task_platform/tests -p '*_r08.py' -v
optimization_v2/three_tasks/build/r08/r08_policy_test optimization_v2/three_tasks/editions/R06/inspection.json optimization_v2/ws/src/task_platform/r08_backend/common.xml
```
构建或修改源码后必须复验受影响案例。不要同时构建与运行 Gazebo；不要修改文件哈希或自动解除定位锁定。全程上限 600 秒仿真，短测上限 240 秒，另有墙钟看门狗。以记录中的实际 RTF 估计时间，不能保证实时速度。

定位稀疏路段的独立原地诊断（不计任务完成）：
```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R08 probe 37.7
```
该命令在原路径约 37.7 m 处按计划初始位置建立新会话，使用带偏差的公开位置提示重新定位；25 s 后定位有效比例须达到 80%，同时确认底盘停止和干净退出。它不是完整巡迹验收，不覆盖正式任务回执。查看该会话 `mapping_inputs/frames.jsonl` 的点云宽高、`quality/evidence.jsonl` 的有效性，以及 `fastdds.xml` 的实际传输配置。

检查原地图行人的绕行时，在对应会话的 `mission_events.jsonl` 查找 `PLANNER_QUERY_RESULT`、`OBSERVED_PATCH_ADMITTED` 和 `PATCH_REJOIN_CONFIRMED`。`tracking/tracks.jsonl` 中 `activity_evidence` 表示传感器历史形成的临时避让区域，不是预先读取的行人路径。`analysis_r08.json` 分别给出相对原路的 `maximum_cross_track_m` 与相对当前执行路径的 `maximum_active_path_cross_track_m`。
