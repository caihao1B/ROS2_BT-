"""Update learning references from current source lines and actual receipts."""
from common17 import *
import datetime

def generate():
    docs=ROOT/'docs/optimization_v2/S17';docs.mkdir(exist_ok=True)
    references=[
        ('include/acceptance_monitor.hpp','inline json monitor','C++ 独立审计：仿真时钟、原始接触计数、1800 秒和工具停止反馈。'),
        ('include/pose_jump17.hpp','inline bool pose_jump','C++ 定位监督：在同一时刻车体位置比较新旧全局变换，避免原点距离放大角度修正。'),
        ('src/supervisor17.cpp','void mapEvidence','C++ 地图匹配观测、GNSS 交叉检查、拒绝真实跳变。'),
        ('src/supervisor17.cpp','void tick','C++ 全局来源、新鲜度、静止恢复和唯一 map→odom 发布。'),
        ('scripts/matrix17.py','def make','Python 冻结六簇×十种子，以及连接路、故障和消融要求。'),
        ('scripts/checkpoint17.py','def commit','Python 哈希绑定与原子提交；指标不一致、重复成绩或证据修改均拒绝。'),
        ('scripts/stage17.py','def main','Python 串行仿真、分片、恢复、前置验证、超时和本会话收尾。'),
        ('scripts/overlay17.py','def build','Python 对原 S14 做精确匹配的独立适配，并保存每次执行源码快照。'),
        ('scripts/temporal_join17.py','def take','Python 栅格与健康状态按相同时间戳连接，拒绝过期/未来观测。'),
        ('scripts/inspection17.py','def execute','Python 长路分段、原路径后缀恢复、参考路径与在线规划动作选择。'),
        ('scripts/inspection_budget17.py','def estimate','Python 按已完成子段与当前 Nav2 路径估计剩余工时；已流逝时间不算完成进度。'),
        ('scripts/inspection_audit17.py','def audit','Python 离线核对真实路段终点顺序和整个车身的授权区域约束。'),
        ('scripts/map_memory17.py','class MapMemory','Python 固定地图坐标系保存已确认障碍，避免里程计漂移拖出虚假占用；未知不能清障。'),
        ('scripts/dynamic_envelope17.py','class Envelope','Python 保存已观测运动轨迹的有界占用，45 秒过期；无仿真对象身份或未来真值输入。'),
        ('scripts/geometry_plan17.py','def qualify','Python 独立几何变体资格；洞同时从运动域扣除，不能把仅作业面积挖洞当成物理可通行。'),
        ('scripts/resume17.py','def restore_mission','Python 验证已确认工具账本，恢复已完成任务与残余，不重复计信用。'),
        ('scripts/evaluate17.py','def evaluate','Python 原始分母、独立域真值插值、定位分位数、唯一出口和停止审计。'),
        ('scripts/statistics17.py','def paired','Python 只有同场景/种子/任务/限制才配对；缺项保留。'),
        ('scripts/audit17.py','def audit','即使回执为 PARTIAL，仍核查所有证据、源码和前置依赖哈希；不改变验收状态。'),
        ('scripts/report17.py','def aggregate','Python 聚合 60 个固定案例及 R01–R08，缺证据不能成为 PASS。'),
        ('scripts/partition17.py','def run','离线分簇实际对照；同输入生成路线，区分几何合格与名义工时合格。'),
        ('scripts/faults17.py','def assess','故障实验：独立核实故障暴露、预期响应和安全；仍需完整运行矩阵。'),
        ('scripts/visual_gate17.py','class Window','视觉候选：有效跟踪的相对增量、时钟/丢失清空、协方差门控，无额外 TF。'),
        ('scripts/coverage_compare17.py','def choose','在线点服务候选：从当前估计位置选最近未服务点，重新等待匹配的 BT 指令。'),
        ('scripts/risk_adapter17.py','def mission','仅开发候选：复用 S15 风险恢复，明确排除它的两点测试任务替换。'),
        ('scripts/prediction_cost17.py','def apply','仅开发候选：将预测占用加入软代价，保留未知格和真实障碍。')]
    lines=['# S17 核心代码导读','','本目录实现实验执行和独立评分，并包含为本轮暴露问题增加的隔离修正。ROS 2、Nav2、robot_localization、RTAB-Map 和 Gazebo 的上游实现没有被冒称为本阶段原创。候选或实验脚本存在不代表对应试验通过。','','## 本版本的实际入口与行号','']
    for rel,needle,description in references:
        p=PKG/rel;hits=[i for i,line in enumerate(p.read_text().splitlines(),1) if needle in line]
        if not hits:raise ValueError('DOC_REFERENCE_MISSING '+rel+' '+needle)
        lines.append(f'- [{rel}:{hits[0]}]({p}:{hits[0]})：`{needle}`。{description}')
    lines+=['','## 数据链与责任边界','','```text','带误差传感器 → 轮速/IMU/激光局部里程计 → 冻结地图定位与 GNSS 监督','               └→ 三维地形/目标跟踪 → 时间戳连接 → 二维代价','固定作业清单 → 可解释 BT → 唯一导航动作 → 唯一底盘/工具出口','实际工具反馈 → 幂等账本 → 完成与原始残余','隔离真值域 120 → 独立审计 → 指标/检查点 → 60 例矩阵与 R01–R08','```','','`FollowPath` 不会自动替长路径重新选择全局绕行；`NavigateToPose` 使用在线全局规划器。S17 巡检子段使用 `NavigateToPose`，当前代价地图决定绕行；真实路段终点保持不变。割草工作段仍必须沿已验证的作业路径，并由有效工具足迹结算。','','C++ 学习例直接调用生产审计与跳变函数；Python 学习例调用生产统计与时间连接函数。教学输入是合成的，不能替代仿真成绩。编译运行命令和用途见 [教学说明]('+str(BASE/'lessons/S17/README.md')+')。','']
    (docs/'CODE_WALKTHROUGH.md').write_text('\n'.join(lines))
    records=[]
    for p in sorted(RESULTS.glob('pilot_*/metrics17.json')):
        m=read(p);records.append((p.parent,m))
    text=['# S17 当前执行结果','',f'更新时间：{datetime.datetime.now().astimezone().isoformat()}。本页只引用实际输出；未执行的矩阵项保持 NOT_RUN。','','目前尚无完整六簇×十种子正式成绩，不能声明 S17 PASS，也不能进入 S18。地图资格、组件测试或短程演示不替代正式矩阵。','','## 已完成的独立工作','','- A/B 16 个测站资格通过；连接路 11 个建图站与 4 个独立站完成，独立 RMSE 0.01947 m。','- 新连续 atlas/plan 已建立，原数据库保持不变；B1/B2/B3 离线路由在测量占据图上重新验证，名义工时约 1040.70 / 1277.15 / 1086.41 秒。','- 54 组离线分簇对照、18 个同输入配对已实际执行。A 各组合可行；B 基础形状中 METIS 时间闭环为 3/3 可行；B 凹口/含洞形状尚未达到所有名义工时资格。','- 实际 A2 重启演示曾从 1/22 恢复至 2/22，未重复派发已确认目标；该演示使用此前 S17 区域版，不能冒充新连续版完整簇。','- 定位跳变几何、栅格时间连接、路径后缀恢复、检查点、教学例已有实际组件检查；最新统一检查详见 component_contracts。','','## 实际运行开发记录','','下表全部使用开发种子，完整道路与短程候选必须按各例 spec/inspection_contract 区分，均不计入 60 次正式成绩。','','| 案例 | 结果 | 仿真秒 | 位置 P95 m | 障碍接触采样数 | 原因 |','|---|---|---:|---:|---:|---|']
    for p,m in records:
        q=m.get('position_error_m') or {};contacts=sum(m.get('monitor',{}).get('obstacle_contacts',{}).values())
        text.append(f"| [{p.name}]({p/'metrics17.json'}) | {m['status']} | {m['sim_seconds_including_stop']:.3f} | {q.get('p95',float('nan')):.5f} | {contacts} | {m.get('reason')} |")
    text+=['','完整道路试验 `pilot_connector_diff_20261112_232123` 已记录车体/工具与 `ped_1` 的实际接触，因此是 FAIL。不能只报告它的定位与停车门限通过。其后短程基线和 IMM 等候候选为安全 PARTIAL，尚不能证明完整通行。预测软代价候选 `pilot_connector_diff_20261113_000717` 同样记录实际接触，位置 P95 约 0.255 m，已拒绝启用；正式默认预测/缓存不变。','','## 有界链路开发检查','']
    for folder in sorted(RESULTS.glob('study_probe_*')):
        if not (folder/'metrics17.json').exists():continue
        m=read(folder/'metrics17.json');measured=m.get('measurement',m)
        line=f"- [{folder.name}]({folder/'metrics17.json'})：实验结果 {m['status']}；任务完成 {m.get('task_completed',measured.get('complete'))}，安全 {m.get('safety')}。"
        if 'visual_frames' in m:line+=f" 双目输出 {m['visual_frames']} 帧、有效相对增量 {m['visual_increment_frames']} 帧；定位 P95 {m['position_error_m']['p95']:.5f} m。"
        if m.get('fault')=='sensor_delay':line+=f" 延迟帧 {m['detail']['delayed_frames']}，故障暴露核实 {m['exposure_verified']}，预期响应核实 {m['expected_response_verified']}。"
        text.append(line)
    text+=['','上述试验都是开发种子，不计正式矩阵。首次延迟故障试验不能证明注入与停车的因果响应，未计通过；新实现要求连续行驶后注入，并以入队时间检查响应窗口。定位消融共用速度上限和降级策略，GNSS 缺失会触发较低速度，因此不能把误差差异只归因于新增传感器。','','多次运行记录到 Gazebo server 收尾退出码 `-11`；退出码已公开，原因仍需诊断。“没有残留进程”和“工具已关闭”不等于所有进程正常退出。','','## 动态交互故障定位','','失败候选首次接触发生于 215.102 仿真秒。独立真值记录显示，接触前 0.5 秒窗口内车辆位置保持不变；最后一条底盘门控记录为零速度且已确认静止。原 S02 ScriptedMotion 插件沿指定周期轨迹持续施加行人速度，没有对机器人避让的行为。由这些记录推断，仅停车不足以避免该场景的接触；需要进一步验证提前选择等待位置、时空绕行以及具有交互反应的行人场景。此解释不豁免接触，原例仍是 FAIL；不能把行人删掉或降低接触标准来通过。','','## 仍未满足','','- 六簇各十次完整任务及对应有效作业/残余：NOT_RUN。','- 两种底盘完整连接路：尚未全部通过。','- 凹口/含洞物理场景的独立可执行资格及闭环抽样：尚未完成；离线任务多边形对照不能替代。','- 控制器、定位输入、预测/记忆、在线覆盖全部配对闭环：尚未完成。','- 十类规定故障的完整暴露与响应验收、当前版重启合同：尚未完成。','','这些缺项是自动验收与实际通行证据的缺口，不是新增的人工批准门槛。修复后仍须执行正式矩阵及规定消融，不能把已有开发测试换名记入成绩。','']
    (docs/'RESULTS.md').write_text('\n'.join(text))
    (docs/'MANUAL_CHECK.md').write_text('''# S17 手工验收命令

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
''')
    print(docs)

if __name__=='__main__':generate()
