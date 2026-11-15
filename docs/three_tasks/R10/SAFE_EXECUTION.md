# B1 新安全执行版本与原作业残余

本版本来自用户明确确认“生成新的安全执行路径版本，保留原版本和残余记录”。原 R09 路径仍有 34 段，原地图与原文件不修改。离线检查发现其中第 20、25、32 段终点的整车轮廓与 10° 坡面相交，而当前地形准入上限为 5°。原参考路径保持 PARTIAL；新路径的通过不等于原 34 个端点全部到达。

## 原分母与新路径

- 原簇面积：195.9943834 m²，保留 `area_map` 与 `work_area_m2`。
- 原条带目标并集：188.3918017 m²，原 9 条带的目标多边形全部保留。
- 当前 V3 生成路径：10 个作业片段、37 个停刀连接段，共 47 段；约 228.03 m。按 0.25 m/s、每段 5 s 附加时间及 100 s 余量估算约 1247.1 仿真秒。该估算不替代原 1800 s 实际任务门限。
- 坡面禁行层是明确新增的静态规划先验：原世界已存在的坡面外接矩形加 0.60 m 缓冲。它不被描述为本次实时传感器发现，也不修改冻结定位数据库。这个禁行多边形也进入在线风险边界层；V3 保留 V2 的此边界，不改变传感器地形门限。
- 整车轮廓、原 0.10 m 先验边界余量、5° 坡度门限、转弯曲率、定位、停车与工具反馈门限保持原值。V3 规划包络保留前后至少各 0.70 m，横向额外空间由车身膨胀、栅格和位姿设计扰动计算，当前为左右各 0.75 m；物理车身本身不变。

V1 的 46 段路径及生成器保留在 `delivery/r10_completion_20261115/safe_execution_v1/`。它的五段启动故障测试通过，但完整运行 `mowing_20261115_033846_289066` 在完成 7/46 段后，因实时碰撞检查与有界恢复失败停车。记录在仿真 385–416 s 内的近车危险栅格样本最高约到世界坐标 y=47.393 m，超过坡体几何边缘 y=47 m；V1 仅有 0.15 m 坡体缓冲，不能充分容纳这一差异。V2 采用四个 0.15 m 栅格的规划缓冲，同时保留原车身规划余量。0.60 m 是基于该次观测的保守规划选择，并非已证明能覆盖所有感知误差的上界；仍需新版完整实测。V1 的五段 PASS 不能继承为 V2 PASS。

V2 完整案例 `mowing_20261115_043537_296551` 同样完成 7/45 段后停车。此次完整栅格表明，距车身 0.7 m 内的 59 个危险风险单元全部位于已声明禁行边界之外，不能把这一圈单元直接当成传感器新检测的坡面。车辆相对原路径的中心偏差约 0.0265 m、朝向偏差约 0.028 rad；旧横向余量 0.20 m 本身不足以兼顾 Nav2 的 0.10 m 车身膨胀和 0.15 m 栅格的半对角线 0.106 m，再容纳转弯跟踪变化。继续只扩大禁行多边形，会连同在线危险边界一起向外移动。

V3 因此保留 V2 的禁行先验，只增加独立的规划车身包络。对上轮保存栅格作局部离线检查，坡面附近 2269 个新路径位姿的 Nav2 膨胀车身加栅格余量均未碰到已记录危险单元。未知区域和后续观测仍须在线验证，不能把这项检查称为整簇 PASS。V2 配置与结果另存于 `delivery/r10_completion_20261115/safe_execution_v2/`。

原 `B1:segment:33` 在 V3 包络下没有至少 1 m 的连续原条带片段，已显式记入 `omitted_original_work`，其 8.3023543 m² 原目标仍保留。这里没有声称该区域全局无解；只是当前固定条带裁剪方法不执行这一项。最终残余仍按全部原目标的几何并集减实际反馈并集计算，不能简单把各条带面积相加。

## 算法与核心代码

`planning_envelope_r10.py` 先对 Nav2 已膨胀的车身求 ±0.12 rad 朝向扰动下的角点极值，再加入 0.25 m 位置设计余量及栅格半对角线，横向按 0.05 m 向外取整。当前额外宽度为每侧 0.75 m。这是离线设计包络，不是新的到达容差，也不是对全部未来跟踪误差的认证上界。`safe_plan_r10.py` 加载时重算包络，并绑定该算法文件哈希。

`prepare_safe_plan_r10.py` 逐点检查原条带的扩展车身轮廓，截取长度至少 1 m 的连续安全片段；每片段保留其原目标多边形、原任务键和原工具目标，不把被截掉的区域从分母移除。若某条带完全无法执行，仍存入 `original_work_segments` 与遗漏原因。

C++ `r10_backend/safe_route.cpp` 使用 OMPL 的 Reeds–Shepp 状态空间，允许前进和倒车但受相同最小转弯半径约束。优先检查直接连接；不通过时使用有界 RRTConnect 搜索。每个状态都用 Boost.Geometry 检查扩展整车轮廓是否包含于允许区域。仅接收精确到达的解，最多三次不同种子、每次三秒；搜索失败不证明全局无路。

曲线按每个 Reeds–Shepp 基元采样，并保留精确换向尖点。不能仅按全路径等间距插值：跨过前进/倒车尖点的弦可能产生假的曲率超限。输出按实际挡位变化拆段；`mission_mowing_r10.py` 每次换向先取消动作、确认底盘与刀具停止，再请求对应方向的 RPP 跟踪。

Python `safe_plan_r10.py` 在加载时复核原文件哈希、生成器/原生程序哈希、曲率、段间连续性、扩展车身轮廓、目标多边形、时间预算与挡位声明。运行任务还必须通过当前传感器代价图和唯一底盘出口，离线可行不直接授予运动权限。

`work_identity()` 把同一条带的多个片段映射到同一个原始作业键与目标。工具账本仍按实际刀具反馈产生的几何并集记账；重复片段不会重复计数，停刀连接不记作业量。`requested_target()` 在完整 B1 评估时取全部原条带并集，包含完全无法进入的区域；残余等于原目标并集减实际确认并集。

新版本的 `execution_segments` 与 `reference_original_segments` 分别保存。兼容旧报告的 `original_segments` 字段表示当前执行版本总段数，查看原参考时必须使用 `reference_original_segments=34` 和 `original_reference_status=PARTIAL`。

官方算法接口：[OMPL Reeds–Shepp 状态空间](https://ompl.kavrakilab.org/classompl_1_1base_1_1ReedsSheppStateSpace.html)、[OMPL RRTConnect](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html)。本机实现使用已安装的 OMPL 1.7 接口；官网较新 API 命名可能不同。

## 手工核查

先退出现有仿真，同一时刻只启动一套。下列检查不生成新的规划版本：

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
optimization_v2/venvs/scene/bin/python -m unittest discover \
  -s optimization_v2/ws/src/task_platform/tests -p 'test_safe_plan_r10.py' -v
xdg-open optimization_v2/three_tasks/results/R10/safe_plan_comparison.png
```

测试应确认原分母没有减小、断开的条带共用原工作键、完整评估保留未执行条带。图中绿色为作业片段、蓝色为停刀连接、红色为新增已知坡面禁行层；图是规划检查，不是执行成功证明。

```bash
bash optimization_v2/three_tasks/advance.sh R10 test record_mowing_full_B1
bash optimization_v2/three_tasks/advance.sh R10 test record_mowing_startup_perception_loss
bash optimization_v2/three_tasks/advance.sh R10 check
```

第一条真实运行当前完整 B1；第二条运行前五段并在 START_WORK 实际中断感知，检查取消、两秒内实测停机、恢复和同键反馈。每次实际运行使用独立结果目录。`analysis_mowing_r10.json` 分别给出执行到达、接触、定位、反馈面积和残余；录像验收还要求完整时间线和图像新鲜度。

原参考保存于 `editions/R09/mission.json`；本次变更前快照与 PARTIAL 指针另存于 `delivery/r10_completion_20261115/original_B1_reference/`。原 S17/S18 仍为 PARTIAL，不执行六十次矩阵。

新路径实际运行中发现的车体回波与停车定位恢复问题另见 [采集与恢复专项](CAPTURE_AND_RECOVERY.md)；规划几何检查不能替代这些在线检查。
