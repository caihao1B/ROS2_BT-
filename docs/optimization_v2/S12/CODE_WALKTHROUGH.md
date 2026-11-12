# S12 核心代码导读

主调用链：`stepctl → run.sh → stage12.main → plan_zone → plan_spray/plan_mowing → backend(C++) → segment/profile → verify_selected → bind_mission → export_plan → Chrome 检查 → 发布`。

## Python 生产实现

统一入口、预算/停止、真实检查后发布与哈希回执。
- [`main`](../../../optimization_v2/ws/src/coverage_planner/scripts/stage12.py#L49)：scripts/stage12.py:49
- [`command`](../../../optimization_v2/ws/src/coverage_planner/scripts/stage12.py#L17)：scripts/stage12.py:17
- [`receipt`](../../../optimization_v2/ws/src/coverage_planner/scripts/stage12.py#L41)：scripts/stage12.py:41

显式任务版本、实际工时近似归因、连通边界交换与有界重算。
- [`resized_domain`](../../../optimization_v2/ws/src/coverage_planner/scripts/planning12.py#L7)：scripts/planning12.py:7
- [`rebalanced`](../../../optimization_v2/ws/src/coverage_planner/scripts/planning12.py#L14)：scripts/planning12.py:14
- [`plan_zone`](../../../optimization_v2/ws/src/coverage_planner/scripts/planning12.py#L37)：scripts/planning12.py:37

障碍可见图/Dijkstra→两种顺序→停靠/旋转/直线路段→按时间选择。
- [`visibility`](../../../optimization_v2/ws/src/coverage_planner/scripts/spray12.py#L7)：scripts/spray12.py:7
- [`nearest_2opt`](../../../optimization_v2/ws/src/coverage_planner/scripts/spray12.py#L38)：scripts/spray12.py:38
- [`build_route`](../../../optimization_v2/ws/src/coverage_planner/scripts/spray12.py#L52)：scripts/spray12.py:52
- [`plan_spray`](../../../optimization_v2/ws/src/coverage_planner/scripts/spray12.py#L70)：scripts/spray12.py:70

全区条带相位、簇裁剪、两种真实 F2C 曲线、工具足迹并集与残余。
- [`connection`](../../../optimization_v2/ws/src/coverage_planner/scripts/mowing12.py#L11)：scripts/mowing12.py:11
- [`plan_mowing`](../../../optimization_v2/ws/src/coverage_planner/scripts/mowing12.py#L25)：scripts/mowing12.py:25

生产验证：完整包络、非完整运动、加减速、停止转向、有效作业几何与时间。
- [`normalize_poses`](../../../optimization_v2/ws/src/coverage_planner/scripts/motion12.py#L4)：scripts/motion12.py:4
- [`swept_check`](../../../optimization_v2/ws/src/coverage_planner/scripts/motion12.py#L19)：scripts/motion12.py:19
- [`profile`](../../../optimization_v2/ws/src/coverage_planner/scripts/motion12.py#L33)：scripts/motion12.py:33
- [`segment`](../../../optimization_v2/ws/src/coverage_planner/scripts/motion12.py#L71)：scripts/motion12.py:71
- [`summarize`](../../../optimization_v2/ws/src/coverage_planner/scripts/motion12.py#L82)：scripts/motion12.py:82
- [`effect_polygon`](../../../optimization_v2/ws/src/coverage_planner/scripts/motion12.py#L87)：scripts/motion12.py:87

连接路插值与重复点清理、停车点进出、两种底盘双向路线。
- [`road_geometry`](../../../optimization_v2/ws/src/coverage_planner/scripts/connector12.py#L8)：scripts/connector12.py:8
- [`approach`](../../../optimization_v2/ws/src/coverage_planner/scripts/connector12.py#L17)：scripts/connector12.py:17
- [`plan_connector`](../../../optimization_v2/ws/src/coverage_planner/scripts/connector12.py#L31)：scripts/connector12.py:31

整区不可行、测量配准、原始数据重验、计划与输入绑定。
- [`original_infeasible`](../../../optimization_v2/ws/src/coverage_planner/scripts/contracts12.py#L6)：scripts/contracts12.py:6
- [`frame_binding`](../../../optimization_v2/ws/src/coverage_planner/scripts/contracts12.py#L14)：scripts/contracts12.py:14
- [`verify_selected`](../../../optimization_v2/ws/src/coverage_planner/scripts/contracts12.py#L28)：scripts/contracts12.py:28
- [`bind_mission`](../../../optimization_v2/ws/src/coverage_planner/scripts/contracts12.py#L56)：scripts/contracts12.py:56
- [`check_binding`](../../../optimization_v2/ws/src/coverage_planner/scripts/contracts12.py#L65)：scripts/contracts12.py:65

GeoJSON/时间表/完整计划及文件哈希发布检查。
- [`export_plan`](../../../optimization_v2/ws/src/coverage_planner/scripts/products12.py#L6)：scripts/products12.py:6
- [`validate_products`](../../../optimization_v2/ws/src/coverage_planner/scripts/products12.py#L27)：scripts/products12.py:27

实际狭窄回转和不可达测试；内存版本注入，不破坏权威文件。
- [`run_faults`](../../../optimization_v2/ws/src/coverage_planner/scripts/faults12.py#L8)：scripts/faults12.py:8

同一份真实路线/时间用于浏览器动画，没有编造模拟轨迹。
- [`render`](../../../optimization_v2/ws/src/coverage_planner/scripts/report12.py#L5)：scripts/report12.py:5

## C++ 生产实现

C++ 负责实际第三方库调用，Python 负责任务合同、几何独立重验、时间反馈与输出。JSON 是进程间接口；不会把某个字段写成 PASS 就跳过检查。

- [`cell`](../../../optimization_v2/ws/src/coverage_planner/src/planner_backend.cpp#L11)：GeoJSON 外环/孔洞转 F2CCell。
- [`swaths`](../../../optimization_v2/ws/src/coverage_planner/src/planner_backend.cpp#L17)：ConstHL、NSwath 搜索方向、Boustrophedon 分解/条带排序；全区统一相位。
- [`turns`](../../../optimization_v2/ws/src/coverage_planner/src/planner_backend.cpp#L28)：F2C DubinsCurvesCC 与普通 Dubins；分辨率/半径固定，输出位姿由 Python 再验。
- [`tsp`](../../../optimization_v2/ws/src/coverage_planner/src/planner_backend.cpp#L44)：真正的 OR-Tools RoutingModel，毫米代价闭合路线，2 s/1000 解上限，无全局最优承诺。
- [`main`](../../../optimization_v2/ws/src/coverage_planner/src/planner_backend.cpp#L54)：解析请求、派发模式、明确非零失败退出。

## 复用与学习边界

- [S11 partition11.py](../../../optimization_v2/ws/src/coverage_partition/scripts/partition11.py#L43)：连通修复、负载边界交换及独立几何复验；保持原文件不变。
- [已有解析 Dubins](/home/ceihao/robot_project/offline_coverage/motion.py)：连接路备选曲线，仍需通过新物理包络验证。
- [C++ 小例](../../../optimization_v2/lessons/S12/lesson.cpp#L1) 与 [Python 小例](../../../optimization_v2/lessons/S12/lesson.py#L1) 是合成输入上的真实库/生产函数调用，不是完整任务验收。

优先阅读一个喷洒点的 build_route → segment → summarize，再阅读一条割草工作段的 plan_mowing → connection → profile。最后观察 planning12 如何把路线实际模型时间反馈给 S11，而不是把 S11 的面积初估直接当作最终任务时间。

普通 Dubins 的曲率跃迁被转成停车转向事件；profile 的 sample_times 包含各次停车保持时间。未来在线执行器必须确认实测速度为零且转向到位才能继续。任务结束状态必须来自 S10 账本，不来自本步离线面积预测。
