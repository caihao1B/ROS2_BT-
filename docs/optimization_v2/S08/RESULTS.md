# S08 实测结果

本轮 `test_20261109_225727_808647`：**PASS**。人工状态 NOT_CHECKED。
可以执行下一步 S09：S08 四项自动验收通过，S07 有效；全地图精细作业仍需修订已知局部地图畸变，不自动执行 S09

[交互报告](/home/ceihao/robot_project/optimization_v2/results/S08/test_20261109_225727_808647/index.html) · [原始指标和命令](/home/ceihao/robot_project/optimization_v2/results/S08/test_20261109_225727_808647/result.json)

全部坐标在冻结 map 下评分，不拟合真值轨迹。基线为 A 区诊断运动，不能推广为全地图商业性能。

| 案例 | 位置 P95 / m | 航向 P95 / rad | 准入时间占比 | 通过 |
|---|---:|---:|---:|---|
| normal | 0.0499637248443744 | 0.004212602192743264 | 1.000 | True |
| gnss_outage | 0.0741784562450458 | 0.003940048323896694 | 0.658 | True |
| false_fixed_and_loss | 0.06091989044753606 | 0.00421006340191834 | 0.518 | True |
| map_degradation | 0.07324793624912913 | 0.009058383857758123 | 0.749 | True |

停车实测 `false_fixed_and_loss`：`{"trigger_t": 48, "gate_t": 48.169, "moving_before_fault": true, "fault_to_stop_s": 0.2700000000000031, "passed": true, "delay_sim_s": 0.1010000000000062, "distance_m": 0.0040792067341079066, "final_v_max": 8.345708591385272e-14, "final_w_max": 3.552713678800501e-15, "thresholds": {"v": 0.01, "w": 0.02, "delay": 2.0}}`


停车实测 `map_degradation`：`{"trigger_t": 40, "gate_t": 41.637, "moving_before_fault": true, "fault_to_stop_s": 1.7310000000000016, "passed": true, "delay_sim_s": 0.0940000000000012, "distance_m": 0.0028192065510467784, "final_v_max": 4.224794955374194e-14, "final_w_max": 0.0, "thresholds": {"v": 0.01, "w": 0.02, "delay": 2.0}}`


[默认 GUI 演示与实时话题检查记录](/home/ceihao/robot_project/optimization_v2/results/S08/manual_entrypoint_validation.json)：由执行代理核验，用户人工状态仍为 NOT_CHECKED。

故障阶段同样报告误差；另存全部已发布位姿的 P95，避免只显示准入窗口。未初始化时没有位姿，记录首次准入时间而不伪造零误差。地图失配为明确的观测接口注入；不是复杂视觉误匹配发生概率的评估。

冻结地图的连接路和 B 区在 S07 已发现空间不均匀偏差，单个刚体配准无法消除。后续在这些区域开展精细作业前需要地图质量修订和版本重新验收。本步未修改 S01–S07 源码和回执。

自动检查：`{"S08-C01": "PASS", "S08-C02": "PASS", "S08-C03": "PASS", "S08-C04": "PASS"}`。每例 runtime.json 保存真实 PID、命令、退出码、地图前后哈希；s08_inputs.json 绑定实际捕获版本。测试停止仅收尾本会话。

参考实现使用已安装 Jazzy/RTAB-Map/robot_localization，无新增系统包。实际 dpkg 版本在 environment_verified.json；C++/Python 教学、真实节点组件测试、浏览器交互检查的退出码在 commands.jsonl.gz。
