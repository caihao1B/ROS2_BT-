# S06 本轮实测结果

运行 `test_20261109_203204_512414`：**PASS**。可以执行下一步 S07：S06 全部检查通过且 S05 有效；不自动执行。人工查看 NOT_CHECKED。

[交互三维报告](/home/ceihao/robot_project/optimization_v2/results/S06/test_20261109_203204_512414/index.html) · [完整指标](/home/ceihao/robot_project/optimization_v2/results/S06/test_20261109_203204_512414/result.json)

- S06-C01：PASS
- S06-C02：PASS
- S06-C03：PASS
- S06-C04：PASS

| 案例 | 通过 | 关键帧 | 视觉／邻近回环 | 优化前／后 ATE m | 对齐 P95 前／后 m | 动态点剔除率 |
|---|---|---:|---:|---:|---:|---:|
| mapping_full | PASS | 676 | 4/291 | 12.170/0.791 | 0.996/0.163 | 78.4% |
| mapping_loop | PASS | 68 | 6/14 | 0.279/0.053 | 1.046/0.250 | 78.6% |
| weak_texture | 诊断通过；建图不合格 | 30 | 0/0 | 0.952/2.197 | N/A/N/A | 75.1% |

- `mapping_full`：[metrics.json](/home/ceihao/robot_project/optimization_v2/results/S06/full_route_04/metrics.json)，[真实进程命令和退出码](/home/ceihao/robot_project/optimization_v2/results/S06/full_route_04/runtime.json)。
- `mapping_loop`：[metrics.json](/home/ceihao/robot_project/optimization_v2/results/S06/test_20261109_203204_512414/mapping_loop/metrics.json)，[真实进程命令和退出码](/home/ceihao/robot_project/optimization_v2/results/S06/test_20261109_203204_512414/mapping_loop/runtime.json)。
- `weak_texture`：[metrics.json](/home/ceihao/robot_project/optimization_v2/results/S06/test_20261109_203204_512414/weak_texture/metrics.json)，[真实进程命令和退出码](/home/ceihao/robot_project/optimization_v2/results/S06/test_20261109_203204_512414/weak_texture/runtime.json)。

补充复核：默认 GUI 演示完成；最终 RViz 显示配置已在实际窗口查看；空重叠点云统计边界回归通过；报告交互重新检查。人工状态仍指用户尚未手工验收。
[复核记录](/home/ceihao/robot_project/optimization_v2/results/S06/test_20261109_203204_512414/final_validation.json) · [最终 RViz 截图](/home/ceihao/robot_project/optimization_v2/results/S06/test_20261109_203204_512414/rviz_verified.png) · [无损日志整理](/home/ceihao/robot_project/optimization_v2/results/S06/test_20261109_203204_512414/retention_summary.json)

正常案例逐项门槛在 metrics.json；原场景弱纹理案例只考察诊断、数据链和安全行为，不能称为成功建图。

ATE 使用启动 5 秒时的单一坐标原点；没有逐段拟合或真值初始化。回访点云误差仅代表该配对的可重叠可见表面，并非整张地图每处精度。动态残留标签仅由离线评分器使用；线上没有仿真物体 ID。

原始 S01–S05 未更改。地图仍处于 MAPPING；S07 的冻结、不同初始位置重定位未在本步宣称通过。

生产参数、源代码、二进制在采集开始时记录 capture_inputs.json。导出会拒绝采集期间变化的运行输入。运行命令包含种子 41，完整导出命令和退出码见 export_commands.jsonl.gz。

观测 JSONL 无损压缩；真实数据库与 PLY 保留。短程调试得到的初期失败见 development_summary.json，不作为本轮通过数据。
