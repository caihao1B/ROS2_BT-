# S15 显示配置复验

正式 15 项无界面矩阵保持原始文件和源码哈希。随后发现 RViz 观察中心为 map 坐标，而显示参考系为 odom，导致初始视图偏离车辆。

仅将 `config/rviz/prediction.rviz` 的 `Fixed Frame: odom` 改为 `map`；预测、风险和控制仍使用 odom，不改变算法、场景、数据、阈值、任务和地图。

实际 GUI 复验：`demo_20261112_174641_2370190`。任务与安全检查通过；助手检查真实截图中的点云、跟踪框、未来分支和概率。用户人工验收仍为 NOT_CHECKED。

新回执组合原始无界面矩阵与这次显示复验，不宣称完整矩阵重新执行。`scripts/revalidate_s15_view.py` 验证唯一字节差异、全部原始证据、原有源码、S14 依赖及实际 GUI 结果后签发。

版本证明：`optimization_v2/results/S15/rviz_frame_regression_20261112/verification.json`。
