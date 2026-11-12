# S09 实测结果

本轮 `test_20261110_001352_924535`：**PASS**；用户人工状态 NOT_CHECKED。
可以执行下一步 S10：S09 四项自动验收通过，S08 有效；未自动实施 S10，闭环导航由 S13 验证

[离线交互报告](/home/ceihao/robot_project/optimization_v2/results/S09/test_20261110_001352_924535/index.html) · [完整指标](/home/ceihao/robot_project/optimization_v2/results/S09/test_20261110_001352_924535/result.json)

| 案例 | 底盘 | 通过 | 地形 P95/ms | 检测 P95/ms | 清除/s |
|---|---|---|---:|---:|---:|
| low_diff | diff | True | 41.93 | 88.66384095017565 | 2.1000000000000014 |
| low_ackermann | ackermann | True | 37.20 | 91.31154515380327 | 2.6000000000000014 |
| occluded_pedestrian | diff | True | 42.67 | 86.37898104680062 | 2.1000000000000014 |
| camera_loss | diff | True | 44.15 | 89.85481225136027 | 2.5 |

`low_diff` 未通过项：[]


`low_ackermann` 未通过项：[]


`occluded_pedestrian` 未通过项：[]


`camera_loss` 未通过项：[]


GUI 复核发现 S08 诊断点在 A/B 作业区外，已明确增加 S09 诊断通行区并补充实际 Nav2 空闲地面检查，重新运行全部四个场景。原 A/B 边界和冻结地图未修改。车辆驻停进行传感器/代价层验收；未运行 S13 闭环控制。局部窗口随 odom 移动；未知格不自动转为空闲。几何不依赖视觉类别。行人采用 CC0 静态步态网格、真实刚体运动，未模拟人体关节步态。

轻量检测器仍有树木误分类和短时漏识别；语义不是安全放行依据。每例 dynamic 指标公开实际可见帧、识别人帧数、ID 切换。长期遮挡超时后生成新 ID，不宣称永久身份。

S08 已知连接路/B 区地图变形仍存在；本步地形风险在局部 odom 下计算。全地图精细导航仍需后续修订地图并复验。坡度上限沿用 S03 已验证 5°，台阶/粗糙度为保守工程候选，不代表实体底盘爬越认证。

检查：`{"S09-C01": "PASS", "S09-C02": "PASS", "S09-C03": "PASS", "S09-C04": "PASS"}`。命令/退出码见 commands.jsonl.gz；运行 PID、时钟、输入哈希、地图前后哈希见每例 runtime.json / s09_inputs.json。依赖锁与模型哈希已核验；本步未新增系统包。
