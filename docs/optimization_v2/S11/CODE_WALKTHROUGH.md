# S11 核心代码导读

真实调用链：`stepctl → stages/S11/run.sh → stage11.main → load_domain/build_graph → METIS 或 K-means → repair → verify → export_bundle/render → 回执`。本步完全离线，不启动机器人。

## C++：真实 METIS CSR 接口

- [void require](../../../optimization_v2/ws/src/coverage_partition/src/partition_metis.cpp#L12)：输入断言：错误返回非零，不继续调用 C 库。
- [std::vector<idx_t> integers](../../../optimization_v2/ws/src/coverage_partition/src/partition_metis.cpp#L13)：严格整数转换与 idx_t 范围。
- [int main](../../../optimization_v2/ws/src/coverage_partition/src/partition_metis.cpp#L22)：CSR 对称/连通检查、METIS 参数及结果标签检查。
- [int status=METIS_PartGraphKway](../../../optimization_v2/ws/src/coverage_partition/src/partition_metis.cpp#L59)：实际调用系统 libmetis 的 k-way 加权图划分。

## geometry11.py

- [components](../../../optimization_v2/ws/src/coverage_partition/scripts/geometry11.py#L21)：BFS 连通分量，可用于整图或某簇。
- [load_domain](../../../optimization_v2/ws/src/coverage_partition/scripts/geometry11.py#L31)：从公开先验产生完整任务域、方向无关包络筛查与当前限速；不读取世界真值。
- [build_graph](../../../optimization_v2/ws/src/coverage_partition/scripts/geometry11.py#L60)：裁剪 1 m 单元，正公共边邻接，分开计算节点工时与边切割权重。

## partition11.py

- [metis](../../../optimization_v2/ws/src/coverage_partition/scripts/partition11.py#L9)：生成 CSR JSON 并启动真实 C++ 可执行程序。
- [kmeans](../../../optimization_v2/ws/src/coverage_partition/scripts/partition11.py#L21)：固定种子的欧氏 K-means++ 基线。
- [repair](../../../optimization_v2/ws/src/coverage_partition/scripts/partition11.py#L37)：最大分量保留、沿边生长、供体连通约束下的边界交换。
- [verify](../../../optimization_v2/ws/src/coverage_partition/scripts/partition11.py#L80)：独立检查图/几何连通、遗漏、重叠、越界、单元和点归属。
- [solve](../../../optimization_v2/ws/src/coverage_partition/scripts/partition11.py#L104)：拼装初估指标、原始/修复标签、几何与 PROVISIONAL 状态。

## products11.py

- [export_bundle](../../../optimization_v2/ws/src/coverage_partition/scripts/products11.py#L13)：导出完整任务单元和点，不删边界义务；manifest 最后写入。
- [validate_bundle](../../../optimization_v2/ws/src/coverage_partition/scripts/products11.py#L34)：核验文件、输入、源代码和上游回执版本。

## darp_adapter.py

- [export_uniform_grid](../../../optimization_v2/ws/src/coverage_partition/scripts/darp_adapter.py#L4)：仅导出满足均一四邻接假设的请求；拒绝不适用的裁剪/加权场景。

## stage11.py

- [command](../../../optimization_v2/ws/src/coverage_partition/scripts/stage11.py#L14)：记录真实子进程命令/退出码，有界超时和本进程组清理。
- [receipt](../../../optimization_v2/ws/src/coverage_partition/scripts/stage11.py#L27)：写四项检查与源文件/证据哈希；人工状态独立。
- [main](../../../optimization_v2/ws/src/coverage_partition/scripts/stage11.py#L42)：前置、矩阵、逐组重复、可视 QA、候选发布和失败回执。

## 为什么这样组织

节点权重回答“这块任务预计要做多久”；边权回答“尽量不要在哪里切断”。C++ 不理解多边形或阿克曼几何，所以 Python 必须复验。修复只改变单元标签，不改变单元形状或任务域分母。

高频在线反馈账本仍由 S10 实现。S11 生成的是未来任务义务，不写入已经完成的作业账本；不能把分到某簇当作已经喷洒/割草。

代码中 `optimistic_total_lower_bound_s` 的割草数值仅为理想直条带模型基准；详细假设见 TECHNICAL。最终任务耗时字段为 null，不能当作仿真实测。

[C++/Python 教学例及运行命令](../../../optimization_v2/lessons/S11/README.md)：C++ 真正链接 METIS，Python 使用生产图/修复/检查，输入都明确为教学小场景。
