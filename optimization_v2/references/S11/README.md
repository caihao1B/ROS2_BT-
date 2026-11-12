# S11 来源与适用范围

2026-11-10 核验：
- [METIS 官方手册 5.1](https://papers.karypis.org/glaros/files/sw/metis/manual.pdf)：CSR、节点/边权、k-way、CONTIG/SEED/UFACTOR。实际链接本机 libmetis 5.1.0，不安装 master。
- [KarypisLab/METIS](https://github.com/KarypisLab/METIS)：官方实现与 Apache-2.0 许可。使用系统动态库，项目只新增参数/输入检查适配。
- [Shapely 2.1.2 STRtree](https://shapely.readthedocs.io/en/2.1.2/strtree.html)：空间候选邻居；实际邻接仍以公共边长检查。
- [原作者 DARP](https://github.com/athakapo/DARP)：只借鉴适用条件，未下载/复制/执行该算法；接口状态为 NOT_EXECUTED。

本地主项目已核对 `field95/planning.py`、`field95/plan_cli.py`、`offline_coverage/planner.py`。复用其“工作域/通行域分开、版本绑定、不可行不可冒充覆盖”的设计思想；旧工具尺寸、旧地图与历史成果不作为新车证据。没有重新依赖新加卷。新的分簇器未复制旧路线生成器，也未在 S11 调用 Fields2Cover 或 OR-Tools；留给 S12。

几何变体参考已验收 S02 的公开边界规则，直接处理 maps/prior；没有导入含仿真真值的世界生成器。
