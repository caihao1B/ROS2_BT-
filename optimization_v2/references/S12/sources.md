# S12 依赖与参考

本步实际使用已有系统安装，未新增系统包。精确版本与头文件哈希见 `optimization_v2/config/s12_dependency_lock.json`。

- Fields2Cover 2.1.1 / BSD-3-Clause，官方项目 https://github.com/Fields2Cover/Fields2Cover ，教程 https://fields2cover.github.io/source/tutorials/path_planning.html 。`/opt/ros/jazzy/include/fields2cover.h` 与实际 C++ 编译为 API 基准。
- OR-Tools vendor 9.9.1 / Apache-2.0，官方 TSP 文档 https://developers.google.com/optimization/routing/tsp 。使用已有 C++ 库，不安装 Python 版。
- METIS 5.1 / Apache-2.0，复用 S11 分区及连通修复，不拷贝另一套实现。
- 本项目 `offline_coverage/motion.py`：复用已存在的 6 类前进 Dubins 解析候选；生产包络检查二次准入。
- `field95/planning.py`、`field95/plan_cli.py`、`field95/test_offline.py`、`field95/test_strips.py`：只参考原任务分母、转向停车和工具协议，旧 dino 参数与旧实验结果不等于 S12 合格。本步未重跑需要历史 fixture 的旧条带验收，也未声称其通过；被复用的运动数学由本步实际测试覆盖。
