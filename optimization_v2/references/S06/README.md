# S06 上游参考与版本边界

生产使用本机 apt 锁定的 RTAB-Map / rtabmap_ros 0.23.13 二进制，精确 Debian 版本见 `optimization_v2/config/s06_dependency_lock.json`。这些源码副本只供学习和接口核查，不能替代实际运行测试。

- CoreWrapper.cpp、CommonDataSubscriber.cpp：rtabmap_ros 标签 0.23.13。来源 https://github.com/introlab/rtabmap_ros/tree/0.23.13 ，保留原 BSD 许可证头。
- Rtabmap_upstream_reference.cpp：2026-11-09 从 https://github.com/introlab/rtabmap/blob/master/corelib/src/Rtabmap.cpp 获取的上游参考。不是声称与已安装包逐字一致的版本；RTAB-Map 主库的同名 0.23.13 GitHub 标签不可用。保留原 BSD 许可证头，回执哈希绑定所读取副本。
- 实际参数含义以 `/opt/ros/jazzy/include/rtabmap-0.23/rtabmap/core/Parameters.h` 和锁定二进制行为为准。
- Memory::computeTransform 的视觉初值分支在线参考：https://github.com/introlab/rtabmap/blob/master/corelib/src/Memory.cpp 。无初值的全局候选需要视觉验证；已有近邻初值可以进入 ICP。

开发试验的错误回环、长距离竖向漂移和批量补回环失败都另存实测摘要，不能只凭源码预期声称算法有效。
