# S07 上游接口依据

本机实际二进制版本在 `config/s07_dependency_lock.json`；生产使用已安装 RTAB-Map / ROS Jazzy，不新建第二份上游源码实现。

- ROS wrapper：复用 `references/S06/CoreWrapper.cpp` / `CommonDataSubscriber.cpp`，来源为 introlab/rtabmap_ros 的 0.23.13 标签。核对定位观测时间戳、实际 covariance、publish_tf、ExactTime 和 QoS 参数。
- 主库参数：本机 `/opt/ros/jazzy/include/rtabmap-0.23/rtabmap/core/Parameters.h`。核对只读 LOCALIZATION、局部合并参考使用优化位姿、整图参考和法向缓存的参数。
- 主库行为参考：[Memory.cpp](https://github.com/introlab/rtabmap/blob/master/corelib/src/Memory.cpp)，2026-11-09 查阅。多扫描邻近配准内部提高 correspondence ratio 的行为，也在本机实际 debug 日志中观察到。master 是解释性参考，不宣称其与锁定包逐字相同。
- 原 S06 的 `Rtabmap_upstream_reference.cpp` 同样是已注明来源的 master 参考。冻结数据库前后哈希、实际节点重启、实际配准和停车结果均由本机运行验证，不仅据源码推断。

最后采用优化局部子图 + 受限精配准。整图配准虽然在开发中匹配正确，但重算及整图查询代价偏大，保留为开发对照结论，不作为生产默认。
