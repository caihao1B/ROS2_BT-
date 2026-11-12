# S09 外部代码和资产

- NanoDet OpenCV Zoo：https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_nanodet 。ONNX 与未改动 vendor/nanodet.py、LICENSE、README.md 保留，本地模型锁在 config/detector_lock.yaml。
- Walking person v3：https://fuel.gazebosim.org/1.0/OpenRobotics/models/Walking%20person 。fuel_metadata.json 的 license_name 为 Creative Commons Zero v1.0 Universal；模型作者 Marina Kollmitz。原模型由 MakeHuman 创建。引用下载地址及内容哈希见 config/perception_assets.lock.json。
- PCL https://pointclouds.org/documentation/tutorials/planar_segmentation.html ；Nav2 https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_costmap_2d 。使用本机已安装 Jazzy 库，实际版本由 tests/dependencies.py 核验。
- 模型处理：原始许可资产不改写；场景生成时才改 URI、模型名和刚体运动。坡面 OBJ 为项目原创，带三角面与法向，避免物理引擎无效网格。

本步未引入额外系统包；OpenCV 4.6 的 NanoDet 推理经过实际部署和仿真帧测试，非根据上游最低版本字样直接假定兼容。Gazebo 系统 protobuf 与 OR-Tools 导出配置冲突通过本包 CMake 预先查找系统 MODULE 解决，未修改系统库。
