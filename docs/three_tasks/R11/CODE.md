# R11 核心代码

Python `visual_r11.py::assets` 用固定绘制规则生成 512×512 面板纹理，`apply` 仅替换材质，`signature` 比较全部非材质 XML。没有下载或冒用厂商标志。`preview_r11.py` 成对启动真实 Gazebo、只读观察桥和捕获进程，收帧后先暂停物理时钟，再有界关闭自己启动的进程。

本步没有新增 C++ 动力学或控制代码。真实渲染仍由 Gazebo/Ogre，图像桥由 ros_gz_bridge 执行；两者的真实退出码进入报告。`stage_r11.py` 聚合四项契约组件测试和三组性能结果，R10 未通过时整体回执保持 PARTIAL，即使材质组件已经通过。
