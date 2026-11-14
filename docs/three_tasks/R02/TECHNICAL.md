# R02 三车几何与运动学契约

点喷采用 Husky A200 尺寸参考的四轮滑移转向模型，巡迹采用 Jackal 尺寸参考的四轮滑移转向模型，割草采用项目现有 Dino 前轮阿克曼改型的轻量碰撞几何。这里只验证仿真模型，不声称厂家硬件动力学或真实割草认证。

|任务|外包络 长×宽|轴距|轮距|轮半径|转向|
|---|---|---:|---:|---:|---|
|spray|0.990×0.670 m|0.512|0.555|0.1651|四轮滑移转向|
|inspection|0.508×0.430 m|0.262|0.37559|0.098|四轮滑移转向|
|mowing|2.780×2.770 m|1.638|1.800|0.3315|前轮阿克曼，后轴驱动|

Husky 的车体尺寸、轴距与轮距参考[厂商数据表](https://www.clearpathrobotics.com/wp-content/uploads/2013/08/HUSKY_DATA_SHEET-v2_editeda.pdf)和[官方 ROS 描述](https://github.com/husky/husky/blob/noetic-devel/husky_description/urdf/husky.urdf.xacro)。Jackal 参考[官方手册](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/jackal/user_manual_jackal/)和[官方 ROS 描述](https://github.com/jackal/jackal/blob/noetic-devel/jackal_description/urdf/jackal.urdf.xacro)。Husky/Jackal 参考离地间隙分别 0.130/0.065 m，附加工具和传感器另行计算。

Dino 使用本机 `ros2_ws/src/dino_description/models/dino/model.sdf` 的现有前轮转向改型。轴距、轮距、轮径及转向方式可从文件核查；不是依据一个模型名称推断厂家商品结构。其保守车体包络沿用原项目已审计尺度。原文件保持不变。附加 1.6 m 作业宽度的抽象割草工具，不加入机械臂机构；工具底部离地约 0.1 m，不能把车身间隙当成整车越障高度。

阿克曼虚拟转角 0.5 rad 对应后轴中心半径 R=L/tan(δ)≈2.998 m。内外前轮转角分别 atan(Lκ/(1∓Tκ/2))，需要额外检查内轮没有超过 45° 物理限位。整车扫掠外圆半径约 4.91 m，因此完整转圈直径约 9.82 m。6 m 宽连接路能支持沿弯道前进，却不意味着能在路中完成掉头。R03 必须用真实扫掠轮廓设计地头，不能用后轴半径直接代表整车所需空间。

差速近似 v=r(ωR+ωL)/2、ω=r(ωR−ωL)/T。真实四轮接触会产生侧滑，故理论圆与 Gazebo 实测圆存在差异，R02 将差异公开保留，R04 再进行有效轮距标定及独立速度试验。当前接触系数是仿真假设，不是实测草地参数。

C++ 只给车轮/转向关节下发速度，车身位姿由 Gazebo 接触动力学积分，不直接指定车体轨迹。反馈采用实际关节状态；真值只发送到独立评价域。当前驱动是有限加减速的关节速度执行器，不是电机扭矩、功耗或轮胎变形模型。
