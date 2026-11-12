# S05 可运行学习例

C++ 例直接调用本机 robot_localization 3.8.3 的真实 `Ekf::correct`，比较正常协方差、膨胀协方差和马氏门控拒绝。调试文件包含上游内部 Innovation/Kalman gain/修正后状态，是有界教学探针；不能把这些有界输入称为完整车辆测试。

```bash
bash /home/ceihao/robot_project/optimization_v2/scripts/build_s05.sh
source /home/ceihao/robot_project/optimization_v2/env.sh
/home/ceihao/robot_project/optimization_v2/ws/install/localization_guard/lib/localization_guard/ekf_probe /home/ceihao/robot_project/optimization_v2/runtime/s05_ekf_trace.txt
/home/ceihao/robot_project/optimization_v2/venvs/scene/bin/python /home/ceihao/robot_project/optimization_v2/lessons/S05/lesson.py
```

两个程序应输出 PASS。C++ 中协方差膨胀后结果更接近原预测，门控拒绝后仍为 0.5；Python 显示固定坐标对齐下累计误差和 1 秒相对位移误差，不逐段重新对齐消除漂移。正式运行的完整轨迹、轮速—ICP 一致性残差和诊断在对应案例中。

例子还构造合法的位置—速度交叉协方差，展示只更新速度也会修正位置；这解释为何公共 odom 另采用估计速度积分。测试 `math_checks.py` 给原始 EKF pose 交替 ±1000 m，同时输入恒定 0.4 m/s，验证一秒后公共位移仍为 0.4 m，重置时显式清零。
