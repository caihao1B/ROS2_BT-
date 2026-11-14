# R01 学习示例

Python 只读实验：
```bash
python3 /home/ceihao/robot_project/lessons/three_tasks/R01/config_version_demo.py
```
修改内存中的 seed，比较规范 JSON 哈希，观察配置版本如何变化；不会修改任务文件。

C++ 并发实验源码位于 `optimization_v2/ws/src/task_runtime/tests/stop_race.cc`。同一个文件分别构建模拟原函数提供库和双线程调用程序。无兼容库时峰值为 2；预加载后峰值为 1。这只证明互斥逻辑；真实 Gazebo 的服务停止和信号停止另有独立运行证据，不能用这个小程序替代。

实际核心源码是 `optimization_v2/ws/src/task_runtime/src/shutdown_compat.cc`。候选构建命令：
```bash
bash /home/ceihao/robot_project/optimization_v2/three_tasks/build_compat.sh
```
只输出 candidate，不替换已验收库。更新生产库必须重新绑定配置并复验，不能直接改哈希来伪装原验收。
