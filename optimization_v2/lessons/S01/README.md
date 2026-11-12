# 第 1 步 C++ 与 Python 教学例

主题是“启动前检查”：读取配置 → 检查 ROS domain → 将资源路径解析为绝对路径 → 确认仍在项目根目录 → 确认文件存在 → READY 或返回非零退出码。它对应生产代码 `baseline_audit.py` 的路径准入和 `stage_runner.py` 的失败传播，是教学简化，不是第二套生产实现。

运行全部正常/边界样例：

```bash
bash /home/ceihao/robot_project/optimization_v2/lessons/S01/run.sh
```

单独运行 Python：

```bash
python3 /home/ceihao/robot_project/optimization_v2/lessons/S01/lesson.py /home/ceihao/robot_project/optimization_v2/lessons/S01/valid.json
```

编译及单独运行 C++：

```bash
mkdir -p /home/ceihao/robot_project/optimization_v2/ws/build/s01_lessons
g++ -std=c++17 -Wall -Wextra -Wpedantic /home/ceihao/robot_project/optimization_v2/lessons/S01/lesson.cpp -o /home/ceihao/robot_project/optimization_v2/ws/build/s01_lessons/lesson
/home/ceihao/robot_project/optimization_v2/ws/build/s01_lessons/lesson /home/ceihao/robot_project/optimization_v2/lessons/S01/valid.json
```

正常输入输出 `status=READY` 与文件数。把最后的 `valid.json` 换成 `missing.json` 会输出 `MISSING_RESOURCE`，换成 `wrong_domain.json` 会输出 `DOMAIN_MISMATCH`；两个程序都返回退出码 2。整体 run.sh 将这种预期拒绝视为边界测试通过。

Python 用 `Path.resolve/is_relative_to`，C++ 用 `weakly_canonical/lexically_relative`；都先规范化路径，避免只比较字符串前缀。生产审计另外对源码、模型和证据计算 SHA-256，检查动态依赖、资源 URI、互斥与运行结果，教学例没有覆盖这些功能。
