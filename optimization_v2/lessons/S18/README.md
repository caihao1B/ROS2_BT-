# S18 教学例

C++ 演示验收状态合成：输入失效返回 INVALID，前置/运行/存储任一缺失返回 PARTIAL。它是教学逻辑，不替代真实回执审核。

Python 调用本轮实际编写的有界存储组件，在临时目录演示轮转、保留受保护文件和拒绝超额写入。没有删除项目历史输出。

```bash
cd /home/ceihao/robot_project
g++ -std=c++17 -O2 optimization_v2/lessons/S18/lesson.cpp -o /tmp/robot_s18_lesson
/tmp/robot_s18_lesson
optimization_v2/venvs/scene/bin/python optimization_v2/lessons/S18/lesson.py
```

真实构建/执行结果由 S18 审计保存在 `commands.json`；教学例打印 PASS 只表示例子检查通过。
