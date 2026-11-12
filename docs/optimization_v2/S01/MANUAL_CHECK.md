# 第 1 步手工查验

使用普通终端即可。每条命令依次运行，不同时启动两个仿真。命令负责环境装载，无需先 source ROS。若 Gazebo 已由原入口运行，第二个仿真实例应被互斥锁拒绝，先关闭自己打开的旧预览再运行。

## 先读状态和下一步判断

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check 01
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next 01
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open 01
```

`check` 校验已有证据及文件哈希，不重新跑仿真；`next` 只有在本步与下一步前置有效时才输出“可以执行第 2 步”。`open` 打开自动验收报告，查看传感器频率、雷达观测、依赖、源码差异和每项命令日志。没有图形桌面时添加 `--print-path` 获取 HTML 文件路径。

## 查看原农田和机器人

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S01 --case legacy_farm --seed 41 --sim-seconds 90 --wall-timeout 240
```

应出现 Gazebo 农田与原 Dino 机器人，机器人静止，终端打印本轮结果目录。可以转动视角检查模型。运行达到 90 仿真秒或内部现实上限时关闭；外层最多 240 现实秒。原模型可能使 RTF 波动，现实耗时和仿真耗时在 `farm/result.json` 分开记录。截图只截 Gazebo 的三维视窗，结果目录包含 `farm/gazebo_window.png`（有桌面并成功取得窗口时）。

演示结果不覆盖已有自动验收回执；查看本次演示应打开终端打印的 `index.html`。画面空白、模型缺块、资源错误或非零运动均应记录为异常，不能仅因窗口存在认定通过。此场景是旧农田基线，新不规则三维世界由 S02 建立。

## 查看依赖与编译接口

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S01 --case dependency_audit --seed 41 --sim-seconds 30 --wall-timeout 180
```

这是离线检查，无仿真时钟。结果目录的依赖清单和 HTML 应显示已安装项、RTAB-Map 等延期项、13 处原源码差异及资源解析情况；C++ F2C/METIS 样例的真实输出在日志中。

## 完整复验

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S01 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1200
```

依次编译新包、运行 API 正常/失败样例、BT/原组件回归、定位适配器故障注入、环境隔离/通信/互斥检查、农田与模型静态检查、两种语言教学例。默认不打开 Gazebo GUI。正常运行约数分钟，外层上限 1200 现实秒；各子案例另有更短时限。结束后重新运行 `check 01` 和 `next 01`。

## 在第二终端停止本步

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S01
```

只停止与本步会话记录匹配的进程；无运行会话时输出 NOT_RUNNING。中断正式复验会产生 PARTIAL，之后需重新复验，不能继续使用上轮 PASS。用户手动关闭 GUI 仅是窗口行为，应以启动器实际检查结果和退出码为准。

## 原入口与 BT 核心

```bash
bash /home/ceihao/robot_project/native_20261108/run.sh check --seconds 35
bash /home/ceihao/robot_project/native_20261108/task_bt/run.sh core-test
```

第一条仍是原静态检查入口，辅助脚本已纳入项目。第二条运行真实编译的 BT 核心及反馈协议测试。对旧 BT 完整任务命令会明确返回 LEGACY_BT_ISOLATED/退出码 64，避免假装缺失的动态任务栈仍可运行。

## 阅读与运行教学例

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S01/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S01/CODE_WALKTHROUGH.md
bash /home/ceihao/robot_project/optimization_v2/lessons/S01/run.sh
```

教学正常输入应显示 READY，缺失文件和错误 domain 应被拒绝。具体 C++ 编译和 Python 独立运行命令见 lessons/S01/README.md。
