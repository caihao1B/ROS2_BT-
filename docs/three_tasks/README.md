# 点喷、巡迹、割草三任务改造

这是独立于原 S01–S18 的新任务系列，采用同地图、共用定位/感知基础、分别运行三种任务。原地图、正式任务与 S17/S18 PARTIAL 状态保留。

- [R01 手工验收与入口](R01/MANUAL_CHECK.md)
- [R01 技术原理](R01/TECHNICAL.md)
- [R01 Python/C++ 核心代码](R01/CODE_WALKTHROUGH.md)
- [R01 实测结果](R01/RESULTS.md)
- [R01–R12 实施步骤及下一步提示词](实施步骤.md)

开发机入口：
```bash
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh doctor
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh smoke spray --gui
bash /home/ceihao/robot_project/optimization_v2/three_tasks/run.sh open
```

本机软件和地图输入仍是运行前提。GitHub 同步源码、配置与精简证据，不代表原 8 GB 以上输入/证据已全部转成新机器安装包；跨机器需要构建与重新验收，不应直接修改哈希绕过检查。
