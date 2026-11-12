# 原项目分阶段优化入口

完整计划在 [命令 Prompt 合集](/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md)，独立执行指令在 `docs/optimization_v2/prompts/S01.md` 至 `S18.md`。

现在可运行：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh list
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh prompt 01
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh manual 01
```

在项目聊天中说“执行第 N 步”即可要求代理完成该步实现、验证、可视结果与学习材料。索引不代替编程，`demo/test` 执行器由各阶段交付。S01 执行器及学习材料已建立，所有步骤的当前有效结果以 `stepctl.sh status` 为准，不能手工预填 PASS。

每步结束必须判断能否进入下一步。可运行 `bash /home/ceihao/robot_project/optimization_v2/stepctl.sh next 01` 核对第 1 步后的准入条件；该命令不会自动开始下一步。

`test` 真正运行检查；`check` 只读回执并校验文件哈希；`open` 打开实测报告；`stop` 仅停止对应执行器启动的会话。接口与回执格式见 [执行器合同](/home/ceihao/robot_project/optimization_v2/stages/README.md)。
