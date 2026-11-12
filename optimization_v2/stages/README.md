# 阶段执行器和验收回执合同

此目录定义统一合同；S01 执行器已交付，其余在用户执行对应阶段时实现。有效验收状态以步骤索引为准，不预置只打印成功的占位脚本。

## 入口

统一通过 `bash /home/ceihao/robot_project/optimization_v2/stepctl.sh` 调用。执行器必须支持本阶段 prompt 列出的所有 case；参数错误、必需组件缺失、超时都返回非零，正常的故障注入用例则按“预期安全行为是否发生”判定测试结果。

```text
run.sh demo --case NAME --seed 41 --sim-seconds N --wall-timeout M
run.sh test --suite acceptance --seed 41 --sim-seconds N --wall-timeout M
run.sh stop
```

第 17 步额外提供 `--suite formal|ablations`、`--shard 1/10` 和 `--resume`。`--seed` 用于生成/选择固定种子表，表一旦冻结不能随批次改变。正式矩阵按单案例保存检查点；未全部执行时汇总状态是 PARTIAL。成功完成一个分片的进程可返回 0，不能据此把全步骤记为 PASS。整体验收项必须由矩阵聚合结果判定。

每次调用必须校验参数，加载本步锁定环境，并复核前置。不允许执行器以脱离会话的后台服务方式逃脱进程组管理。启动器须保存本次 PID、进程创建标识、会话/ROS domain/Gazebo partition、地图/计划版本；`stop` 只终止匹配本会话的进程。不能使用全局 `killall` 或不带所有权核验的 `pkill`。执行器 trap 要先发停止/关闭工具、保存账本，再等待子进程退出。

现实超时是外层保护，不是可用的任务计时器。内部用 `/clock` 限制单案例仿真秒数；预算中留出停止和保存时间。离线规划/报表例没有仿真钟，明确标记 N/A。暂停/时钟倒退/重启应触发独立 wall watchdog 或 epoch 检查。

长测试使用工具的异步会话和有界轮询，及时向用户报告进度；不能用一次阻塞调用长时间中断沟通。未完成正式矩阵时继续断点执行，不能拿一段演示替代全部验收。

## 结果文件

每轮真实输出放在 `optimization_v2/results/SNN/<run_id>/`。至少包含离线可看的 `index.html`、指标 JSON/CSV、命令/退出码、配置/版本/种子和足够复查的短证据。无需保存所有完整视频或每一帧点云。说明图与真实运行图必须有不同标识。

运行结束后原子更新 `optimization_v2/results/SNN/latest.json`。测试中断或失败同样写回 PARTIAL/FAIL；保留最近一次必要有效基准证据，但不得让它冒充本轮结果。`demo` 输出独立演示记录，不能仅凭演示覆盖自动验收 PASS。没有自动回执时可写带 `report_path` 的 NOT_RUN/PARTIAL 回执供打开演示报告。

下面只是结构示意，尖括号内容必须由本次真实运行生成，不能原样复制成通过回执：

```json
{
  "schema_version": 1,
  "step": "S01",
  "plan_revision": "20261109.1",
  "run_id": "<本轮唯一运行标识>",
  "checked_at_utc": "<实际 UTC 时间>",
  "automated_status": "<PASS|FAIL|PARTIAL|NOT_RUN>",
  "manual_status": "NOT_CHECKED",
  "reason": "<实测结果及尚未满足的条件>",
  "check_exit_code": 0,
  "checks": {"S01-C01": "<PASS|FAIL|NOT_RUN>"},
  "report_path": "optimization_v2/results/S01/<run_id>/index.html",
  "source_files": [{"path": "<实际参与该步骤的源码或配置>", "sha256": "<文件哈希>"}],
  "artifacts": [{"path": "<实际证据或文档>", "sha256": "<文件哈希>"}],
  "dependency_receipts": {}
}
```

`checks` 必须覆盖 `plan/steps.json` 对应阶段全部检查 ID。`source_files` 必须包含实际实现、依赖锁、测试及其有效配置；不能只哈希一个无关的占位文件。`artifacts` 必须包括可视报告、真实指标、四份学习/验收材料，以及 `optimization_v2/lessons/SNN/lesson.cpp`、`lesson.py`、`README.md`。核心运行输入、模型、地图和计划也需有可追踪哈希；大型文件可由受控 manifest 管理，但不能只检查未变化的 manifest 而忽略输入已变化。

有前置阶段时，`dependency_receipts` 为每个直接前置的最新回执 SHA-256，例如 `{"S01": "<S01/latest.json 的 SHA-256>"}`。前置状态、源码或回执变化都会使下游失效，需要按影响范围复验。修改人工查看字段也会改变回执哈希；建议单独保存人工查看记录，在重发回执时一并处理影响。

所有路径须位于主项目根目录内，不能通过符号链接绕到新加卷。`stepctl check` 检查格式、必需检查项、材料/源码哈希和前置证据链；它不是测试运行器，也不能识别人为编造的数据，因此生成回执的执行器仍必须使用真实测量。

## 人工查看与学习材料

人工验证未实际发生时保持 `NOT_CHECKED`。自动测试通过不代表用户已经看过界面，也不要求用户额外授权才能进入已批准的下一步。每阶段必须有自己的 `MANUAL_CHECK.md`，精确列出终端数量、环境装载、演示命令、等待时间、可见现象、失败含义和停止方法。

C++ 与 Python 教学例应聚焦该阶段核心数据流或算法，使用正常/边界输入，不必复刻整套 ROS 系统。例子要真实编译/执行，标记生产实现、上游代码和教学简化之间的差别。`CODE_WALKTHROUGH.md` 给出源文件绝对路径、函数名和本版本行号；只描述伪代码不算完成。
