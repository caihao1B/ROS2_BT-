# R08 手工检查与续作入口

R08 当前是开发检查点。先看结果文档和回执；PARTIAL 不等于完整巡迹/跟随/绕行通过，也不允许直接进入 R09。以下命令从主文件夹运行，不依赖原移动卷。

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh status
bash optimization_v2/three_tasks/advance.sh R07 check
bash optimization_v2/three_tasks/advance.sh R08 check
bash optimization_v2/three_tasks/advance.sh R08 open
```

`check` 复核封存文件哈希及阶段状态，PARTIAL 返回非零；`open` 打开本地结果页。原世界、原 88.48 m 连接路和历史正式验收保持独立。

按需复现；一次只运行一个仿真，不要求依次执行全部：

```bash
bash optimization_v2/three_tasks/advance.sh R08 build
bash optimization_v2/three_tasks/advance.sh R08 test nominal 20
bash optimization_v2/three_tasks/advance.sh R08 test nominal 0
bash optimization_v2/three_tasks/advance.sh R08 test leader 15
bash optimization_v2/three_tasks/advance.sh R08 test blocked 15
```

第二参数是选取的原路径前缀长度；0 表示完整原路径。裁取只用于诊断，不删去剩余义务，也不能将短测替代完整连接路。正常全程上限 420 秒仿真，诊断上限 240 秒；墙钟一般为仿真时间的约 2.5 倍再加启动/退出，程序另有墙钟上限。失败会保存原因并停车退出，不自动解除定位跳变锁定。

运行期间另开终端：

```bash
source /home/ceihao/robot_project/optimization_v2/env.sh
export ROS_DOMAIN_ID=119
ros2 topic echo /r06/directive --once
ros2 topic echo /r08/leader --once
ros2 topic echo /r07/gate --once
ros2 topic info /s10/tool/command --verbose
```

巡迹车应无工具命令发布者。共用定位和安全适配话题仍保留 `/r07/` 前缀；全局会话锁保证只有一个任务运行。检查行为原因、距离/TTC/置信度、当前观测路径资格及实际停车确认，而不是只看 BT 名称。

本轮结束后，继续提示词：

> 继续 R08：先检查最新回执和结果中的真实失败，完成原连接路 FollowPath、基于当前点云的同向前车跟随/停车/恢复，并实现与验证当前观测约束的绕行候选或有界停车上报。不得放宽原定位、未知区域、停车及版本门限，不得把规划器失败当作全局无路证明。R08 真正 PASS 后，按已有授权继续 R09 及后续步骤；每阶段保留手工命令、技术和代码说明，定期同步 GitHub。
