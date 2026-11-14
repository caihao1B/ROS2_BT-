# R01 核心代码与学习顺序

## Python：配置 → 运行 → 结果

1. `optimization_v2/ws/src/task_runtime/scripts/registry_r1.py`：`load_task()` 计算规范 JSON 哈希，检查结果目录和任务角色，再逐项检查绑定文件；`validate_task()` 复用原地图/计划校验，形成运行 spec。错误会在启动仿真前暴露。
2. `taskctl.py`：`run()` 获取共享 flock，记录会话身份，启动子任务并观察在线状态。达到短测时长、现实预算或收到停止信号时，请求原链路退出。启动器记录真实退出码、工具关闭、残留进程和评分，不把取消当作完成。
3. `one_r1.py`：生成独立运行 overlay，只在生成副本的 server 启动位置注入环境函数。替换要求精确且唯一匹配；旧运行器结构变动时失败，避免模糊替换出错。原文件不改。
4. `lifecycle_r1.py`：逐项确认兼容库及依赖的哈希，只有 `name == server` 才设置 LD_PRELOAD。外部已有预加载库会被拒绝，以免混合未验收的环境。
5. `ownership_r1.py`：停止请求先匹配 task_id，再校验进程身份；错误任务不能结束正在运行的任务。
6. `baseline_r1.py`：对原回执引用的文件做完整比对。它不重新生成原阶段的 PASS，也不掩盖缺失证据。

伪代码：

```python
profile, spec = validate_task(task)
validate_shutdown_libraries()
with exclusive_project_session():
    run = create_task_result_directory(task, profile.version)
    child = launch_original_stack_in_isolated_overlay(spec, run)
    while child.running:
        observe_localization_tool_and_clock()
        if operator_stop or bounded_time_reached:
            request_original_orderly_shutdown(child)
            break
    save_real_exit_codes_tool_feedback_and_remaining_work()
```

## C++：停止事件串行化

`optimization_v2/ws/src/task_runtime/src/shutdown_compat.cc` 导出当前 Gazebo ABI 的 Stop 符号。`dlsym(RTLD_NEXT, ...)` 找到真正的函数；pthread mutex 使两个并发停止请求排队执行原函数，避免同时操作退出事件。找不到符号直接报错退出，不做成功降级。链接参数 `--no-as-needed` 保留 Boost 依赖，`-z,nodelete` 保留本兼容库的映射。

这段 C++ 是退出兼容层，不是新的规划器、控制器或 SLAM 算法。不要把它用于未锁定的 Gazebo 版本。对应测试同时检查两线程竞争，以及真实 Gazebo 相机/项目世界的退出。

## 当前阶段不新增的算法

RTAB-Map、轮速/IMU/激光里程计、三维感知到二维代价、DWB/RPP、原任务 BT 和工具反馈账本继续复用原实现。R01 的工程重点是任务配置与运行结果的隔离、版本可追溯及可控停止。后续改变车辆、路径或 BT 时，应更新独立任务版本并重新验收受影响链路。

`lifecycle_r1.close_owned_views()` 根据 Popen 记录中的 RViz PID 匹配 X11 窗口，发送 `_NET_CLOSE_WINDOW` 请求并等待正常退出；未匹配窗口不会关闭，失败时原有界清理继续执行并保留真实异常码。GUI 默认视图只跟随估计 base_link，不改变 TF、定位或车辆参数。

当前显示层兼容还包括移除存在析构缺陷的 ImageDisplay 面板；仅影响 RViz 展示，不影响摄像头/感知。关闭前保存的是运行目录内的临时生成视图。
