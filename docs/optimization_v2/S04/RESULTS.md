# S04 实测结果

运行 `test_20261109_155521_93459`：**PASS**。

可以执行下一步 S05：多传感器观测、频率与误差注入通过，S03 前置有效。人工目视状态 NOT_CHECKED。

- S04-C01：PASS
- S04-C02：PASS
- S04-C03：PASS
- S04-C04：PASS

[交互报告](/home/ceihao/robot_project/optimization_v2/results/S04/test_20261109_155521_93459/index.html) · [完整指标](/home/ceihao/robot_project/optimization_v2/results/S04/test_20261109_155521_93459/result.json)。实际命令及退出码在 commands.jsonl，各场景 runtime.json 包含完整进程命令、域、PID、退出码。

| 案例 | 状态 | RTF | 峰值进程 RSS GiB |
|---|---|---|---|
| diff_nominal | PASS | 0.983 | 1.27 |
| ackermann_nominal | PASS | 0.988 | 1.03 |
| diff_sensor_faults | PASS | 0.974 | 1.28 |
| ackermann_sensor_faults | PASS | 0.980 | 1.04 |

| 案例 | 激光 Hz | 左/右目 Hz | IMU / 轮速 / GNSS Hz | 录包 MiB |
|---|---|---|---|---|
| diff_nominal | 9.70 | 14.87 / 15.06 | 200.0 / 100.0 / 5.0 | 51.31 |
| ackermann_nominal | 9.81 | 15.08 / 15.06 | 200.0 / 100.0 / 5.0 | 51.24 |
| diff_sensor_faults | 9.83 | 15.15 / 14.81 | 200.0 / 100.0 / 5.0 | 51.86 |
| ackermann_sensor_faults | 10.00 | 15.15 / 14.98 | 200.0 / 100.0 / 5.0 | 51.33 |

故障案例频率只统计故障前稳态。双目配对最大时间差、定位/感知支路实收频率、CPU/GPU 资源及掉帧缺口见完整指标。录包已实际解码回读。

`diff_sensor_faults` 的实际故障指标：
```json
{
  "delay_s": 0.17999999999999972,
  "image_hz": 6.268500783562596,
  "wheel_scale_error": 0.3000795865526855,
  "fixed_error_rms_m": 0.04923612317151341,
  "float_error_rms_m": 0.6729848712917395,
  "false_fixed_error_m": 3.642950246118737,
  "dropout_fix_count": 0,
  "gyro_step_error": 0.04284285269792048
}
```

`ackermann_sensor_faults` 的实际故障指标：
```json
{
  "delay_s": 0.17999999999999972,
  "image_hz": 5.722942265611848,
  "wheel_scale_error": 0.3000804075696617,
  "fixed_error_rms_m": 0.04923612317151341,
  "float_error_rms_m": 0.6729848712917395,
  "false_fixed_error_m": 3.642950246118737,
  "dropout_fix_count": 0,
  "gyro_step_error": 0.04284285269792048
}
```

附加公开交付验证：
```json
{
  "public_sensor_panel": "PASS",
  "public_sensor_faults": "PASS",
  "live_dashboard": "PASS",
  "report_interactions": "PASS",
  "isolated_rosbag_replay": "PASS",
  "mutex_and_owned_stop": "PASS",
  "unrelated_process_survived": true,
  "manual_review": "NOT_CHECKED",
  "replay_domain": 122,
  "new_apt_install_required": false
}
```


保持原始目标：雷达 32×1024/10 Hz、双目 640×360/15 Hz、IMU 200 Hz、轮速 100 Hz、GNSS 5 Hz。未静默降配。此前单队列方案未达高频目标；已分离视觉与惯性生成进程并异步保存 PNG。延迟恢复后的过时包丢弃并记数。

本步只验证观测和接口，不代表融合定位/SLAM/动态避障通过。GNSS 假固定解及轮滑为受控测量失配；源文件、有效配置及真值隔离边界见 TECHNICAL.md。
