# S05 实测结果

运行 `test_20261109_180306_181533`：**PASS**。可以执行下一步 S06：S05 九案例全部通过且 S04 有效；未自动执行 S06。人工查看 NOT_CHECKED。

- S05-C01：PASS
- S05-C02：PASS
- S05-C03：PASS
- S05-C04：PASS

[交互报告](/home/ceihao/robot_project/optimization_v2/results/S05/test_20261109_180306_181533/index.html) · [汇总指标](/home/ceihao/robot_project/optimization_v2/results/S05/test_20261109_180306_181533/result.json)。完整进程命令、PID、退出码在各案例 runtime.json；各门槛和未通过项在 metrics.json。

| 案例 | 状态 | 物理路程 m | 轮速 ATE m | 轮速+IMU ATE m | 融合 ATE m | RTF |
|---|---|---:|---:|---:|---:|---:|
| diff_reset | PASS | 8.05 | 0.065 | 0.042 | 0.061 | 0.911 |
| diff_slope | PASS | 10.45 | 0.153 | 0.132 | 0.364 | 0.855 |
| ackermann_slope | PASS | 9.79 | 0.116 | 0.168 | 0.468 | 0.805 |
| diff_loop | PASS | 18.73 | 0.241 | 0.152 | 0.375 | 0.847 |
| ackermann_loop | PASS | 19.17 | 0.006 | 0.149 | 0.434 | 0.791 |
| diff_faults | PASS | 27.31 | 1.269 | 1.856 | 1.158 | 0.830 |
| ackermann_faults | PASS | 31.76 | 1.250 | 2.260 | 1.869 | 0.793 |
| diff_imu_loss | PASS | 9.65 | 0.093 | 1.817 | 0.277 | 0.842 |
| ackermann_imu_loss | PASS | 10.35 | 0.003 | 2.063 | 0.284 | 0.835 |

diff_faults 故障指标：
```json
{
  "wheel_slip_score": {
    "n": 3000,
    "ate_rmse_m": 1.4244384770101206,
    "ate_max_m": 1.812778873683836,
    "endpoint_error_m": 1.2380831101925074,
    "rpe_1s_rmse_m": 0.13075492711897135,
    "step_max_m": 0.005948292013351809,
    "pitch_rmse_rad": 4.600994527059465e-09,
    "estimated_pitch_max_rad": 0.0,
    "truth_pitch_max_rad": 8.492765479530711e-09
  },
  "wheel_imu_slip_score": {
    "n": 900,
    "ate_rmse_m": 1.3701824184646987,
    "ate_max_m": 1.7264707705364501,
    "endpoint_error_m": 1.3421238566167186,
    "rpe_1s_rmse_m": 0.12767640755186754,
    "step_max_m": 0.023543859271933226,
    "pitch_rmse_rad": 0.0022565573797796584,
    "estimated_pitch_max_rad": 0.007081574092647758,
    "truth_pitch_max_rad": 8.483432337915244e-09
  },
  "fused_slip_score": {
    "n": 900,
    "ate_rmse_m": 0.25921689740273784,
    "ate_max_m": 0.46686645821640604,
    "endpoint_error_m": 0.3186951459097562,
    "rpe_1s_rmse_m": 0.10450796163474922,
    "step_max_m": 0.023481036301110068,
    "pitch_rmse_rad": 0.0022605740266601598,
    "estimated_pitch_max_rad": 0.007081574089681464,
    "truth_pitch_max_rad": 8.483432337915244e-09
  },
  "fused_to_wheel_ate_ratio": 0.18197830344124866,
  "icp_stop_t": 67.217,
  "imu_stop_t": 86.261,
  "icp_physical_stop": {
    "passed": true,
    "delay_sim_s": 0.27200000000000557,
    "distance_m": 0.02589923298486271,
    "final_v_max": 0.0,
    "final_w_max": 0.0,
    "thresholds": {
      "v": 0.01,
      "w": 0.02,
      "delay": 2.0
    }
  },
  "imu_physical_stop": {
    "passed": true,
    "delay_sim_s": 0.0040000000000048885,
    "distance_m": 0.0,
    "final_v_max": 0.0,
    "final_w_max": 0.0,
    "thresholds": {
      "v": 0.01,
      "w": 0.02,
      "delay": 2.0
    }
  }
}
```


ackermann_faults 故障指标：
```json
{
  "wheel_slip_score": {
    "n": 3000,
    "ate_rmse_m": 1.2861438216480572,
    "ate_max_m": 1.6749032832971988,
    "endpoint_error_m": 1.3767005603515092,
    "rpe_1s_rmse_m": 0.12887488599053468,
    "step_max_m": 0.00595034378050219,
    "pitch_rmse_rad": 1.1362483434135856e-10,
    "estimated_pitch_max_rad": 0.0,
    "truth_pitch_max_rad": 5.541612925327074e-10
  },
  "wheel_imu_slip_score": {
    "n": 900,
    "ate_rmse_m": 1.2691833684807736,
    "ate_max_m": 1.644123456938939,
    "endpoint_error_m": 1.3022223756772668,
    "rpe_1s_rmse_m": 0.13053035524124407,
    "step_max_m": 0.023586533569164164,
    "pitch_rmse_rad": 0.0020105264088606384,
    "estimated_pitch_max_rad": 0.007152039372773089,
    "truth_pitch_max_rad": 5.493228591206831e-10
  },
  "fused_slip_score": {
    "n": 900,
    "ate_rmse_m": 0.5795334592632958,
    "ate_max_m": 0.8770956952726597,
    "endpoint_error_m": 0.6960016423722866,
    "rpe_1s_rmse_m": 0.09335209025572747,
    "step_max_m": 0.024446072460322824,
    "pitch_rmse_rad": 0.0020118507891354782,
    "estimated_pitch_max_rad": 0.007152081579693714,
    "truth_pitch_max_rad": 5.493228591206831e-10
  },
  "fused_to_wheel_ate_ratio": 0.45059770883219347,
  "icp_stop_t": 67.267,
  "imu_stop_t": 86.264,
  "icp_physical_stop": {
    "passed": true,
    "delay_sim_s": 0.2610000000000099,
    "distance_m": 0.02709924366324704,
    "final_v_max": 0.0,
    "final_w_max": 2.842170943040401e-14,
    "thresholds": {
      "v": 0.01,
      "w": 0.02,
      "delay": 2.0
    }
  },
  "imu_physical_stop": {
    "passed": true,
    "delay_sim_s": 0.5880000000000081,
    "distance_m": 0.13713852210869026,
    "final_v_max": 3.5904852008286593e-14,
    "final_w_max": 0.0,
    "thresholds": {
      "v": 0.01,
      "w": 0.02,
      "delay": 2.0
    }
  }
}
```


补充复验说明：针对实测失败补验：修复 RTAB 私有 reset 服务与显式默认 Q/P 复位；阿克曼在同一 5° 坡道受支撑坡面起步；额外验证两车在实际行驶中 IMU 断流。正常非复位案例的有效 Q/P 与上游原默认一致。

原失败案例保留在 superseded 中；它们没有被改写成 PASS。补充命令和退出码在 commands.jsonl；完整启动命令仍在各案例 runtime.json。

观测 JSONL 已无损 gzip 压缩；评分器直接支持解压读取，未丢弃采样。

固定同一起点对齐；全程 ATE 与 20–50 s 打滑窗口分开统计。没有按段拟合误差。真实 IMU/轮编码器/激光观测进入估计，domain 120 真值只用于离线评分。默认不使用 GNSS、地图回环或视觉定位。

在线可查量是输入质量、协方差、轮速与独立激光速度一致性残差；实际 EKF 内部创新另由链接已安装 C++ 引擎的有限探针展示，不冒充完整实车 EKF 内部创新日志。

局限：15 状态普通 EKF，没有在线 IMU 偏置状态；静止启动校准；无逐点激光时间戳，未做运动去畸变；稀疏农田可能停车，不能视为全场定位保证。RTAB-Map 自身配准参数选择及被否定方案见 TECHNICAL.md。
