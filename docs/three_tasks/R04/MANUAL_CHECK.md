# 手工验收

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
bash optimization_v2/three_tasks/advance.sh R04 check
bash optimization_v2/three_tasks/advance.sh R04 open
cat optimization_v2/three_tasks/editions/R04/qualification.json
cat optimization_v2/three_tasks/results/R04/tests.log
```

网页显示三车命令与真实速度曲线。检查三档前进、正反转弯、倒车、断流停车、锁存拒绝和重新 ARM 恢复。`check` 应显示 passed=true，changed=[]。qualification 内的 slope_qualified 与 full_navigation_qualified 仍为 false，说明适用边界。

```bash
# 单独重跑一车（约 2–3 分钟平地物理仿真），会生成新的独立结果目录
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 bash optimization_v2/three_tasks/advance.sh R04 test spray
# 另两车可将 spray 换成 inspection 或 mowing
```

运行命令只持有自己的会话锁、启停自己启动的进程。重复实测不会修改 R02 车辆或原世界。重新 finalize 会绑定新证据，后续版本需按依赖复验，不作为普通查看命令。
