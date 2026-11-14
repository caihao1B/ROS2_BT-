# 手工验收命令

```bash
cd /home/ceihao/robot_project
source optimization_v2/env.sh
bash optimization_v2/three_tasks/advance.sh R03 check
bash optimization_v2/three_tasks/advance.sh R03 open
cat optimization_v2/three_tasks/results/R03/validation.json
cat optimization_v2/three_tasks/results/R03/tests.log
```

`check` 应显示 passed=true，changed=[]。网页包含地图、起终点、三簇边界和时间图：灰色是原工作区内未计作作业的地头，深色是作业/巡迹线，紫色是工具关闭转场，绿点是起点，红叉是终点。A 是点服务路线，点服务时原地执行；不是紫色路线本身完成喷洒。B 不再要求每条相邻条带都用完整大圆绕行。

```bash
# 重新执行独立检查与故障注入测试，不重新生成计划
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 bash optimization_v2/three_tasks/advance.sh R03 test
# 同时确认旧版入口和上一阶段仍有效
bash optimization_v2/three_tasks/run.sh check
bash optimization_v2/three_tasks/advance.sh R02 check
```

七项测试包含缺簇、路径断点、倒车方向错误、越界、超时/哈希错误，以及原生 RS 倒车与换向终点检查。本阶段不启动 Gazebo，也不是正式 60 次矩阵。

开发者要重规划时执行 `advance.sh R03 build`，随后 `test` 和 `finalize`。重规划会改变 R03 版本并使依赖旧版本的后续回执失效，因此不作为普通查看命令。
