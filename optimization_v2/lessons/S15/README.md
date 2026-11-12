# S15 可运行算法教学

以下例子使用生产函数，输入是明确标记的合成教学数据，不计入 Gazebo 验收。

一个终端，顺序执行：

```bash
cd /home/ceihao/robot_project
optimization_v2/venvs/prediction/bin/python optimization_v2/lessons/S15/lesson.py
optimization_v2/venvs/prediction/bin/python - <<'PY'
import json,yaml
from pathlib import Path
Path('/tmp/s15_lesson_config.json').write_text(json.dumps(yaml.safe_load(Path('optimization_v2/config/prediction.yaml').read_text())))
PY
c++ -std=c++17 -O2 -UNDEBUG -Ioptimization_v2/ws/src/prediction_risk_guard/include optimization_v2/lessons/S15/lesson.cpp -o /tmp/s15_lesson
/tmp/s15_lesson /tmp/s15_lesson_config.json
```

Python 输出 CV、CA、左转、右转的不同概率与 3.2 秒终点，验证概率和为 1，并拒绝时间倒退。C++ 输出预测冲突的速度上限 0，以及过期预测回退的原因；回退仍受原独立安全门约束。

真实神经网络验证（CPU 隔离环境）：

```bash
optimization_v2/venvs/prediction/bin/python optimization_v2/ws/src/motion_prediction/tests/inference15.py /tmp/s15_inference_check.json
```

这个检查实际加载固定的官方权重，检查 25 个分支、邻居输入影响、停止输入影响、概率/协方差、未来标签不可见，以及只省略最终无用网格搜索前后的分布一致性。它是组件验证，不能代替真实闭环矩阵。
