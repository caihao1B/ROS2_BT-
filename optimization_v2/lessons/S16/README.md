# S16 可运行教学例

Python 直接调用生产 SQLite 库与 C++ ABI，创建临时数据库，演示正常命中、反向拒绝、TTL、时钟回退、版本、锁竞争、超时、LRU 淘汰和重启；不访问原 SLAM 数据库。
C++ 直接调用生产准入函数，再调用 S15 原风险评估。最后一个例子即使带 `cache_hit=true`，当前障碍风险仍要求停车；缓存不产生速度命令。

```bash
cd /home/ceihao/robot_project
bash optimization_v2/scripts/build_s16.sh
optimization_v2/venvs/prediction/bin/python optimization_v2/lessons/S16/lesson.py
c++ -std=c++17 -O2 -UNDEBUG -Ioptimization_v2/ws/src/experience_memory/include -Ioptimization_v2/ws/src/prediction_risk_guard/include optimization_v2/lessons/S16/lesson.cpp -o /tmp/s16_lesson
optimization_v2/venvs/prediction/bin/python -c 'import yaml,json; json.dump(yaml.safe_load(open("optimization_v2/config/prediction.yaml")),open("/tmp/s16_prediction.json","w"))'
/tmp/s16_lesson /tmp/s16_prediction.json
```

预期出现 PASS。断言失败会返回非零。Python 管事务和检索，C++ 管轻量准入；原 S15/S14 C++ 风险与底盘安全出口继续有效，无需重复实现数据库层。
