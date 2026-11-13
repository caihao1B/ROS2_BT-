"""Small statistical example; synthetic numbers are not robot measurements."""
from pathlib import Path
import sys,math
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws/src/acceptance_runner/scripts'))
from statistics17 import wilson,paired
from temporal_join17 import TerrainJoin
interval=wilson(10,10)
assert .72<interval[0]<.73 and math.isclose(interval[1],1.)
assert wilson(0,0) is None
records=[dict(method=m,scenario='example_only',seed=41,task_hash='same_task',limits_hash='same_limits',cpu_ms=t) for m,t in [('event',100.),('cache',98.)]]
result=paired(records,'event','cache','cpu_ms')
assert result['pairs']==1 and result['difference']['p50']==-2.
records[1]['task_hash']='different_task'
assert paired(records,'event','cache','cpu_ms')['pairs']==0
print('教学虚构数据：10/10 的 Wilson 95% 区间为',interval,'；它不等于真实成功率必为 100%。')
join=TerrainJoin()
join.status(dict(t=10.2,valid=True))
join.grid(10.,'diff','old_grid')
assert join.take(10.2)==[]  # Newer health cannot validate an older grid.
join.status(dict(t=10.,valid=False,reason='unverified_surface'))
result=join.take(10.2)
assert result[0][0]=='old_grid' and result[0][2]['valid'] is False
join.grid(11.,'diff','expired_grid');join.status(dict(t=11.,valid=True))
assert join.take(11.5)==[]
print('7 Python checks passed: statistics, same-condition pairs and exact-stamp joins; no simulation completion claim.')
