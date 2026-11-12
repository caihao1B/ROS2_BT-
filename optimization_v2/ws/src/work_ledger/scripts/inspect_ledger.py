"""Read-only terminal inspection. Never opens the ledger for writing."""
import argparse,json,sqlite3
from pathlib import Path
root=Path(__file__).resolve().parents[5];base=root/'optimization_v2'
p=argparse.ArgumentParser();p.add_argument('--case',choices=['point_service_states','continuous_work_and_retry'],default='point_service_states');p.add_argument('--demo',action='store_true');p.add_argument('--db',type=Path);a=p.parse_args()
if a.db:db=a.db.resolve()
else:
 pointer=json.loads((base/'results/S10'/('last_demo.json' if a.demo else 'latest.json')).read_text());result=json.loads((root/Path(pointer['report_path']).parent/'result.json').read_text());db=root/result['cases'][a.case]['path']/'ledger/ledger.sqlite'
con=sqlite3.connect(db.as_uri()+'?mode=ro',uri=True);print('READ ONLY:',db);print('数据库完整性:',con.execute('PRAGMA integrity_check').fetchone()[0]);print('key | 状态 | 确认量 | 目标量 | 不确定 | 原因')
for key,status,q,goal,u,reason in con.execute('SELECT key,status,quantity,goal,uncertain,reason FROM jobs ORDER BY rowid'):
 g=json.loads(goal);print(f'{key} | {status} | {q:.4f} | {g["requested_quantity"]:.4f} | {bool(u)} | {reason}')
print('反馈事件数/确认数:',con.execute('SELECT COUNT(*),SUM(accepted) FROM events').fetchone());con.close()
