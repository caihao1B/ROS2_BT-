from common import *
from prepare_r07 import build,build_terrain
from runtime_r07 import run
from analyze_r07 import analyze,figures

def main(command,args):
    require('R06');out=APP/'results/R07';out.mkdir(parents=True,exist_ok=True)
    if command=='build':build();build_terrain();return 0
    if command=='test':
        cluster=args[0] if args else 'A1';case=args[1] if len(args)>1 else 'nominal';limit=int(args[2]) if len(args)>2 else 3
        if cluster not in ['A1','A2','A3'] or case not in ['nominal','tool_jam'] or limit<0:raise ValueError('USE test A1|A2|A3 nominal|tool_jam POINT_LIMIT (0=whole cluster)')
        p=run(cluster=cluster,case=case,point_limit=limit,seconds=1800 if limit==0 else 240);r=analyze(p);print(json.dumps(dict(passed=r['passed'],path=str(p),failed=[k for k,v in r['checks'].items() if not v])));return 0 if r['passed'] else 2
    if command!='finalize':raise ValueError('USE build / test [cluster case limit] / finalize / check / open')
    paths={case:ROOT/read(out/('spray_'+case+'_latest.json'))['path'] for case in ['nominal','tool_jam']};results={case:analyze(p) for case,p in paths.items()}
    with (out/'tests.log').open('w') as f:tests=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(PKG/'tests'),'-p','*r07*.py'],stdout=f,stderr=subprocess.STDOUT,timeout=30)
    summary=dict(passed=all(r['passed'] for r in results.values()) and tests.returncode==0,cases=results,unit_exit=tests.returncode,scope='Point-service component integration and residual retry, not whole-cluster/global reliability acceptance',next='R08 independent inspection and observed leader interaction')
    images=figures(paths);report=render('R07','R07 / Nav2 point service and measured work ledger',summary,images);sources=list((PKG/'r07_nav').glob('*'))+list((PKG/'scripts').glob('*_r07.py'))+list((PKG/'tests').glob('*_r07.py'))+list((ROOT/'docs/three_tasks/R07').glob('*.md'));files=list(edition('R07').rglob('*'))+[APP/'results/R06/latest.json',out/'tests.log',report,*images,APP/'build/r07/r07_terrain',APP/'build/r07_source/terrain.cpp',APP/'build/r07/r07_supervisor',APP/'build/r07_source/supervisor.cpp',APP/'build/r07_nav_install/lib/libr07_exact_goal.so',BASE/'ws/src/work_ledger/scripts/ledger_core.py']
    for p in paths.values():files+=list(p.rglob('*'))
    files+=list((out/'unknown_policy_test').glob('*'))
    d=seal('R07',summary,files,sources,'R08');print(json.dumps(dict(status=d['automated_status'],next=d['next_step'])));return 0 if summary['passed'] else 2
