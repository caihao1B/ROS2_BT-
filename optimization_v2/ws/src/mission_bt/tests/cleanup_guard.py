"""Process-containment unit: interrupt a launch/cleanup and preserve foreign sessions.
This is an OS process test, not a simulated robot success claim.
"""
import sys,subprocess,time,os,uuid,types
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import runtime14
from common14 import *

def main(out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);partition='s14_cleanup_unit_'+uuid.uuid4().hex
    foreign=subprocess.Popen(['/usr/bin/python3','-c','import time; time.sleep(30)'],env=dict(os.environ,GZ_PARTITION=partition+'_foreign'))
    owned=[]
    def interrupted_case(*args,**kwargs):
        p=subprocess.Popen(['/usr/bin/python3','-c','import time; time.sleep(30)'],env=dict(os.environ,GZ_PARTITION=partition));owned.append(p)
        dump(out/'processes.json',dict(partition=partition,created_wall=time.time(),commands=[]));raise KeyboardInterrupt('INJECTED_DURING_CLEANUP')
    runtime14.runtime=lambda *args,**kwargs:types.SimpleNamespace(run_case=interrupted_case)
    try:
        try:runtime14.run_case(out,'zone_a','cleanup_unit')
        except KeyboardInterrupt:pass
        assert owned and owned[0].poll() is not None,'owned process leaked'
        assert foreign.poll() is None,'foreign session was signaled'
        dump(out/'result.json',dict(passed=True,kind='OS_PROCESS_CONTAINMENT_UNIT',owned_exit=owned[0].returncode,foreign_untouched=True));print('CLEANUP_INTERRUPT_CONTAINMENT_PASS')
    finally:
        foreign.terminate();foreign.wait(timeout=3)
        for p in owned:
            if p.poll() is None:p.kill();p.wait(timeout=3)
if __name__=='__main__':main(sys.argv[1])
