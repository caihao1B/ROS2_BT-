import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

SOURCE=Path(__file__).resolve().parents[1]/'ws/src/project_baseline/scripts/session_owner.py'
spec=importlib.util.spec_from_file_location('session_owner',SOURCE)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class OwnershipTests(unittest.TestCase):
    def test_matching_live_session_stops_and_saves(self):
        with tempfile.TemporaryDirectory() as directory:
            record=Path(directory)/'session.json'; saved=Path(directory)/'saved.txt'
            code='''import os,signal,sys,time
from pathlib import Path
sys.path.insert(0,sys.argv[1])
from session_owner import save_session
record=Path(sys.argv[2]);saved=Path(sys.argv[3])
def finish(sig,frame):
 saved.write_text('INTERRUPTED_AND_SAVED');record.unlink();sys.exit(0)
signal.signal(signal.SIGINT,finish)
save_session(record,'s01_owned_fixture','fixture')
time.sleep(10)
'''
            child=subprocess.Popen([sys.executable,'-c',code,str(SOURCE.parent),str(record),str(saved),'s01_owned_fixture'])
            try:
                deadline=time.monotonic()+3
                while not record.exists() and time.monotonic()<deadline:time.sleep(.02)
                self.assertTrue(record.exists())
                self.assertEqual(module.stop_session(record,timeout=3)['status'],'STOPPED')
                self.assertEqual(child.wait(timeout=3),0)
                self.assertEqual(saved.read_text(),'INTERRUPTED_AND_SAVED')
            finally:
                if child.poll() is None:child.terminate();child.wait(timeout=3)
    def test_start_time_and_marker_must_match(self):
        row=module.identity(os.getpid());row['marker']='unittest'
        self.assertTrue(module.matches(row))
        row['start_ticks']='0';self.assertFalse(module.matches(row))
    def test_wrong_marker_refused(self):
        row=module.identity(os.getpid());row['marker']='not_a_real_command_marker_71839'
        self.assertFalse(module.matches(row))
    def test_stale_record_does_not_kill_unrelated_process(self):
        child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(10)'])
        try:
            row=module.identity(child.pid);row.update(marker='stage_runner.py',run_id='bad_fixture')
            with tempfile.TemporaryDirectory() as directory:
                p=Path(directory)/'session.json';p.write_text(json.dumps(row))
                self.assertEqual(module.stop_session(p,timeout=.1)['status'],'STALE_SESSION_REFUSED')
                self.assertIsNone(child.poll())
        finally:child.terminate();child.wait(timeout=3)

if __name__=='__main__':unittest.main()
