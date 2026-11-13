import sys,unittest,tempfile,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from bounded_store18 import BoundedStore
from release18 import spec_for,verified_restart,termination_label

class Contracts(unittest.TestCase):
    def test_stop_reason_preserves_underlying_scoring(self):
        r=termination_label(dict(status='PARTIAL',reason='NEXT_OBLIGATION'),'RELEASE_STOP_REQUEST')
        self.assertEqual(r['reason'],'REQUESTED_STOP');self.assertEqual(r['scoring_reason'],'NEXT_OBLIGATION')
        r=termination_label(dict(status='FAIL',reason='LOCALIZATION_INVALID'),'RELEASE_STOP_REQUEST')
        self.assertEqual(r['status'],'FAIL');self.assertEqual(r['reason'],'LOCALIZATION_INVALID')
    def test_no_wrong_vehicle_for_cluster(self):
        for command,cluster in [('spray','B1'),('mow','A1')]:
            with self.assertRaisesRegex(ValueError,'CHASSIS'):spec_for(command,cluster,'diff')
    def test_three_real_dispatch_choices(self):
        self.assertEqual(spec_for('spray','A2','diff')['cluster'],'A2')
        self.assertEqual(spec_for('mow','B3','diff')['controller'],'RPP')
        self.assertEqual(spec_for('inspect',None,'ackermann')['operation'],'inspection')
    def test_restart_rejects_external_or_unowned_run(self):
        with self.assertRaisesRegex(ValueError,'OWNED_RUN'):verified_restart(Path('/tmp'))
    def test_ring_is_bounded_and_keeps_pinned(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=BoundedStore(Path(tmp)/'raw',100)
            store.put('protected',b'p'*30,True);store.put('a',b'a'*40)
            result=store.put('b',b'b'*40)
            self.assertEqual(result['payload_bytes'],70);self.assertEqual(result['evicted'],['a'])
            self.assertEqual((store.root/'protected').read_bytes(),b'p'*30)
    def test_protected_exhaustion_does_not_delete(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=BoundedStore(Path(tmp)/'raw',50);store.put('a',b'a'*50,True)
            with self.assertRaisesRegex(ValueError,'PROTECTED'):store.put('b',b'b')
            self.assertEqual((store.root/'a').stat().st_size,50)
    def test_unregistered_and_changed_files_not_deleted(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=BoundedStore(Path(tmp)/'raw',50);store.put('a',b'a'*20)
            (store.root/'foreign').write_bytes(b'x')
            with self.assertRaisesRegex(ValueError,'UNREGISTERED'):store.put('b',b'b'*40)
            (store.root/'foreign').unlink();(store.root/'a').write_bytes(b'z'*20)
            with self.assertRaisesRegex(ValueError,'CHUNK_CHANGED'):store.put('b',b'b'*40)
            self.assertTrue((store.root/'a').exists())
    def test_no_escape_or_symlink(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=BoundedStore(Path(tmp)/'raw',50)
            with self.assertRaises(ValueError):store.put('../outside',b'x')
            (store.root/'outside').symlink_to('/tmp')
            with self.assertRaises(ValueError):store.put('outside',b'x')
    def test_oversized_does_not_evict(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=BoundedStore(Path(tmp)/'raw',50);store.put('a',b'a'*20)
            with self.assertRaises(ValueError):store.put('b',b'b'*51)
            self.assertTrue((store.root/'a').exists())

if __name__=='__main__':unittest.main()
