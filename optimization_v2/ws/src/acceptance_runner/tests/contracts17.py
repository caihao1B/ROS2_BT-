"""Adversarial checkpoint and scoring contracts, not a simulated robot test."""
import sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common17 import *
from checkpoint17 import Journal
from report17 import aggregate
from overlay17 import build

class Contracts(unittest.TestCase):
    def setUp(self):
        RESULTS.mkdir(parents=True,exist_ok=True);self.tmp=tempfile.TemporaryDirectory(dir=RESULTS,prefix='contract_');self.root=Path(self.tmp.name);self.j=Journal(self.root/'journal');self.case=self.root/'case';self.case.mkdir();self.spec=dict(id='A1_seed41001');self.binding=dict(version=1);self.m=dict(status='FAIL');dump(self.case/'metrics17.json',self.m)
    def tearDown(self):self.tmp.cleanup()
    def test_duplicate_rejected(self):
        self.j.commit(self.spec,self.binding,self.case,self.m)
        with self.assertRaisesRegex(ValueError,'DUPLICATE_SCORE'):self.j.commit(self.spec,self.binding,self.case,self.m)
        self.assertEqual(len(self.j.collect(self.binding)),1)
    def test_tamper_rejected(self):
        self.j.commit(self.spec,self.binding,self.case,self.m);dump(self.case/'metrics17.json',dict(status='PASS'))
        with self.assertRaisesRegex(ValueError,'EVIDENCE_CHANGED'):self.j.load(self.spec,self.binding)
    def test_stale_version_not_reused(self):
        self.j.commit(self.spec,self.binding,self.case,self.m);self.assertIsNone(self.j.load(self.spec,dict(version=2)))
    def test_failed_not_made_success(self):
        self.j.commit(self.spec,self.binding,self.case,self.m);self.assertEqual(self.j.load(self.spec,self.binding)['metrics']['status'],'FAIL')
    def test_resume_cannot_skip_failed_or_partial_trial(self):
        from stage17 import require_completed_checkpoint
        for status in ['FAIL','PARTIAL','NOT_RUN']:
            record=dict(spec=self.spec,metrics=dict(status=status))
            with self.assertRaisesRegex(ValueError,'RECORDED_CASE_NOT_COMPLETE'):require_completed_checkpoint(record)
        require_completed_checkpoint(dict(spec=self.spec,metrics=dict(status='PASS')))
    def test_metrics_disagreement_rejected(self):
        with self.assertRaisesRegex(ValueError,'METRICS_DISAGREE'):self.j.commit(self.spec,self.binding,self.case,dict(status='PASS'))
    def test_empty_matrix_not_pass(self):
        d=aggregate([]);self.assertEqual(d['status'],'PARTIAL');self.assertEqual(len(d['missing_formal']),60);self.assertTrue(all(c['executed']==0 for c in d['clusters'].values()))
    def test_partial_receipt_still_checks_integrity(self):
        from audit17 import audit
        from unittest.mock import patch
        receipt=dict(schema_version=1,step='S17',plan_revision='test',automated_status='PARTIAL',source_files=[dict(path=str((self.case/'metrics17.json').relative_to(ROOT)),sha256='incorrect')],artifacts=[dict(path=p,sha256='unused') for p in [*[f'docs/optimization_v2/S17/{n}.md' for n in ['TECHNICAL','CODE_WALKTHROUGH','MANUAL_CHECK','RESULTS']],*[f'optimization_v2/lessons/S17/{n}' for n in ['lesson.cpp','lesson.py','README.md']]]],checks={},report_path='docs/optimization_v2/S17/RESULTS.md')
        plan=dict(revision='test',steps=[dict(id='S17',checks=[],depends_on=[])])
        with patch('audit17.read',side_effect=lambda p:plan if Path(p).name=='steps.json' else receipt):
            with self.assertRaisesRegex(ValueError,'EVIDENCE_CHANGED'):audit(self.case/'receipt.json')
    def test_generated_adapters_compile(self):
        folder=build(self.root/'overlay',RESULTS/'qualification/stations.json',full=True)
        for p in folder.glob('*.py'):compile(p.read_text(),str(p),'exec')
        # Generated regional_prior contains a second source adapter. Compile
        # that actual inner program too, without creating ROS nodes or sensors.
        from types import SimpleNamespace
        from unittest.mock import patch
        regional=(folder/'regional_prior14.py').read_text()
        tail="    exec(compile(source,str(NAV/'scripts/prior_cost13.py'),'exec'),scope)"
        self.assertEqual(regional.count(tail),1)
        regional=regional.replace(tail,"    compile(source,'regional_prior_inner17','exec')")
        sys.path.insert(0,str(folder))
        try:
            with patch.dict(sys.modules,{'binding14':SimpleNamespace(region=lambda:({},dict(cloud_frame='site_enu')))}):
                exec(compile(regional,str(folder/'regional_prior14.py'),'exec'),dict(__name__='__main__',__file__=str(folder/'regional_prior14.py')))
        finally:sys.path.remove(str(folder))
        import subprocess
        code='import sys;sys.path.insert(0,'+repr(str(folder))+');import near_field17,fanout17'
        imported=subprocess.run(['/usr/bin/python3','-c',code],capture_output=True,text=True,timeout=20)
        self.assertEqual(imported.returncode,0,imported.stderr)
    def test_partial_does_not_count_complete(self):
        from matrix17 import make
        spec=make()[1]['formal'][0];data=aggregate([dict(spec=spec,metrics=dict(status='PARTIAL',sim_seconds_including_stop=55),case_path='unused',identity='unit_test_only')]);self.assertEqual(data['clusters']['A1']['passed'],0);self.assertEqual(data['clusters']['A1']['partial'],1)
if __name__=='__main__':unittest.main()
