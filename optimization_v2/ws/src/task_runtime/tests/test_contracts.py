"""Reject wrong editions, unsafe budgets and non-owned stop requests."""
import sys,copy,tempfile,unittest,os
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common_r1 import read,APP,canonical,dump
import registry_r1 as registry
import lifecycle_r1 as lifecycle
import ownership_r1 as owner
import taskctl
from types import SimpleNamespace
from taskctl import budgets

class Contracts(unittest.TestCase):
    def test_three_distinct_versions_same_original_map(self):
        p=[registry.load_task(t) for t in registry.TASKS]
        self.assertEqual(len({x['version'] for x in p}),3)
        self.assertEqual(len({x['results_root'] for x in p}),3)
        self.assertEqual(len({x['map_id'] for x in p}),1)
        self.assertEqual(len({x['plan_revision'] for x in p}),1)
    def test_config_tampering_rejected(self):
        d=read(APP/'config/tasks/spray.json');d['seed']+=1
        with patch.object(registry,'read',return_value=d),self.assertRaisesRegex(ValueError,'VERSION_MISMATCH'):registry.load_task('spray')
    def test_wrong_result_scope_rejected_even_if_rehashed(self):
        d=read(APP/'config/tasks/spray.json');d['results_root']='optimization_v2/results/S17';d['version']=canonical({k:v for k,v in d.items() if k!='version'})
        with patch.object(registry,'read',return_value=d),self.assertRaisesRegex(ValueError,'RESULT_SCOPE'):registry.load_task('spray')
    def test_changed_map_digest_rejected(self):
        d=read(APP/'config/tasks/spray.json');d['bindings']['atlas']['sha256']='0'*64;d['version']=canonical({k:v for k,v in d.items() if k!='version'})
        with patch.object(registry,'read',return_value=d),self.assertRaisesRegex(ValueError,'INPUT_CHANGED atlas'):registry.load_task('spray')
    def test_cross_task_cluster_rejected(self):
        with self.assertRaisesRegex(ValueError,'CLUSTER_NOT_IN_TASK'):registry.validate_task('spray','B1')
    def test_invalid_budgets(self):
        for args in [(0,300,1,None),(300,0,1,None),(300,300,0,None),(300,300,float('nan'),None),(300,300,1,float('nan')),(300,300,1,299)]:
            with self.subTest(args=args),self.assertRaises(ValueError):budgets(*args)
    def test_valid_bounded_budget(self):budgets(300,300,1,60)
    def test_nonserver_environment_unchanged(self):
        e={'A':'B'};self.assertEqual(lifecycle.environment('slam',e,APP),e)
        self.assertEqual(e,{'A':'B'})
    def test_view_uses_local_base_without_changing_fixed_frame(self):
        import yaml
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'scene/localization.rviz';p.parent.mkdir();p.write_text(yaml.safe_dump({'Visualization Manager':{'Global Options':{'Fixed Frame':'odom'},'Views':{'Current':{'Focal Point':dict(X=6,Y=33,Z=.5)}}}}))
            self.assertEqual(lifecycle.environment('rviz',{'A':'B'},Path(tmp)),{'A':'B'})
            d=yaml.safe_load(p.read_text())['Visualization Manager'];self.assertEqual(d['Global Options']['Fixed Frame'],'odom')
            self.assertEqual(d['Views']['Current']['Target Frame'],'base_link');self.assertEqual(d['Views']['Current']['Focal Point']['X'],0)
    def test_only_faulty_image_viewer_is_omitted(self):
        import yaml
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'scene/localization.rviz';p.parent.mkdir();p.write_text(yaml.safe_dump({'Visualization Manager':{'Global Options':{'Fixed Frame':'odom'},'Views':{'Current':{}},'Displays':[{'Class':'rviz_default_plugins/Image'},{'Class':'rviz_default_plugins/PointCloud2'}]}}))
            lifecycle.environment('rviz',{},Path(tmp));d=yaml.safe_load(p.read_text())['Visualization Manager']['Displays']
            self.assertEqual(d,[{'Class':'rviz_default_plugins/PointCloud2'}])
    def test_window_close_never_touches_unowned_process(self):
        from unittest.mock import Mock
        child=Mock()
        with tempfile.TemporaryDirectory() as tmp,patch('subprocess.run') as runner:
            lifecycle.close_owned_views([{'role':'server'}],[child],Path(tmp));runner.assert_not_called();child.wait.assert_not_called()
    def test_server_external_preload_rejected(self):
        with self.assertRaisesRegex(ValueError,'EXTERNAL_PRELOAD'):lifecycle.environment('server',{'LD_PRELOAD':'unknown'},APP)
    def test_abi_hash_mismatch_rejected(self):
        d=read(APP/'config/shutdown.json');d['libraries'][0]['sha256']='0'*64
        with patch.object(lifecycle,'read',return_value=d),self.assertRaisesRegex(ValueError,'ABI_OR_LIBRARY_CHANGED'):lifecycle.validate()
    def test_pid_reuse_rejected(self):
        me=owner.identity(os.getpid());me.update(marker=sys.argv[0]);self.assertTrue(owner.matches(me))
        me['start_ticks']='0';self.assertFalse(owner.matches(me))
    def test_stop_wrong_task_or_stale_owner_never_sends_signal(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(owner,'APP',Path(tmp)),patch.object(owner.os,'kill') as kill:
            p=Path(tmp)/'runtime/session.json';dump(p,dict(task_id='inspection',pid=os.getpid(),uid=os.getuid(),start_ticks='0',marker='taskctl.py'))
            self.assertEqual(owner.request_stop('mowing')['status'],'TASK_SCOPE_MISMATCH')
            self.assertEqual(owner.request_stop('inspection')['status'],'STALE_SESSION_REFUSED');kill.assert_not_called()
    def test_no_session_stop_is_idempotent(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(owner,'APP',Path(tmp)):
            self.assertEqual(owner.request_stop('spray')['status'],'NOT_RUNNING')
    def test_spawn_failure_records_failure_and_removes_session(self):
        with tempfile.TemporaryDirectory(dir=APP/'runtime') as tmp,patch.object(taskctl,'APP',Path(tmp)),patch.object(taskctl.fcntl,'flock'),patch.object(taskctl.subprocess,'Popen',side_effect=FileNotFoundError('injected missing launcher')):
            a=SimpleNamespace(task='spray',cluster=None,sim_seconds=300,wall_timeout=300,rtf=1,stop_after_sim=60,command='smoke',gui=False)
            self.assertEqual(taskctl.run(a),2)
            result=read(next((Path(tmp)/'results/spray').glob('r01_*/result.json')))
            self.assertEqual(result['lifecycle_status'],'FAIL')
            self.assertIn('injected missing launcher',result['error'])
            self.assertFalse((Path(tmp)/'runtime/session.json').exists())

if __name__=='__main__':unittest.main(verbosity=2)
