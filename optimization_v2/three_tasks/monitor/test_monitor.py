"""Observer boundaries: partial writes, stale data, run isolation and no false PASS."""
import json,math,os,tempfile,time,unittest,socket,sys
from unittest.mock import patch
from pathlib import Path
from data import Evidence,tail_records,freshness,json_value

class MonitorEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.app=Path(self.tmp.name)
        self.run=self.app/'results/R12/spray_20261115_000001';self.run.mkdir(parents=True)
        (self.run/'spec.json').write_text(json.dumps(dict(task='spray',case='nominal',cluster='A1',sim_seconds=1800)))
        (self.run/'live.json').write_text(json.dumps(dict(t=30,phase='NAVIGATING',index=1)))
        self.e=Evidence(self.app)
    def tearDown(self):self.tmp.cleanup()
    def test_stop_uses_fresh_observations_before_event(self):
        (self.run/'mission_events.jsonl').write_text('{"event":"TASK_STOP","t":29,"error":"WAIT_DEADLINE"}\n')
        (self.run/'reference').mkdir()
        samples=[dict(t=28.93,native_supervisor=dict(t=28.92,gnss_reason='VALID',innovation_m=.10)),
                 dict(t=28.97,native_supervisor=dict(t=29.02,gnss_reason='FUTURE')),
                 dict(t=30,native_supervisor=dict(t=30,gnss_reason='REJECTED'))]
        (self.run/'reference/reference.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in samples))
        (self.run/'decisions.jsonl').write_text('{"t":22,"action":"EXECUTE"}\n')
        context=self.e.state()['stop_context']
        self.assertEqual(context['localization']['gnss_reason'],'VALID')
        self.assertEqual(context['localization']['t'],28.92)
        self.assertEqual(context['decision'],{})
        self.assertEqual(context['gate'],{})
    def test_partial_json_line_not_treated_as_complete(self):
        p=self.run/'x';p.write_bytes(b'{"t":1}\n{"t":2}\n{"t":3')
        rows,whole=tail_records(p);self.assertEqual([r['t'] for r in rows],[1,2]);self.assertTrue(whole)
    def test_bounded_tail_discloses_incomplete_history(self):
        p=self.run/'x';p.write_text(''.join(json.dumps({'t':i})+'\n' for i in range(1000)))
        rows,whole=tail_records(p,limit=200,count=5);self.assertFalse(whole);self.assertEqual(rows[-1]['t'],999);self.assertEqual(len(rows),5)
    def test_record_count_limit_discloses_incomplete_history(self):
        p=self.run/'x';p.write_text(''.join(json.dumps({'t':i})+'\n' for i in range(8)))
        rows,whole=tail_records(p,limit=1000,count=3)
        self.assertFalse(whole);self.assertEqual([r['t'] for r in rows],[5,6,7])
    def test_occupied_port_does_not_spawn_ros_child(self):
        import server
        with socket.socket() as occupied:
            occupied.bind(('127.0.0.1',0));occupied.listen()
            port=occupied.getsockname()[1]
            with patch.object(sys,'argv',['server.py','--app',str(self.app),'--port',str(port)]), patch.object(server.subprocess,'Popen') as spawn:
                with self.assertRaises(OSError):server.main()
                spawn.assert_not_called()
    def test_missing_stale_and_future_distinguished(self):
        self.assertEqual(freshness(None,30)[0],'MISSING');self.assertEqual(freshness(28,30)[0],'STALE')
        self.assertEqual(freshness(31,30)[0],'FUTURE');self.assertEqual(freshness(29.9,30)[0],'FRESH')
    def test_new_experiment_group_exposes_persisted_operator_notice(self):
        run = self.app/'results/experiments/spray_operator_handoff/spray_20261119_120000_000001'
        run.mkdir(parents=True)
        for file,data in {'spec.json':{'task':'spray','case':'last_obligation'},'runtime.json':{},
                          'operator_handoff.json':{'point':'point_10_20','reason':'bounded'},
                          'candidate_handoff_acceptance.json':{'handoff_status':'CONTROLLED_MANUAL_HOLD'}}.items():
            (run/file).write_text(json.dumps(data))
        s = self.e.state(run.relative_to(self.e.results).as_posix())
        self.assertTrue(s['run']['candidate'])
        self.assertEqual(s['operator_handoff']['point'], 'point_10_20')
        self.assertEqual(s['candidate_result']['handoff_status'], 'CONTROLLED_MANUAL_HOLD')

    def test_isolated_candidates_are_labelled_and_sorted_by_recording_time(self):
        archive=self.app/'results/experiments/spray_defer/spray_defer_20261114_121000_123456'
        archive.mkdir(parents=True)
        spec=dict(task='spray',case='blocked',cluster='A1',point_limit=2,candidate_selected_point_ids=['p2','p3'])
        (archive/'spec.json').write_text(json.dumps(spec));(archive/'runtime.json').write_text('{}')
        (archive/'mission.json').write_text(json.dumps(dict(clusters=[dict(id='A1',points=[dict(id=k) for k in ('p1','p2','p3')])])))
        name=archive.relative_to(self.e.results).as_posix()
        self.assertEqual(self.e.runs()[0]['id'],'R12/'+self.run.name)
        (archive/'candidate_acceptance.json').write_text(json.dumps({'behavior_result':'PASS','formal_credit':False}))
        state=self.e.state(name)
        self.assertTrue(state['run']['candidate'])
        self.assertEqual(state['candidate_result']['behavior_result'],'PASS')
        self.assertEqual(state['analysis'],{})
        self.assertEqual(self.e.geometry(name)['selected_point_ids'],['p2','p3'])
        self.assertEqual(self.e.geometry(name)['original_count'],3)
    def test_no_path_traversal_or_unknown_run(self):
        for p in ['../../etc/passwd','R12/no_such_run','/etc']:
            with self.assertRaises(ValueError):self.e.resolve(p)
    def test_live_run_does_not_infer_pass(self):
        s=self.e.state();self.assertEqual(s['run']['status'],'LIVE');self.assertEqual(s['analysis'],{})
        self.assertEqual(s['stages']['R10'],'NOT_RUN');self.assertEqual(s['sensors']['imu']['freshness'],'MISSING')
    def test_historical_run_not_marked_live(self):
        (self.run/'runtime.json').write_text('{}');self.assertEqual(self.e.state()['run']['status'],'CLOSED')
    def test_old_observer_cannot_cross_session(self):
        p=self.app/'monitor_state';p.mkdir();(p/'live.json').write_text(json.dumps({'run':'R12/another_run','channels':{'camera':{'t':30}}}))
        self.assertEqual(self.e.state()['observer'],{})
    def test_file_stall_distinguished_from_sim_freshness(self):
        old=time.time()-15;os.utime(self.run/'live.json',(old,old))
        self.assertEqual(self.e.state()['run']['status'],'STALE_OR_STARTING')
    def test_invalid_measurement_not_json_nan(self):
        safe=json_value({'a':float('nan'),'b':[float('inf'),1]})
        self.assertEqual(safe,{'a':None,'b':[None,1]});json.dumps(safe,allow_nan=False)
    def test_no_work_credit_for_failed_or_replayed_action(self):
        events=[dict(event='TOOL_RESULT',state='FAILED',point='a'),dict(event='TOOL_RESULT',state='SUCCEEDED',point='b',replayed=True),dict(event='TOOL_RESULT',state='SUCCEEDED',point='c',replayed=False)]
        (self.run/'mission_events.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in events))
        s=self.e.state();self.assertEqual(s['confirmed_points'],1);self.assertEqual(s['confirmed_ids'],['c'])
    def test_short_demo_does_not_shrink_original_plan(self):
        spec=json.loads((self.run/'spec.json').read_text());spec['point_limit']=3;(self.run/'spec.json').write_text(json.dumps(spec))
        (self.run/'mission.json').write_text(json.dumps({'clusters':[{'id':'A1','points':[{'id':str(i),'target':[i,0]} for i in range(56)]}]}))
        g=self.e.geometry();self.assertEqual(g['selected_count'],3);self.assertEqual(g['original_count'],56);self.assertEqual(len(g['points']),56)
    def test_inspection_selected_distance_keeps_original_route(self):
        (self.run/'spec.json').write_text(json.dumps(dict(task='inspection',case='leader',distance_limit_m=2)))
        (self.run/'mission.json').write_text(json.dumps(dict(route_map=[[0,0],[3,4]],original_route_length_m=5)))
        g=self.e.geometry();self.assertEqual(g['original_route_length_m'],5);self.assertEqual(g['selected_route_length_m'],2)
if __name__=='__main__':unittest.main()
