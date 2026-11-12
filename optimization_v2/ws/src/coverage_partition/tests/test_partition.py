"""Production geometry/repair/validator tests, including adversarial failure inputs."""
import sys,unittest,tempfile,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from geometry11 import *
from partition11 import *
from darp_adapter import export_uniform_grid
from shapely.geometry import Polygon,MultiPolygon

class PartitionTests(unittest.TestCase):
    def setUp(self):self.c=parameters();self.domain=box(0,0,9,6);self.g=build_graph(self.domain,self.c)
    def solution(self):return repair(self.g,kmeans(self.g,3,41),3,self.c)[0]
    def test_complete_area_and_unique_ownership(self):
        v=verify(self.domain,self.g,self.solution(),3,self.c);self.assertTrue(v['passed']);self.assertEqual(v['missing_m2'],0)
    def test_hole_and_boundary_slivers_retained(self):
        p=Polygon([(0,0),(9.12,0),(8,6.2),(0,5)],holes=[[(2,2),(3,2),(3,3),(2,3)]])
        p=set_precision(p,self.c['geometry_precision_m']);g=build_graph(p,self.c);l,_=repair(g,kmeans(g,3,41),3,self.c)
        self.assertTrue(verify(p,g,l,3,self.c)['passed']);self.assertLess(p.symmetric_difference(union_all(g['cells'])).area,1e-7)
    def test_isolated_fragments_are_repaired(self):
        raw=np.arange(len(self.g['cells']))%3;l,history=repair(self.g,raw,3,self.c)
        self.assertGreater(max(history['raw_components']),1);self.assertTrue(verify(self.domain,self.g,l,3,self.c)['passed'])
    def test_point_contact_is_not_adjacency(self):
        with self.assertRaisesRegex(ValueError,'DOMAIN'):build_graph(MultiPolygon([box(0,0,1,1),box(1,1,2,2)]),self.c)
    def test_split_work_domain_rejected(self):
        with self.assertRaisesRegex(ValueError,'DOMAIN'):build_graph(MultiPolygon([box(0,0,2,2),box(3,0,5,2)]),self.c)
    def test_self_intersection_rejected(self):
        with self.assertRaisesRegex(ValueError,'DOMAIN'):build_graph(Polygon([(0,0),(4,4),(0,4),(4,0)]),self.c)
    def test_no_silent_k_reduction(self):
        with self.assertRaisesRegex(ValueError,'TOO_FEW'):build_graph(box(0,0,1,1),self.c)
        self.assertFalse(verify(self.domain,self.g,np.zeros(len(self.g['cells'])),3,self.c)['passed'])
    def test_overlap_and_missing_detected_independently(self):
        labels=self.solution();regions=[union_all([p for i,p in enumerate(self.g['cells']) if labels[i]==k]) for k in range(3)]
        bad=regions.copy();bad[0]=bad[0].union(bad[1]);v=verify(self.domain,self.g,labels,3,self.c,bad);self.assertFalse(v['checks']['overlap']);self.assertTrue(v['checks']['omission'])
        bad=regions.copy();bad[0]=bad[0].buffer(-.1);v=verify(self.domain,self.g,labels,3,self.c,bad);self.assertFalse(v['checks']['omission']);self.assertTrue(v['checks']['overlap'])
    def test_wrong_unit_owner_detected(self):
        labels=self.solution();regions=[union_all([p for i,p in enumerate(self.g['cells']) if labels[i]==k]) for k in range(3)]
        labels[0]=(labels[0]+1)%3;self.assertFalse(verify(self.domain,self.g,labels,3,self.c,regions)['checks']['assignment_geometry'])
    def test_darp_contract_supported_and_unsupported(self):
        self.assertEqual(export_uniform_grid(self.g,[0,20,40],1)['status'],'INTERFACE_ONLY_NOT_EXECUTED')
        g=build_graph(box(.1,0,9,6),self.c)
        with self.assertRaisesRegex(ValueError,'PARTIAL'):export_uniform_grid(g,[0,20,40],1)
    def test_node_weight_is_not_edge_weight(self):
        m=load_domain('A','base',self.c);g=build_graph(m['domain'],self.c,m)
        altered=m|{'speed_mps':m['speed_mps']/2};g2=build_graph(m['domain'],self.c,altered)
        self.assertEqual(g['edges'],g2['edges']);self.assertGreater(g2['weights'].sum(),g['weights'].sum())
    def test_conservative_envelope_does_not_discard_work(self):
        m=load_domain('B','base',self.c);g=build_graph(m['domain'],self.c,m)
        self.assertGreater(m['unresolved_boundary'].area,0);self.assertAlmostEqual(sum(g['area']),m['domain'].area,places=6)
    def test_budget_conflict_is_visible(self):
        m=load_domain('B','base',self.c)
        self.assertGreater(m['domain'].area/(.9*m['speed_mps']),self.c['k']*self.c['hard_budget_s'])
        self.assertEqual(m['speed_mps'],.12)
    def test_kmeans_seed_reproducible(self):self.assertTrue(np.array_equal(kmeans(self.g,3,42),kmeans(self.g,3,42)))
    def test_metis_reproducible_real_binary(self):
        with tempfile.TemporaryDirectory() as t:
            a=metis(self.g,3,41,self.c,Path(t));b=metis(self.g,3,41,self.c,Path(t));self.assertTrue(np.array_equal(a,b))
    def test_cpp_rejects_malformed_csr(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);req=dict(xadj=[0,1,2,2],adjncy=[1,2],adjwgt=[1,1],vwgt=[1,1,1],k=3,seed=41,ufactor=50)
            dump(p/'bad.json',req);r=subprocess.run([str(BIN),str(p/'bad.json'),str(p/'out.json')],capture_output=True,text=True,timeout=5)
            self.assertNotEqual(r.returncode,0);self.assertIn('ASYMMETRIC',r.stderr);self.assertFalse((p/'out.json').exists())
    def test_cpp_rejects_disconnected_csr(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);req=dict(xadj=[0,1,2,2],adjncy=[1,0],adjwgt=[1,1],vwgt=[1,1,1],k=3,seed=41,ufactor=50)
            dump(p/'bad.json',req);r=subprocess.run([str(BIN),str(p/'bad.json'),str(p/'out.json')],capture_output=True,text=True,timeout=5)
            self.assertIn('DISCONNECTED',r.stderr)
    def test_cpp_out_of_range_offset_refused_without_crash(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);req=dict(xadj=[0,99,1,2],adjncy=[1,0],adjwgt=[1,1],vwgt=[1,1,1],k=3,seed=41,ufactor=50)
            dump(p/'bad.json',req);r=subprocess.run([str(BIN),str(p/'bad.json'),str(p/'out.json')],capture_output=True,text=True,timeout=5)
            self.assertEqual(r.returncode,2);self.assertIn('OFFSET',r.stderr)
    def test_bundle_tamper_and_stale_input_refused(self):
        from products11 import validate_bundle
        with tempfile.TemporaryDirectory() as t:
            p=Path(t);dump(p/'a.json',{'value':1});m=dict(status='PROVISIONAL',execution_allowed=False,k=3,files={'a.json':sha(p/'a.json')},binding=dict(input_hashes={},source_hashes={},dependency_receipts={}))
            def save():
                m.pop('revision',None);m['revision']=identity(m);dump(p/'partition_manifest.json',m)
            save();self.assertEqual(validate_bundle(p)['k'],3)
            dump(p/'a.json',{'value':2})
            with self.assertRaisesRegex(ValueError,'BUNDLE_HASH'):validate_bundle(p)
            m['files']['a.json']=sha(p/'a.json');m['binding']['input_hashes']['optimization_v2/config/tool_contract.yaml']='0'*64;save()
            with self.assertRaisesRegex(ValueError,'INPUT_VERSION'):validate_bundle(p)
            m['binding']['input_hashes']={};m['binding']['source_hashes']['optimization_v2/config/partition.yaml']='0'*64;save()
            with self.assertRaisesRegex(ValueError,'SOURCE_VERSION'):validate_bundle(p)
if __name__=='__main__':unittest.main(verbosity=2)
