import sys,unittest,tempfile,hashlib
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import runtime_binding_r10 as binding
from prepare_r12 import recording_source

class RuntimeBinding(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.pkg=self.root/'package';(self.pkg/'scripts').mkdir(parents=True);self.ed=self.root/'edition';self.ed.mkdir();self.original=self.pkg/'scripts/runtime_spray_r10.py';self.original.write_text((Path(__file__).resolve().parents[1]/'scripts/runtime_spray_r10.py').read_text());self.derived=self.ed/'runtime_spray.py';self.derived.write_text(recording_source(self.original.read_text(),'scene_spray_r10','R10'));self.hash=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();self.spec=dict(runtime_sha256=self.hash(self.derived),implementation_bindings={str(self.original.relative_to(self.root)):self.hash(self.original),str(self.derived.relative_to(self.root)):self.hash(self.derived)})
  self.patch=patch.multiple(binding,ROOT=self.root,PKG=self.pkg,edition=lambda stage:self.ed);self.patch.start()
 def tearDown(self):self.patch.stop();self.temp.cleanup()
 def test_exact_observer_derivation_accepted(self):self.assertTrue(binding.spray_runtime_valid(self.spec))
 def test_task_command_modification_rejected_even_with_updated_self_hash(self):
  self.derived.write_text(self.derived.read_text().replace("'seconds':", "'seconds_changed':")+'\n# arbitrary change\n');self.spec['runtime_sha256']=self.hash(self.derived);self.spec['implementation_bindings'][str(self.derived.relative_to(self.root))]=self.hash(self.derived);self.assertFalse(binding.spray_runtime_valid(self.spec))
 def test_changed_original_rejected(self):self.original.write_text(self.original.read_text()+'\n# changed upstream\n');self.assertFalse(binding.spray_runtime_valid(self.spec))
 def test_normal_original_runtime_accepted(self):self.spec['runtime_sha256']=self.hash(self.original);self.assertTrue(binding.spray_runtime_valid(self.spec))
 def test_observer_buffer_derivation_requires_exact_content(self):
  from prepare_spray_video_r12 import derive
  p=self.ed/'runtime_spray_video.py';p.write_text(derive(self.original.read_text()));self.spec.update(runtime_sha256=self.hash(p));self.spec['implementation_bindings'][str(p.relative_to(self.root))]=self.hash(p)
  self.assertTrue(binding.spray_runtime_valid(self.spec))
  p.write_text(p.read_text()+'\n# unauthorized runtime mutation\n');self.spec['runtime_sha256']=self.hash(p);self.spec['implementation_bindings'][str(p.relative_to(self.root))]=self.hash(p)
  self.assertFalse(binding.spray_runtime_valid(self.spec))
