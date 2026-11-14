"""Deliver real process signals while a native ROS callback is still running."""
import os,signal,subprocess,selectors,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5]

class CooperativeShutdown(unittest.TestCase):
 def exercise(self,sig):
  executable=ROOT/'optimization_v2/three_tasks/build/r08/r08_shutdown_probe'
  p=subprocess.Popen([str(executable)],env=dict(os.environ,ROS_DOMAIN_ID='122'),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
  try:
   with selectors.DefaultSelector() as ready:
    ready.register(p.stdout,selectors.EVENT_READ)
    self.assertTrue(ready.select(8),'Native callback did not start')
   self.assertEqual(p.stdout.readline().strip(),'CALLBACK_ACTIVE')
   p.send_signal(sig);out,err=p.communicate(timeout=5)
   self.assertEqual(p.returncode,0,err);self.assertIn('CALLBACK_CONTEXT_INTACT',out)
  finally:
   if p.poll() is None:p.kill();p.communicate()
 def test_sigint_does_not_destroy_active_callback_context(self):self.exercise(signal.SIGINT)
 def test_sigterm_does_not_destroy_active_callback_context(self):self.exercise(signal.SIGTERM)
