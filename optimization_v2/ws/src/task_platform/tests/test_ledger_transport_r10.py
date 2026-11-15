import sys,unittest
from pathlib import Path
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts';sys.path.insert(0,str(SCRIPTS))
from ledger_mowing_r10 import derive

class LedgerTransport(unittest.TestCase):
 def test_original_credit_and_staleness_guards_are_preserved(self):
  source=(SCRIPTS/'ledger_r09.py').read_text();derived=derive(source)
  for marker in ["result=engine.observe(d,ctx,st['t'])", "st['t']-feedbacks[0]['t']>=.035", "time.monotonic()-st['wall']>c['clock_wall_s']", "if reason=='CLOCK_WATCHDOG':start_reason=reason", "n.create_timer(.01,process_feedback)"]:
   self.assertIn(marker,derived)
  self.assertIn('clock_type=_ClockType.STEADY_TIME',derived)
  compile(derived,'derived_ledger','exec')
 def test_spray_keeps_point_credit_and_pairing(self):
  source=(SCRIPTS/'ledger_r07.py').read_text();derived=derive(source)
  self.assertIn('from ledger_core import Ledger,latest_at',derived);self.assertIn("result=engine.observe(d,ctx,st['t'])",derived);self.assertIn("st['t']-feedbacks[0]['t']>=.035",derived);compile(derived,'spray_ledger','exec')
 def test_source_drift_fails_closed(self):
  with self.assertRaises(ValueError):derive('no timer')
if __name__=='__main__':unittest.main()
