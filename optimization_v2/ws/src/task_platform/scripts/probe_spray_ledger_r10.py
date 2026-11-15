"""Same slow-clock/paused-clock probe for the point-service ledger transport."""
from pathlib import Path
source=Path(__file__).with_name('probe_ledger_clock_r10.py')
code=source.read_text().replace('134','136').replace('ledger_clock_probe','spray_ledger_clock_probe').replace('ledger_r09.py','ledger_r07.py').replace('ledger_mowing_r10.py','ledger_spray_r10.py').replace('mowing/tool.json','spray/tool.json')
if __name__=='__main__':exec(compile(code,str(source),'exec'),globals())
