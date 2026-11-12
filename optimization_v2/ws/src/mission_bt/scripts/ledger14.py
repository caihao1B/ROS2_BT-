"""S14 area-only overlay adapter; S10 feedback and transaction guards unchanged."""
from common14 import *
from ledger_geometry14 import area_union

LEDGER=BASE/'ws/src/work_ledger/scripts'
source=(LEDGER/'ledger_core.py').read_text()
source=replace_once(source,'credit=union(credit,set_precision(p,precision),grid_size=precision)',
                    'credit=area_union(credit,p,precision)')
scope={'__name__':'s14_area_ledger','__file__':str(LEDGER/'ledger_core.py'),'area_union':area_union}
exec(compile(source,str(LEDGER/'ledger_core.py'),'exec'),scope)
Ledger=scope['Ledger'];latest_at=scope['latest_at']

if __name__=='__main__':
    node=replace_once((LEDGER/'ledger_node.py').read_text(),
                      'from ledger_core import Ledger,latest_at',
                      'from ledger14 import Ledger,latest_at')
    exec(compile(node,str(LEDGER/'ledger_node.py'),'exec'),
         {'__name__':'__main__','__file__':str(LEDGER/'ledger_node.py')})
