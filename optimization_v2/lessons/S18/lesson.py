"""Teaching use of the actual bounded-store component with disposable bytes."""
import sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/release_tools/scripts'))
from bounded_store18 import BoundedStore
with tempfile.TemporaryDirectory() as folder:
    store=BoundedStore(Path(folder)/'chunks',100)
    store.put('protected',b'p'*20,True)
    store.put('old',b'a'*60)
    result=store.put('new',b'b'*60)
    assert result['payload_bytes']==80 and result['evicted']==['old']
    try:store.put('too_large',b'x'*101)
    except ValueError:pass
    else:raise AssertionError('oversized payload accepted')
    assert (store.root/'protected').read_bytes()==b'p'*20
    print('PASS:',result,'; original project evidence untouched')
