"""Run the unchanged production transaction probe with the mowing policy."""
from pathlib import Path
import hashlib
from common import APP,read,dump,sha

if __name__=='__main__':
    template=Path(__file__).with_name('probe_revalidation_r10.py');source=template.read_text()
    for before,after in [("results/R10/revalidation_probe","results/R10/mowing_revalidation_probe"),("'spray/manager.json'","'mowing/manager.json'")]:
        assert source.count(before)==1;source=source.replace(before,after)
    namespace=dict(globals(),__name__='mowing_transaction_probe');exec(compile(source,str(template),'exec'),namespace)
    code=namespace['run']();path=APP/'results/R10/mowing_revalidation_probe/result.json';report=read(path);report.update(template_sha256=sha(template),derived_source_sha256=hashlib.sha256(source.encode()).hexdigest());dump(path,report);raise SystemExit(code)
