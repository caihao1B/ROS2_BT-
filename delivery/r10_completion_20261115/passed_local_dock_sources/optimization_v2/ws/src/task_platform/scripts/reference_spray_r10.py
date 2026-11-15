"""Keep motion invalid during stopped revalidation; expose a planning-only preview."""
from pathlib import Path
def derive(source):
    marker='d=dict(t=t,valid=valid,'
    assert source.count(marker)==1
    return source.replace(marker,"d=dict(t=t,valid=valid,planning_valid=bool(s.get('recovery_preview') and -.03<=t-s.get('t',-100)<.2 and m and -.03<=t-stamp(m)<.2),localization_epoch=s.get('recovery_epoch',0),")
if __name__=='__main__':
    source=Path(__file__).with_name('reference_r07.py');exec(compile(derive(source.read_text()),str(source),'exec'),globals())
