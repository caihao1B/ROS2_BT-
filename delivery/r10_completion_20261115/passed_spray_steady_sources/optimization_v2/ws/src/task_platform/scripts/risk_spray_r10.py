"""Qualified stopped preview may build costs; actuator still requires motion validity."""
from pathlib import Path
def derive(source):
    marker="loc and loc['valid'] and"
    assert source.count(marker)==1
    source=source.replace(marker,"loc and (loc['valid'] or loc.get('planning_valid',False)) and")
    return source.replace('d=dict(t=t,valid=valid and boundary,',"d=dict(t=t,localization_epoch=(loc or {}).get('localization_epoch',0),valid=valid and boundary,")
if __name__=='__main__':
    source=Path(__file__).with_name('risk_r07.py');exec(compile(derive(source.read_text()),str(source),'exec'),globals())
