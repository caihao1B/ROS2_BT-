"""Self-contained measured report with observed-history and future-mode playback."""
from common15 import *
import html

def render(out,data):
    out=Path(out)
    examples=read(out/'prediction_examples.json') if (out/'prediction_examples.json').exists() else []
    from analysis15 import rows
    timelines={}
    for name in data.get('cases',{}):
        sampled=[];last=-10.;reason=None
        for r in rows(out/name/'risk.jsonl'):
            if r['t']-last>=.4 or r['reason']!=reason:
                sampled.append({k:r[k] for k in ['t','reason','mode_envelope_mass','max_speed','hold']});last=r['t'];reason=r['reason']
        decisions=[dict(t=r['t'],action=r['action'],reason=r['reason_code'],obligation=r.get('evidence',{}).get('obligation'),global_reason=r.get('evidence',{}).get('global_reason'),perception_reason=r.get('evidence',{}).get('perception_reason')) for r in rows(out/name/'decisions.jsonl')]
        timelines[name]=dict(risk=sampled,decisions=decisions)
    payload=json.dumps(dict(results=data,examples=examples,timelines=timelines),ensure_ascii=False,allow_nan=False).replace('</','<\\/')
    template=(PKG/'scripts/report15.html').read_text()
    (out/'index.html').write_text(template.replace('__PAYLOAD__',payload))
