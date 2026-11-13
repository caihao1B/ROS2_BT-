from common16 import *
import html

def render(out,data):
 payload=dict(result=data,offline=read(out/'offline/offline.json') if (out/'offline/offline.json').exists() else {},examples=read(out/'offline/examples.json') if (out/'offline/examples.json').exists() else [],contracts=read(out/'contracts.json') if (out/'contracts.json').exists() else {})
 template=(PKG/'scripts/report16.html').read_text();(out/'index.html').write_text(template.replace('__DATA__',json.dumps(payload,ensure_ascii=False,allow_nan=False).replace('</','<\\/')))
