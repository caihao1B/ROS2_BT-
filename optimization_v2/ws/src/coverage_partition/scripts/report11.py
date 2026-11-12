"""Standalone, offline browser report built only from actual algorithm outputs."""
import json
from common11 import *
def render(out,data):
    payload=json.dumps(data,ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
    template=(PKG/'scripts/report_template.html').read_text()
    (out/'index.html').write_text(template.replace('__S11_DATA__',payload))
