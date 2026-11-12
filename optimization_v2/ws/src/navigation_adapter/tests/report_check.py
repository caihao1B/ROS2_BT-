import sys,html,json,re,subprocess,tempfile
from pathlib import Path
p=Path(sys.argv[1]).resolve();out=p.parent
script='''<script>try{let checks={cases:cases.length>0,rendered:document.querySelector('#trajectory').childNodes.length>0,metrics:document.querySelector('#details').textContent.length>20};let s=document.querySelector('#signal');s.value='compute';s.dispatchEvent(new Event('change'));checks.compute=document.querySelector('#series').childNodes.length>0;if(cases.length>1){sel.value='1';sel.dispatchEvent(new Event('change'));checks.switch=ci===1}checks.passed=Object.values(checks).every(Boolean);let q=document.createElement('pre');q.id='qa';q.textContent=JSON.stringify(checks);document.body.append(q)}catch(e){let q=document.createElement('pre');q.id='qa';q.textContent=JSON.stringify({passed:false,error:String(e)});document.body.append(q)}</script>'''
q=out/'_qa.html';q.write_text(p.read_text().replace('</html>',script+'</html>'))
try:
 with tempfile.TemporaryDirectory(prefix='s13_browser_') as tmp:
  args=['google-chrome','--headless','--disable-gpu','--no-first-run','--no-default-browser-check','--user-data-dir='+tmp,'--virtual-time-budget=1500','--window-size=1400,1700','--screenshot='+str(out/'report_preview.png'),'--dump-dom',q.as_uri()];r=subprocess.run(args,capture_output=True,text=True,timeout=40);m=re.search('<pre id="qa">(.*?)</pre>',r.stdout);d=json.loads(html.unescape(m.group(1))) if m else dict(passed=False,error=r.stderr[-1000:]);d.update(exit_code=r.returncode,command=args);(out/'browser_check.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(d));raise SystemExit(0 if d['passed'] and r.returncode==0 else 1)
finally:q.unlink(missing_ok=True)
