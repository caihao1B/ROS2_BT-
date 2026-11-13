"""Actual Chromium DOM interaction; it does not mark user manual acceptance."""
import sys,re,json,html,subprocess,tempfile,os,signal
from pathlib import Path
out=Path(sys.argv[1]).resolve();s=(out/'index.html').read_text()
qa='''<script>setTimeout(()=>{let q={};try{q.no_horizontal_overflow=document.documentElement.scrollWidth<=window.innerWidth;q.data=D.examples.length>0;q.fair=D.offline.fair_schedules===true;let before=$('metrics').textContent;$('split').value='val';metric();q.split=before!==$('metrics').textContent;$('split').value='test';metric();q.scenes=$('episode').options.length===8;let b=$('sampleLabel').textContent;$('sample').value=$('sample').max;example();q.slider=b!==$('sampleLabel').textContent;q.detail=$('lookup').textContent.includes('clearance');q.case_switch=$('case').options.length>0;q.bars=$('bars').getContext('2d').getImageData(0,0,1100,230).data.some((v,i)=>i%4===3&&v>0);q.trajectory=$('trajectory').getContext('2d').getImageData(0,0,550,400).data.some((v,i)=>i%4===3&&v>0)}catch(e){q.error=String(e)}q.passed=!q.error&&Object.values(q).every(x=>x===true);let el=document.createElement('pre');el.id='qa';el.textContent=JSON.stringify(q);document.body.appendChild(el)},700)</script>'''
(out/'_qa.html').write_text(s.replace('</html>',qa+'</html>'))
with tempfile.TemporaryDirectory(prefix='s16_browser_') as tmp:
 args=['google-chrome','--headless','--disable-gpu','--no-first-run','--no-default-browser-check','--user-data-dir='+tmp,'--virtual-time-budget=1400','--window-size=1400,2100','--screenshot='+str(out/'report_preview.png'),'--dump-dom',(out/'_qa.html').as_uri()]
 p=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
 try:stdout,stderr=p.communicate(timeout=40)
 except subprocess.TimeoutExpired:
  os.killpg(p.pid,signal.SIGTERM);stdout,stderr=p.communicate(timeout=5);raise
 m=re.search('<pre id="qa">(.*?)</pre>',stdout);d=json.loads(html.unescape(m.group(1))) if m else dict(passed=False,error=stderr[-800:]);d['exit_code']=p.returncode;(out/'browser_check.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(d));raise SystemExit(0 if d['passed'] and p.returncode==0 else 1)
