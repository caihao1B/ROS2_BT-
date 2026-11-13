"""Exercise real Chromium filters; screenshot is automated QA, not manual acceptance."""
import sys,re,json,html,subprocess,tempfile,os,signal
from pathlib import Path
out=Path(sys.argv[1]).resolve();source=(out/'index.html').read_text()
qa='''<script>setTimeout(()=>{let q={};try{q.cards=$('cards').children.length===6;q.sixty_rows=$('rows').children.length===60;q.no_overflow=document.documentElement.scrollWidth<=window.innerWidth;$('cluster').value='A2';table();q.cluster_filter=$('rows').children.length===10;$('seed').selectedIndex=1;table();q.seed_filter=$('rows').children.length===1;$('rows').firstElementChild.click();q.case_details=$('replayTitle').textContent.startsWith('A2_seed');$('cluster').value='';$('seed').value='';$('controller').value='DWB';table();q.controller_filter=$('rows').children.length===30;$('controller').value='';table();q.missing_visible=data.formal.every(r=>r.metrics.status!=='NOT_RUN'||!r.replay);q.status_honest=data.status!=='PASS'||data.missing_formal.length===0;q.requirements=Object.keys(data.traceability??{}).length===8;if(data.demo?.length){$('demos').lastElementChild.click();q.demo_replay=$('replayTitle').textContent===data.demo.at(-1).spec.id&&!!current.replay;$('timeline').value=.5;draw();q.replay_slider=true}q.extra_buttons=$('extra').querySelectorAll('button').length>=data.extra.length;}catch(e){q.error=String(e)}q.passed=!q.error&&Object.values(q).every(v=>v===true);let e=document.createElement('pre');e.id='qa';e.textContent=JSON.stringify(q);document.body.appendChild(e)},700)</script>'''
(out/'_qa.html').write_text(source.replace('</html>',qa+'</html>'))
with tempfile.TemporaryDirectory(prefix='s17_browser_') as tmp:
    args=['google-chrome','--headless','--disable-gpu','--no-first-run','--no-default-browser-check','--user-data-dir='+tmp,'--virtual-time-budget=1800','--window-size=1400,1900','--screenshot='+str(out/'report_preview.png'),'--dump-dom',(out/'_qa.html').as_uri()]
    p=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
    try:stdout,stderr=p.communicate(timeout=40)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);stdout,stderr=p.communicate(timeout=5);raise
    m=re.search('<pre id="qa">(.*?)</pre>',stdout);d=json.loads(html.unescape(m.group(1))) if m else dict(passed=False,error=stderr[-1000:]);d['exit_code']=p.returncode;(out/'browser_check.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(d))
    if d['passed'] and p.returncode==0:(out/'_qa.html').unlink()
    raise SystemExit(0 if d['passed'] and p.returncode==0 else 1)
