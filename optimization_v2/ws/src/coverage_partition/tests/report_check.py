"""Actual Chrome executes all report controls and saves a visual QA screenshot."""
import sys,html,json,re,subprocess,tempfile
from pathlib import Path
page=Path(sys.argv[1]).resolve();out=page.parent
script='''<script>try{let checks={rendered:document.body.dataset.rendered==='true'};const picture=()=>by('map').toDataURL();const cv=by('map'),ctx=cv.getContext('2d');checks.map_has_polygons=ctx.getImageData(0,0,cv.width,cv.height).data.filter((v,i)=>i%4===3&&v>0).length>10000;let box=cv.getBoundingClientRect(),pt=drawn[0].point;cv.dispatchEvent(new MouseEvent('click',{clientX:box.left+pt[0]*box.width/cv.width,clientY:box.top+pt[1]*box.height/cv.height}));checks.cluster_click=by('details').textContent.includes('个单元');let before=picture();by('toggle').click();checks.unpartitioned=before!==picture()&&!partitioned;by('toggle').click();checks.partition_restored=before===picture();
for(const id of ['zone','algorithm','variant','seed']){if(by(id).options.length>1){before=picture();by(id).selectedIndex=1;by(id).dispatchEvent(new Event('change'));checks[id]=before!==picture();}}
for(const id of ['margin','points','adjacency']){if(id==='points'){by('zone').value='A';by('zone').dispatchEvent(new Event('change'));}before=picture();by(id).click();checks[id]=before!==picture();}
checks.provisional=document.body.textContent.includes('PROVISIONAL')&&document.body.textContent.includes('不是实测任务时间');checks.passed=Object.values(checks).every(Boolean);let p=document.createElement('pre');p.id='qa';p.textContent=JSON.stringify(checks);document.body.append(p);
by('zone').value='both';by('algorithm').value='metis';by('variant').value='base';by('seed').selectedIndex=0;by('margin').checked=false;by('points').checked=false;by('adjacency').checked=true;refresh();
}catch(e){let p=document.createElement('pre');p.id='qa';p.textContent=JSON.stringify({passed:false,error:String(e)});document.body.append(p);}</script>'''
with tempfile.TemporaryDirectory(prefix='s11_browser_') as tmp:
    p=out/'_qa.html';p.write_text(page.read_text().replace('</html>',script+'</html>'))
    args=['google-chrome','--headless','--disable-gpu','--no-first-run','--no-default-browser-check',f'--user-data-dir={tmp}/profile','--virtual-time-budget=2500','--window-size=1480,1550',f'--screenshot={out}/report_preview.png','--dump-dom',p.as_uri()]
    r=subprocess.run(args,capture_output=True,text=True,timeout=35);m=re.search('<pre id="qa">(.*?)</pre>',r.stdout)
    d=json.loads(html.unescape(m.group(1))) if m else {'passed':False,'error':'missing QA DOM'};d.update(exit_code=r.returncode,command=args)
    (out/'browser_check.json').write_text(json.dumps(d,indent=2)+'\n');p.unlink(missing_ok=True);print(json.dumps(d));raise SystemExit(0 if d['passed'] and r.returncode==0 else 1)
