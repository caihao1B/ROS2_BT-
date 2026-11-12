"""Execute the actual report JavaScript in Chrome and verify interactions."""
import argparse,html,json,re,subprocess,tempfile
from pathlib import Path
def main():
    p=argparse.ArgumentParser();p.add_argument('report',type=Path);a=p.parse_args();a.report=a.report.resolve();out=a.report.parent
    script='''<script>
try{let cv=document.getElementById('trajectory'),s=document.getElementById('time'),sel=document.getElementById('case'),before=cv.toDataURL();s.value=20;s.dispatchEvent(new Event('input'));let states={time_cursor_changes_actual_trajectory:before!==cv.toDataURL(),measured_data_present:Object.values(data).every(d=>d.paths.fused.t.length>10),rendered:window.s05Rendered===true};if(sel.options.length>1){before=cv.toDataURL();sel.selectedIndex=sel.options.length-1;sel.dispatchEvent(new Event('change'));states.case_changes_trajectory=before!==cv.toDataURL();}states.passed=Object.values(states).every(Boolean);let p=document.createElement('pre');p.id='s05_qa_result';p.textContent=JSON.stringify(states);document.body.append(p);}catch(e){let p=document.createElement('pre');p.id='s05_qa_result';p.textContent=JSON.stringify({passed:false,error:String(e)});document.body.append(p);}
</script>'''
    with tempfile.TemporaryDirectory(prefix='s05_report_') as tmp:
        tmp=Path(tmp);page=tmp/'qa.html';page.write_text(a.report.read_text().replace('<meta charset="utf-8">','<meta charset="utf-8"><base href="'+out.as_uri()+'/">').replace('</html>',script+'</html>'))
        args=['google-chrome','--headless','--disable-gpu','--no-first-run','--no-default-browser-check',f'--user-data-dir={tmp}/profile','--virtual-time-budget=2000','--window-size=1440,1100',f'--screenshot={out}/report_preview.png','--dump-dom',page.as_uri()]
        r=subprocess.run(args,capture_output=True,text=True,timeout=40);m=re.search(r'<pre id="s05_qa_result">(.*?)</pre>',r.stdout)
        result=json.loads(html.unescape(m.group(1))) if m else dict(passed=False,error='Chrome did not produce QA DOM',exit_code=r.returncode)
        result['command']=args;result['exit_code']=r.returncode;(out/'browser_check.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result));return 0 if result.get('passed') and r.returncode==0 else 1
if __name__=='__main__':raise SystemExit(main())
