"""Execute the actual report JavaScript in Chrome and verify interactions."""
import argparse,html,json,re,subprocess,tempfile
from pathlib import Path
def main():
    p=argparse.ArgumentParser();p.add_argument('report',type=Path);a=p.parse_args();out=a.report.parent
    script='''<script>
try {
const cv=document.getElementById('cloud'),sel=document.getElementById('case'),s=document.getElementById('time'),r=document.getElementById('rotation'),states={};
const picture=()=>cv.toDataURL();let before=picture();r.value=90;r.dispatchEvent(new Event('input'));states.rotation_changes_actual_cloud=before!==picture();
before=picture();s.value=s.max;s.dispatchEvent(new Event('input'));states.measured_snapshot_changes=+s.max>0&&before!==picture();
if(sel.options.length>1){before=picture();sel.selectedIndex=sel.options.length-1;sel.dispatchEvent(new Event('change'));states.case_changes_cloud=before!==picture();}
states.real_clouds_present=Object.values(data).every(v=>v.clouds.length>0&&v.clouds[0].xyz.length>100);
states.passed=Object.values(states).every(Boolean);let x=document.createElement('pre');x.id='s04_qa_result';x.textContent=JSON.stringify(states);document.body.appendChild(x);
}catch(error){let x=document.createElement('pre');x.id='s04_qa_result';x.textContent=JSON.stringify({passed:false,error:String(error)});document.body.appendChild(x);}
</script>'''
    with tempfile.TemporaryDirectory(prefix='s04_report_') as tmp:
        tmp=Path(tmp);page=tmp/'qa.html';page.write_text(a.report.read_text().replace('<meta charset="utf-8">','<meta charset="utf-8"><base href="'+out.as_uri()+'/">').replace('</html>',script+'</html>'))
        args=['google-chrome','--headless','--disable-gpu','--no-first-run','--no-default-browser-check',f'--user-data-dir={tmp}/profile','--virtual-time-budget=2000','--window-size=1440,1100',f'--screenshot={out}/report_preview.png','--dump-dom',page.as_uri()]
        r=subprocess.run(args,capture_output=True,text=True,timeout=40);m=re.search(r'<pre id="s04_qa_result">(.*?)</pre>',r.stdout)
        result=json.loads(html.unescape(m.group(1))) if m else dict(passed=False,error='Chrome did not produce QA DOM',exit_code=r.returncode)
        result['command']=args;result['exit_code']=r.returncode;(out/'browser_check.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result));return 0 if result.get('passed') and r.returncode==0 else 1
if __name__=='__main__':raise SystemExit(main())
