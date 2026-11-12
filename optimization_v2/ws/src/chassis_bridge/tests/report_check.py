"""Real headless Chrome executes layer toggles and the recorded-time slider."""
import argparse,html,json,re,subprocess,tempfile
from pathlib import Path
def main():
    p=argparse.ArgumentParser();p.add_argument('report',type=Path);a=p.parse_args();out=a.report.parent
    javascript='''<script>
(async()=>{try {
 const cv=document.getElementById('map'), states={};
 const picture=()=>cv.toDataURL(); let before=picture();
 const sel=document.getElementById('case');states.all_cases=sel.options.length===25;
 sel.value='diff_connector';sel.dispatchEvent(new Event('change'));before=picture();
 const slider=document.getElementById('time');slider.value=Math.floor(+slider.max/2);slider.dispatchEvent(new Event('input'));states.measured_time_slider=before!==picture()&&+slider.max>10;
 before=picture();sel.value='ackermann_connector';sel.dispatchEvent(new Event('change'));states.vehicle_selection=before!==picture();
 states.qualified_text=document.body.textContent.includes('actors parked');
 states.no_failed_case=!Array.from(document.querySelectorAll('td')).some(x=>x.textContent==='FAIL');
 const old=+slider.value;document.getElementById('play').click();await new Promise(r=>setTimeout(r,200));document.getElementById('play').click();states.playback=+slider.value!==old;
 states.passed=Object.values(states).every(Boolean);
 let output=document.createElement('pre');output.id='s03_qa_result';output.textContent=JSON.stringify(states);document.body.appendChild(output);
} catch(error) {let output=document.createElement('pre');output.id='s03_qa_result';output.textContent=JSON.stringify({passed:false,error:String(error)});document.body.appendChild(output);}})();
</script>'''
    with tempfile.TemporaryDirectory(prefix='s03_report_') as tmp:
        tmp=Path(tmp);page=tmp/'qa.html';page.write_text(a.report.read_text().replace('</html>',javascript+'</html>'))
        args=['google-chrome','--headless','--disable-gpu','--no-first-run','--no-default-browser-check',f'--user-data-dir={tmp}/profile','--virtual-time-budget=1500','--dump-dom',page.as_uri()]
        r=subprocess.run(args,capture_output=True,text=True,timeout=30)
        m=re.search(r'<pre id="s03_qa_result">(.*?)</pre>',r.stdout)
        result=json.loads(html.unescape(m.group(1))) if m else dict(passed=False,error='Chrome did not produce QA DOM',exit_code=r.returncode)
        (out/'browser_check.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result));return 0 if result.get('passed') and r.returncode==0 else 1
if __name__=='__main__':raise SystemExit(main())
