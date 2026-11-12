#!/usr/bin/env python3
"""Verify only this learning artifact. Does not start ROS/Gazebo or change stages."""
import functools
import hashlib
import html
import http.server
import json
import os
import re
import shutil
import subprocess
import tempfile
import threading
from datetime import datetime, timezone
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlparse

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]

HARNESS=r'''<!doctype html><meta charset="utf-8"><title>Learning artifact QA</title>
<iframe id="site" src="index.html" style="width:1100px;height:900px;border:0"></iframe>
<pre id="qa-result">PENDING</pre><script>
const iframe=document.getElementById('site');
function sleep(ms){return new Promise(r=>setTimeout(r,ms));}
async function run(){
const checks=[];
try{
for(let i=0;i<60&&!iframe.contentWindow.learningReady;i++)await sleep(50);
const w=iframe.contentWindow,d=w.document,$=id=>d.getElementById(id);
const check=(name,condition,detail='')=>{checks.push({name,passed:!!condition,detail});if(!condition)throw Error(name+': '+detail);};
const input=(id,value)=>{$(id).value=value;$(id).dispatchEvent(new w.Event('input',{bubbles:true}));};
check('app initialized',w.learningReady);
check('no JavaScript errors',w.learningErrors.length===0,JSON.stringify(w.learningErrors));
$('wheel-normal').click();const vn=Number($('velocity-result').dataset.value);
$('wheel-slip').click();const vs=Number($('velocity-result').dataset.value);
check('wheel covariance changes estimate',Math.abs(vn-0.63238095238)<1e-6&&Math.abs(vs-0.41656686627)<1e-6);
input('gnss-offset',16);check('16cm rejected despite NIS passing',$('gnss-result').dataset.accepted==='false'&&Number($('nis-value').textContent)<9.21);
input('gnss-offset',10);check('10cm passes two gates',$('gnss-result').dataset.accepted==='true');
input('loop-weight',0);check('no loop retains odometric endpoint',Math.abs(Number($('graph-x2').dataset.value)-2.2)<1e-9);
input('loop-weight',1);check('weighted loop solves normal equations',Math.abs(Number($('graph-x2').dataset.value)-2.0666666667)<1e-9);
input('tf-yaw',0);check('translation composition',Math.abs(Number($('tf-x').textContent)-2.4)<1e-9&&Math.abs(Number($('tf-y').textContent)-.8)<1e-9);
input('tf-yaw',10);check('rotation composition and unchanged local pose',Math.abs(Number($('tf-x').textContent)-2.196)<1e-9&&$('tf-local').textContent==='2.000, 1.000');
$('tab-regional').click();check('regional branch tab',$('tab-regional').getAttribute('aria-selected')==='true'&&$('flow-nodes').textContent.includes('SE(2)'));
$('tab-regional').dispatchEvent(new w.KeyboardEvent('keydown',{key:'ArrowRight',bubbles:true}));check('keyboard tab navigation',$('tab-local').getAttribute('aria-selected')==='true');
input('source-search','regional_math14');check('source search filters accurately',d.querySelectorAll('#sources tbody tr:not([hidden])').length===1);
input('source-search','');check('all 32 source files visible',d.querySelectorAll('#sources tbody tr:not([hidden])').length===32);
const quiz=d.querySelector('.quiz-q');quiz.querySelector('[data-choice="false"]').click();check('quiz explanation',quiz.querySelector('output').dataset.correct==='true');
check('chapter anchors resolve',[...d.querySelectorAll('a[href^="#"]')].every(a=>d.getElementById(a.hash.slice(1))));
const chapter=d.querySelector('.sidebar a[href="#chapter-03"]');chapter.click();await sleep(150);
check('chapter navigation scrolls to content',w.scrollY>1000&&Math.abs($('chapter-03').getBoundingClientRect().top-25)<3);
check('copy controls present',d.querySelectorAll('button.copy').length>=6);
for(const width of [1100,760,390,320]){
iframe.style.width=width+'px';await sleep(100);
check('no page overflow at '+width,d.documentElement.scrollWidth<=d.documentElement.clientWidth+1,
      d.documentElement.scrollWidth+' / '+d.documentElement.clientWidth);
}
check('no late JavaScript errors',w.learningErrors.length===0);
document.getElementById('qa-result').textContent=JSON.stringify({status:'PASS',checks});
}catch(error){document.getElementById('qa-result').textContent=JSON.stringify({status:'FAIL',error:String(error),checks});}
}
iframe.addEventListener('load',run);
</script>'''


class Links(HTMLParser):
    def __init__(self):super().__init__();self.links=[]
    def handle_starttag(self,tag,attrs):
        for key,value in attrs:
            if (tag=='a' and key=='href') or (tag in ['script','link'] and key in ['src','href']):self.links.append(value)


class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self,*args):pass


def run(cmd,timeout=30):
    r=subprocess.run(cmd,text=True,capture_output=True,timeout=timeout,env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1'})
    if r.returncode:raise RuntimeError(f'{cmd[0]} exited {r.returncode}: {r.stderr[-2000:]}')
    return r


def main():
    out=HERE/'checks';out.mkdir(exist_ok=True)
    report=dict(scope='LEARNING_MATERIAL_ONLY; NOT_A_ROBOT_STAGE_RECEIPT',
                checked_utc=datetime.now(timezone.utc).isoformat(),status='RUNNING',
                production_mutations=False,ros_or_gazebo_started=False)
    (HERE/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    snapshot=json.loads((HERE/'SOURCE_SNAPSHOT.json').read_text())
    changed=[]
    for item in snapshot['files']:
        if hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()!=item['sha256']:changed.append(item['path'])
    report['source_snapshot_current']=not changed;report['changed_source_files']=changed

    py=run(['/usr/bin/python3',str(HERE/'examples/learning_lab.py')])
    (out/'python_lab.txt').write_text(py.stdout)
    report['python_lab']='PASS' if 'PASS: 5 ' in py.stdout else 'FAIL'
    with tempfile.TemporaryDirectory(prefix='robot-learning-build-') as build:
        binary=str(Path(build)/'pose_correction')
        compiler=run(['g++','-std=c++17','-O2','-Wall','-Wextra','-pedantic',str(HERE/'examples/pose_correction.cpp'),'-o',binary])
        cpp=run([binary]);(out/'cpp_lab.txt').write_text(cpp.stdout)
        report['cpp_lab']='PASS' if 'PASS:' in cpp.stdout else 'FAIL'
        report['cpp_compiler_warnings']=compiler.stderr

    links=Links();links.feed((HERE/'index.html').read_text())
    missing=[]
    for link in links.links:
        url=urlparse(link)
        if url.scheme or not url.path:continue
        if not (HERE/unquote(url.path)).resolve().exists():missing.append(link)
    report['missing_local_links']=missing
    assert not missing,missing

    handler=functools.partial(Quiet,directory=str(ROOT))
    server=http.server.ThreadingHTTPServer(('127.0.0.1',0),handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    harness=HERE/'_browser_qa.html';harness.write_text(HARNESS)
    chrome=shutil.which('google-chrome') or shutil.which('chromium')
    if not chrome:raise RuntimeError('No existing Chrome/Chromium available; no installation attempted')
    try:
        base=f'http://127.0.0.1:{server.server_port}/docs/optimization_v2/learning_localization/'
        with tempfile.TemporaryDirectory(prefix='robot-learning-chrome-') as profile:
            flags=[chrome,'--headless','--disable-gpu','--disable-background-networking','--no-first-run',
                   '--no-default-browser-check','--disable-dev-shm-usage','--force-device-scale-factor=1',
                   '--user-data-dir='+profile,'--virtual-time-budget=5000']
            dom=run(flags+['--window-size=1400,1000','--dump-dom',base+'_browser_qa.html'])
            result=re.search(r'<pre id="qa-result">(.*?)</pre>',dom.stdout,re.S)
            if not result:raise RuntimeError('Browser QA marker missing')
            report['browser']=json.loads(html.unescape(result[1]))
            (out/'browser_checks.json').write_text(json.dumps(report['browser'],ensure_ascii=False,indent=2)+'\n')
            assert report['browser']['status']=='PASS',report['browser']
            run(['/usr/bin/python3',str(HERE/'capture_browser.py')])
            report['screenshots']=['checks/'+x+'.png' for x in ['desktop','mobile','experiment']]
    finally:
        server.shutdown();server.server_close();harness.unlink(missing_ok=True)
    # Detect concurrent changes; never overwrite production sources to restore a snapshot.
    report['changed_source_files_after_check']=[x['path'] for x in snapshot['files']
        if hashlib.sha256((ROOT/x['path']).read_bytes()).hexdigest()!=x['sha256']]
    report['status']='PASS' if report['python_lab']=='PASS' and report['cpp_lab']=='PASS' and not missing and not report['changed_source_files_after_check'] else 'REVIEW'
    (HERE/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='browser'},ensure_ascii=False,indent=2))
    print('Browser checks:',len(report['browser']['checks']))


if __name__=='__main__':main()
