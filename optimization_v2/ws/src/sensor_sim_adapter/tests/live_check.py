"""Verify actual loopback dashboard, sensor images and measured rate UI during a run."""
import argparse,html,json,re,subprocess,tempfile,time,urllib.request,urllib.error
from pathlib import Path
from io import BytesIO
from PIL import Image
def main():
    p=argparse.ArgumentParser();p.add_argument('case',type=Path);p.add_argument('--after',type=float,default=25);p.add_argument('--wait',type=float,default=180);a=p.parse_args();online=a.case/'online';deadline=time.monotonic()+a.wait
    while time.monotonic()<deadline:
        try:
            url=(online/'live_url.txt').read_text().strip()
            with urllib.request.urlopen(url+'live.json',timeout=2) as f:d=json.load(f)
            if d['t']>=a.after:break
        except (OSError,ValueError):pass
        time.sleep(.2)
    else:raise TimeoutError('live panel did not reach capture time')
    result=dict(url=url,simulation_time=d['t'],counts=d['counts'],cloud_points=len(d['cloud']))
    result['actual_images']=True
    for key in ['left','right']:
        with urllib.request.urlopen(url+key+'_latest.png',timeout=2) as f:im=Image.open(BytesIO(f.read()));result['actual_images'] &= im.size==(640,360)
    try:
        urllib.request.urlopen(url+'generation.jsonl',timeout=2);result['file_allowlist']=False
    except urllib.error.HTTPError as e:result['file_allowlist']=e.code==404
    with tempfile.TemporaryDirectory(prefix='s04_live_') as tmp:
        args=['google-chrome','--headless','--disable-gpu','--no-first-run','--no-default-browser-check',f'--user-data-dir={tmp}/profile','--virtual-time-budget=6000','--window-size=1440,1250',f'--screenshot={a.case}/live_preview.png','--dump-dom',url]
        r=subprocess.run(args,capture_output=True,text=True,timeout=30)
        status=re.search(r'<div id="status">(.*?)</div>',r.stdout);rates=re.search(r'<div id="rates"[^>]*>(.*?)</div>',r.stdout)
        result.update(command=args,exit_code=r.returncode,status=html.unescape(status.group(1)) if status else '',rates=html.unescape(rates.group(1)) if rates else '')
        result['rendered_live_data']='仿真时间' in result['status'] and all(k in result['rates'] for k in ['IMU','轮速','GNSS'])
    result['passed']=all(result[k] for k in ['actual_images','file_allowlist','rendered_live_data']) and r.returncode==0 and len(d['cloud'])>100
    (a.case/'live_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
