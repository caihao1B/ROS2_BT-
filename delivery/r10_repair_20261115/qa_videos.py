"""Decode the delivered exports and complete-task attempts, preserving their verdicts."""
from pathlib import Path
import json,subprocess
import imageio_ffmpeg

ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent/'qa';OUT.mkdir(exist_ok=True)
APP=ROOT/'optimization_v2/three_tasks';media=APP/'results/R12/media';manifest=json.loads((media/'manifest.json').read_text());ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
files=[ROOT/manifest[k]['path'] for k in ['full','short','behaviors']]
for name in ['spray_full_A1','mowing_full_B1']:
    receipt=json.loads((APP/'results/R12'/(name+'_latest.json')).read_text());files.append(ROOT/receipt['path']/'video/full.mp4')
results=[]
for index,path in enumerate(files):
    stream=imageio_ffmpeg.read_frames(str(path));meta=next(stream);stream.close()
    result=subprocess.run([ffmpeg,'-v','error','-threads','2','-i',str(path),'-an','-f','null','-'],capture_output=True,text=True,timeout=180)
    results.append(dict(path=str(path.relative_to(ROOT)),duration_s=meta['duration'],fps=meta['fps'],size=meta['size'],decode_exit=result.returncode,errors=result.stderr,passed=result.returncode==0 and not result.stderr.strip(),scope='File decode only; task completeness and active frame freshness remain separately scored'))
    print(json.dumps(results[-1]),flush=True)
for name,when in [('three_tasks_short',20),('behavior_highlights',135)]:
    subprocess.run([ffmpeg,'-v','error','-y','-ss',str(when),'-i',str(media/(name+'.mp4')),'-frames:v','1',str(OUT/(name+'.png'))],check=True,timeout=30)
(OUT/'video_checks.json').write_text(json.dumps(dict(passed=all(r['passed'] for r in results),videos=results),indent=2)+'\n')
if not all(r['passed'] for r in results):raise SystemExit(2)
