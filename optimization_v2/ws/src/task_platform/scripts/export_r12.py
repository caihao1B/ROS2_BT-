"""Export full real recordings and explicitly accelerated overview cuts."""
from common import *
import math
import imageio_ffmpeg
from highlights_r12 import select as select_highlight

def export(names):
 out=APP/'results/R12/media';out.mkdir(parents=True,exist_ok=True);ffmpeg=imageio_ffmpeg.get_ffmpeg_exe();episodes=[];videos=[];shorts=[];behaviors=[]
 def execute(args,name):
  with (out/(name+'.log')).open('w') as f:r=subprocess.run([ffmpeg,'-hide_banner','-y',*args],stdout=f,stderr=subprocess.STDOUT,timeout=600)
  if r.returncode:raise RuntimeError('VIDEO_EXPORT_FAILED '+name)
 def speed_header(speed):
  return f"drawbox=x=0:y=0:w=iw:h=36:color=0x111b25:t=fill,drawtext=fontfile=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc:text='真实原片 {speed}倍速展示 / 原始实速视频另附 / S17-S18 PARTIAL':fontcolor=white:fontsize=18:x=14:y=8"
 for name in names:
  receipt=read(APP/'results/R12'/(name+'_latest.json'));run=ROOT/receipt['path'];rec=read(run/'video/recording.json');video=run/'video/full.mp4'
  if not rec['valid'] or not receipt['mission_timeline_complete'] or not receipt['active_video_quality'] or sha(video)!=receipt['source_video_sha256']:raise ValueError('VIDEO_TIMELINE_QUALITY_OR_HASH_INVALID '+name)
  timeline=[json.loads(l) for l in (run/'video/frames.jsonl').read_text().splitlines()]
  if any(b['sim_t']<a['sim_t'] or b['video_s']<=a['video_s'] for a,b in zip(timeline,timeline[1:])):raise ValueError('VIDEO_CLOCK_REGRESSION')
  task=read(run/'mission_result.json');spec=read(run/'spec.json')
  if receipt['task']=='spray':scope=f"A1 选择 {task['selected_points']}/{task['original_points']} 点"
  elif receipt['task']=='mowing':
   scope=f"B1 选择 {task['selected_segments']}/{task['original_segments']} 段"
   if task.get('safe_plan_revision'):scope=f"安全 B1 选择 {task['selected_segments']}/{task['execution_segments']} 段 / 旧34段 PARTIAL"
  else:scope='完整原连接路' if not spec.get('distance_limit_m') else f"巡迹专项 / 路段上限 {spec['distance_limit_m']:g} m"
  videos.append(video);speed=max(1,math.ceil(rec['duration_s']/45));short=out/(name+'_overview.mp4')
  count=len(task.get('completed',task.get('completed_segments',[])));selected=task.get('selected_points',task.get('selected_segments'))
  verdict='任务通过' if receipt['task_passed'] else '未完成 / PARTIAL'
  progress=f'完成 {count}/{selected}' if selected is not None else verdict
  version=read(run/'mission.json').get('version','unknown')[:8]
  label=out/(name+'_overview.txt');label.write_text(f'{name} / {verdict} / {progress} / {speed}倍速概览\n{scope} / 计划 {version}')
  filters=f"setpts=(PTS-STARTPTS)/{speed},fps=20,"+speed_header(speed)+f",drawtext=fontfile=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc:textfile='{label}':fontcolor=white:fontsize=16:line_spacing=3:box=1:boxcolor=black@0.8:x=14:y=h-52"
  execute(['-i',str(video),'-an','-vf',filters,'-c:v','libx264','-threads','2','-preset','fast','-crf','25','-pix_fmt','yuv420p','-movflags','+faststart',str(short)],name+'_overview');shorts.append(short)
  episodes.append(dict(case=name,run=receipt['path'],task_passed=receipt['task_passed'],displayed_scope=scope,full_path=str(video.relative_to(ROOT)),full_sha256=sha(video),full_seconds=rec['duration_s'],sim_first=timeline[0]['sim_t'],sim_last=timeline[-1]['sim_t'],overview_speed=speed,overview_path=str(short.relative_to(ROOT)),overview_sha256=sha(short),timeline_complete=True))
  events=[json.loads(l) for l in (run/'mission_events.jsonl').read_text().splitlines()];analysis=read(run/'analysis_r08.json') if name.startswith('inspection') else {}
  excerpt=select_highlight(name,events,timeline,analysis)
  if excerpt:
   target=out/(name+'_behavior.mp4');caption=out/(name+'_behavior.txt');caption.write_text(f'{verdict} / {progress} / '+scope+'\n'+excerpt['label']+f" / {excerpt['speed']}倍速 / 真实原片片段")
   filters=f"setpts=(PTS-STARTPTS)/{excerpt['speed']},fps=20,"+speed_header(excerpt['speed'])+f",drawtext=fontfile=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc:textfile='{caption}':fontcolor=white:fontsize=18:line_spacing=3:box=1:boxcolor=black@0.85:x=14:y=h-58"
   execute(['-ss',str(excerpt['start_video_s']),'-i',str(video),'-t',str(excerpt['duration_video_s']/excerpt['speed']),'-an','-vf',filters,'-c:v','libx264','-threads','2','-preset','fast','-crf','25','-pix_fmt','yuv420p','-movflags','+faststart',str(target)],name+'_behavior');behaviors.append(target);episodes[-1]['behavior_excerpt']=dict(excerpt,path=str(target.relative_to(ROOT)),sha256=sha(target))
 def concat(paths,name):
  listing=out/(name+'.txt');listing.write_text(''.join("file '"+str(p).replace("'","'\\''")+"'\n" for p in paths));target=out/(name+'.mp4');execute(['-f','concat','-safe','0','-i',str(listing),'-c','copy','-movflags','+faststart',str(target)],name);return target
 full=concat(videos,'full_recordings');short=concat(shorts,'three_tasks_short')
 summary=dict(episodes=episodes,full=dict(path=str(full.relative_to(ROOT)),sha256=sha(full),speed=1),short=dict(path=str(short.relative_to(ROOT)),sha256=sha(short),speed='per-episode labeled acceleration'),no_generated_scene_motion=True,formal_S17_S18='PARTIAL',scope='Complete recorded execution intervals, including actual holds/failures; full recording does not mean full work completion')
 if behaviors:
  highlights=concat(behaviors,'behavior_highlights');summary['behaviors']=dict(path=str(highlights.relative_to(ROOT)),sha256=sha(highlights),speed='per-excerpt labeled acceleration')
 dump(out/'manifest.json',summary)
 body='<!doctype html><html lang="zh"><meta charset="utf-8"><title>三任务真实录像</title><style>body{background:#14222d;color:#e4eef2;font:17px/1.7 system-ui;max-width:1100px;margin:32px auto}video{width:100%;background:#000}a{color:#87dcd1}section{margin:32px 0}</style><h1>三任务真实仿真录像</h1><p>原始实速录像与明确标注倍速的精简版。失败和停车保留；S17/S18 仍为 PARTIAL。</p><video controls preload="metadata" src="three_tasks_short.mp4"></video>'
 if behaviors:body+='<section><h2>核心行为片段</h2><p>按真实事件截取；每段标明倍速。原始完整记录见下方。</p><video controls preload="metadata" src="behavior_highlights.mp4"></video></section>'
 for e in episodes:
  rel=os.path.relpath(ROOT/e['full_path'],out);body+='<section><h2>'+html.escape(e['case'])+' · '+('通过本例检查' if e['task_passed'] else '未完成 / 保留失败证据')+'</h2><video controls preload="metadata" src="'+html.escape(rel)+'"></video><p>实速 '+str(round(e['full_seconds'],1))+' 秒，仿真时钟 '+str(round(e['sim_last']-e['sim_first'],1))+' 秒。</p></section>'
 (out/'index.html').write_text(body);print(json.dumps(summary,ensure_ascii=False));return summary
