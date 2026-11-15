"""Record actual Gazebo RGB frames and measured task state. No motion reconstruction."""
from common import *
import argparse,time,signal,sqlite3,math
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from PIL import Image as PILImage,ImageDraw,ImageFont
import imageio_ffmpeg

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();out=a.out;out.mkdir(parents=True,exist_ok=True);run=out.parent;spec=read(run/'spec.json');task=spec['task'];titles=dict(spray='Husky 点喷',inspection='Jackal 巡检',mowing='Dino 割草');font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',18)
 rclpy.init();n=rclpy.create_node('r12_video_observer');state=dict(stop=False,image=None,t=0.,wall=0.);proc=None;frames=0;first=None;deadline=0.;fps=5.;timeline=(out/'frames.jsonl').open('w');trail=[];last=None;ledger_t=-100;quantity=0.;jobs=0;fflog=None
 def image(m):
  if m.encoding not in ['rgb8','bgr8']:return
  pixels=np.frombuffer(bytes(m.data),np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
  if m.encoding=='bgr8':pixels=pixels[:,:,::-1]
  state.update(image=pixels.copy(),t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,wall=time.monotonic())
 n.create_subscription(Image,'/r12/observer/image',image,qos_profile_sensor_data)
 def stop(*_):state['stop']=True
 for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,stop)
 (out/'ready').write_text('Observer camera ROS domain 120. No actuator, localization, or safety authority.')
 try:
  while not state['stop']:
   rclpy.spin_once(n,timeout_sec=.01)
   if state['image'] is None:continue
   now=time.monotonic()
   if proc is None:
    first=now;deadline=now;fflog=(out/'encoder.log').open('w');argv=[imageio_ffmpeg.get_ffmpeg_exe(),'-hide_banner','-loglevel','warning','-y','-f','rawvideo','-pixel_format','rgb24','-video_size','960x650','-framerate',str(fps),'-i','-','-an','-c:v','libx264','-threads','2','-preset','ultrafast','-crf','27','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'full.mp4')];proc=subprocess.Popen(argv,stdin=subprocess.PIPE,stderr=fflog);dump(out/'encoder_command.json',dict(argv=argv,wall_pacing=True,source='actual Gazebo RGB; no synthetic trajectory'))
   if now<deadline:continue
   skipped=max(0,int((now-deadline)*fps));deadline+=(skipped+1)/fps
   try:live=read(run/'live.json')
   except (OSError,ValueError):live={}
   bt=live.get('bt') or {};loc=live.get('loc') or {};gate=live.get('gate') or {};phase=live.get('phase','STARTUP');reason=bt.get('reason_code','WAITING_FOR_ADMISSION')
   if (run/'mission_result.json').exists():
    terminal=read(run/'mission_result.json');phase=terminal.get('terminal','COMPLETED' if terminal.get('passed') else 'INCOMPLETE')
   if loc.get('valid') and 'pose' in loc and (last is None or math.dist(last,loc['pose'][:2])>.04):last=loc['pose'][:2];trail.append(last)
   if now-ledger_t>1 and (run/'ledger/ledger.sqlite').exists():
    ledger_t=now
    try:
     con=sqlite3.connect('file:'+str(run/'ledger/ledger.sqlite')+'?mode=ro',uri=True,timeout=.1);jobs,quantity=con.execute('select count(*),coalesce(sum(quantity),0) from jobs').fetchone();con.close()
    except sqlite3.Error:pass
   canvas=PILImage.new('RGB',(960,650),(17,27,37));canvas.paste(PILImage.fromarray(state['image']),(0,110));d=ImageDraw.Draw(canvas);age=now-state['wall']
   d.text((14,8),f'{titles[task]}  |  实际仿真录制 1×  |  独立展示，S17/S18 PARTIAL',font=font,fill=(231,241,248))
   d.text((14,40),f'仿真 {state["t"]:.1f}s   {phase}   {reason[:48]}',font=font,fill=(255,189,108) if bt.get('action') in ['HUMAN','STOP'] else (100,221,177))
   credit=f'反馈账本 {quantity:.2f} '+('剂量单位' if task=='spray' else 'm²')+f' / {jobs} 个动作' if task!='inspection' else f'观测运动累计 {gate.get("progress_m",0):.2f} m'
   d.text((14,73),credit+f'   速度 {gate.get("v",0):.2f} m/s'+('   画面已过期' if age>2 else ''),font=font,fill=(208,220,229))
   # Small measured-position trace only; no planned path presented as completed work.
   if len(trail)>1:
    d.rectangle((720,126,947,340),fill=(17,30,39));d.text((732,134),'传感器定位轨迹',font=font,fill=(204,221,228));p=np.asarray(trail);lo=p.min(axis=0)-.8;hi=p.max(axis=0)+.8;scale=min(195/(hi[0]-lo[0]),160/(hi[1]-lo[1]));xy=[(732+(x-lo[0])*scale,326-(y-lo[1])*scale) for x,y in p];d.line(xy,fill=(100,203,211),width=2);x,y=xy[-1];d.ellipse((x-4,y-4,x+4,y+4),fill=(255,202,85))
   data=np.asarray(canvas).tobytes()
   for k in range(skipped+1):
    proc.stdin.write(data);timeline.write(json.dumps(dict(frame=frames,video_s=frames/fps,sim_t=state['t'],wall_s=now-first,image_wall_age_s=age,phase=phase,bt_reason=reason,quantity=quantity,jobs=jobs,repeat_wall_slot=k>0))+'\n');frames+=1
   timeline.flush()
   if frames==skipped+1:canvas.save(out/'first_frame.png')
   if frames%40==0:canvas.save(out/'preview.png')
 finally:
  timeline.close();code=None
  if proc:
   proc.stdin.close()
   try:code=proc.wait(timeout=15)
   except subprocess.TimeoutExpired:proc.terminate();code=proc.wait(timeout=5)
   fflog.close()
  dump(out/'recording.json',dict(valid=bool(frames and code==0),frames=frames,fps=fps,duration_s=frames/fps,encoder_exit=code,source='actual Gazebo camera',sim_last=state['t'],no_generated_motion=True,wall_pacing=True));n.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
