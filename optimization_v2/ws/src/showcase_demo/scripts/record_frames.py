"""Actual Gazebo image recording with real-wall pacing and an audited timeline."""
from common_showcase import *
import argparse,signal,subprocess,math
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from PIL import Image as PILImage,ImageDraw,ImageFont
import imageio_ffmpeg

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--fps',type=float,default=5.);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('showcase_visual_recorder');st=dict(stop=False,image=None,t=None,received=None);count=0;proc=None;started=time.monotonic();first=None;missed=0
    font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',18)
    spec=read(a.out.parent/'spec.json');title=('连接路巡检' if spec['operation']=='inspection' else '差速点服务' if spec['vehicle']=='diff' else '阿克曼条带割草')+' · '+spec['cluster']
    from video_map import Inset
    inset=Inset(spec)
    def frame(m):
        if m.encoding not in ['rgb8','bgr8']:return
        arr=np.frombuffer(bytes(m.data),np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='bgr8':arr=arr[:,:,::-1]
        st.update(image=arr.copy(),t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,received=time.monotonic())
    n.create_subscription(Image,'/showcase/observer/image',frame,qos_profile_sensor_data)
    def stop(s,f):st['stop']=True
    for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
    (a.out/'ready').write_text('Observer image only; no robot command publisher')
    timeline=(a.out/'frames.jsonl').open('w');deadline=None
    try:
        while not st['stop']:
            rclpy.spin_once(n,timeout_sec=.01)
            if st['image'] is None:continue
            now=time.monotonic()
            if proc is None:
                h,w=st['image'].shape[:2];cmd=[imageio_ffmpeg.get_ffmpeg_exe(),'-hide_banner','-loglevel','warning','-y','-f','rawvideo','-pixel_format','rgb24','-video_size',f'{w}x{h+110}','-framerate',str(a.fps),'-i','-','-an','-c:v','libx264','-preset','ultrafast','-crf','26','-pix_fmt','yuv420p','-movflags','+faststart',str(a.out/'full.mp4')]
                proc=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=(a.out/'encoder.log').open('w'));first=now;deadline=now;dump(a.out/'encoder_command.json',dict(argv=cmd,source='actual Gazebo camera',realtime_wall_pacing=True,speedup=1.))
            if now<deadline:continue
            # A slow encoder is visible as a dropped-slot counter; no simulated
            # position is invented or interpolated for visual evidence.
            skipped=max(0,int((now-deadline)*a.fps));missed+=skipped;deadline+=(skipped+1)/a.fps
            monitor={}
            try:monitor=read(a.out.parent/'client/motion_monitor.json')
            except (OSError,ValueError):pass
            canvas=PILImage.new('RGB',(w,h+110),(16,25,37));canvas.paste(PILImage.fromarray(st['image']),(0,110));draw=ImageDraw.Draw(canvas)
            name=a.out.parent.name;state=monitor.get('state','STARTING');age=now-st['received']
            draw.text((14,8),f'{title}  |  实时录制 1×  |  独立演示，不计正式验收',font=font,fill=(224,238,248))
            draw.text((14,40),f'仿真 {st["t"]:.1f}s   {state}   {monitor.get("bt_reason") or "等待准入"}'+('  画面数据过期' if age>2 else ''),font=font,fill=(255,194,116) if monitor.get('requires_manual') or age>2 else (105,223,180))
            ledger={}
            try:ledger=read(a.out.parent/'client/mission.json')
            except (OSError,ValueError):pass
            credit=('点服务 '+str(ledger.get('confirmed_points',0))+'/'+str(ledger.get('required_points','?'))) if spec['vehicle']=='diff' else ('确认覆盖 '+(str(round(100*ledger['coverage_fraction'],1))+'%' if ledger.get('coverage_fraction') is not None else '—'))
            draw.text((14,72),credit+'   工具 '+str(monitor.get('tool',{}).get('phase','等待反馈'))+'   已完成动作 '+str(monitor.get('completed_jobs',0)),font=font,fill=(194,211,225))
            canvas.paste(inset.render(monitor,ledger),(w-262,122))
            data=np.asarray(canvas).tobytes()
            for duplicate in range(skipped+1):
                proc.stdin.write(data);timeline.write(json.dumps(dict(frame=count,video_s=count/a.fps,wall_since_first_s=now-first,sim_t=st['t'],image_wall_age_s=age,state=state,job=monitor.get('job'),completed_jobs=monitor.get('completed_jobs'),reason=monitor.get('reason'),repeated_for_wall_pacing=duplicate>0))+'\n');count+=1
            timeline.flush()
            if count==skipped+1:canvas.save(a.out/'first_frame.png')
            if count%50==0:canvas.save(a.out/'latest_frame.png')
    finally:
        timeline.close()
        code=None
        if proc:
            proc.stdin.close()
            try:code=proc.wait(timeout=12)
            except subprocess.TimeoutExpired:proc.terminate();code=proc.wait(timeout=5)
        dump(a.out/'recording.json',dict(frames=count,fps=a.fps,encoded_seconds=count/a.fps,source_wall_seconds=time.monotonic()-first if first else None,repeated_wall_slots=missed,encoder_exit=code,sim_last=st['t'],valid=bool(count and code==0),no_generated_motion=True))
        n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
