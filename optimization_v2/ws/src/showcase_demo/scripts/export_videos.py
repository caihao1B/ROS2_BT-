"""Concatenate complete actual recordings; label accelerated presentation cuts."""
from common_showcase import *
import argparse,subprocess,math
import imageio_ffmpeg

def command(args,log):
    with log.open('w') as f:result=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT,timeout=1200)
    if result.returncode:raise RuntimeError('VIDEO_COMMAND_FAILED '+str(log))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--runs',type=Path,nargs='+',required=True);ap.add_argument('--out',type=Path,default=OUT/'media');a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);ffmpeg=imageio_ffmpeg.get_ffmpeg_exe();evidence=[];videos=[]
    expanded=[]
    for run in a.runs:
        result=read(run.resolve()/'showcase_result.json')
        for episode in result.get('execution_episodes',[str(run.resolve().relative_to(ROOT))]):
            path=(ROOT/episode).resolve()
            if not path.is_relative_to(BASE/'results/S17') or path in expanded:raise ValueError('INVALID_OR_DUPLICATE_VIDEO_EPISODE')
            expanded.append(path)
    a.runs=expanded
    for run in a.runs:
        run=run.resolve();rec=read(run/'video/recording.json');result=read(run/'showcase_result.json');spec=read(run/'spec.json');times=[json.loads(line) for line in (run/'video/frames.jsonl').read_text().splitlines()]
        if not rec['valid'] or not times or not (run/'video/full.mp4').is_file():raise ValueError('RECORDING_NOT_VALID '+str(run))
        monitor=read(run/'client/motion_monitor.json');events=[json.loads(line) for line in (run/'client/events.jsonl').read_text().splitlines()];admitted=next((e['t'] for e in events if e['event']=='mission_admitted'),None)
        complete_timeline=admitted is not None and times[0]['sim_t']<=admitted and times[-1]['sim_t']>=monitor['t']-.3
        if not complete_timeline:raise ValueError('VIDEO_DOES_NOT_COVER_ADMISSION_THROUGH_TERMINAL '+str(run))
        video=run/'video/full.mp4';videos.append(video);evidence.append(dict(run=str(run.relative_to(ROOT)),cluster=spec['cluster'],operation=spec['operation'],status=result['status'],source_sha256=sha(video),duration_s=rec['encoded_seconds'],sim_first=times[0]['sim_t'],sim_last=times[-1]['sim_t'],terminal_sim_t=monitor['t'],complete_mission_timeline=complete_timeline,repair_probe=spec.get('repair_probe',False)))
    listing=a.out/'concat.txt';listing.write_text(''.join("file '"+str(p).replace("'","'\\''")+"'\n" for p in videos))
    full=a.out/'full_demo.mp4';command([ffmpeg,'-hide_banner','-y','-f','concat','-safe','0','-i',str(listing),'-c','copy','-movflags','+faststart',str(full)],a.out/'concat.log')
    if full.stat().st_size>90_000_000:
        source=a.out/'full_source_concat.mp4';full.replace(source)
        command([ffmpeg,'-hide_banner','-y','-i',str(source),'-an','-c:v','libx264','-preset','fast','-crf','28','-pix_fmt','yuv420p','-movflags','+faststart',str(full)],a.out/'full_encode.log')
    total=sum(e['duration_s'] for e in evidence);speed=max(1,math.ceil(total/185));short=a.out/'showcase_short.mp4';fast=a.out/'accelerated_overview.mp4'
    filters=f"setpts=(PTS-STARTPTS)/{speed},fps=25,drawtext=fontfile=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc:text='精简版 {speed}倍速 · 原始时钟见画面 · S17 仍为 PARTIAL':fontcolor=white:fontsize=19:box=1:boxcolor=black@0.75:x=12:y=h-30"
    command([ffmpeg,'-hide_banner','-y','-i',str(full),'-an','-vf',filters,'-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(fast)],a.out/'highlight.log')
    clips=[];seen=set()
    for run in a.runs:
        run=run.resolve();spec=read(run/'spec.json');events=[json.loads(l) for l in (run/'client/events.jsonl').read_text().splitlines()];frames=[json.loads(l) for l in (run/'video/frames.jsonl').read_text().splitlines()];targets=[]
        tool=next((e for e in events if e['event']=='tool_result' and e.get('quantity',0)>0 and not e.get('uncertain')),None)
        if tool:targets.append(('tool_'+spec['vehicle'],'工具反馈确认',tool['t']))
        replan=next((e for e in events if e['event']=='replan_attempt'),None)
        if replan:targets.append(('replan','BT 停车与重新规划',replan['t']))
        patch=next((e for e in events if e['event']=='patch_planning_started'),None)
        if patch:targets.append(('residual_patch','真实残余触发补作规划',patch['t']))
        wait=next((e for e in events if e['event']=='observation_admission_wait'),None)
        if wait:targets.append(('observation_wait','感知准入暂失：停车等待恢复',wait['t']))
        handoff_path=run/'client/manual_handoff.json'
        if handoff_path.exists():
            handoff=read(handoff_path)
            if handoff['requires_manual'] and handoff['stop_confirmed']:targets.append(('live_handoff','有界停止并请求人工检查',handoff['t']))
        actors=run/'scene/actor_interactions.jsonl'
        if actors.exists():
            actor=next((json.loads(l) for l in actors.read_text().splitlines() if json.loads(l).get('yielding')),None)
            if actor:targets.append(('interaction','动态障碍局部停让',actor['t']))
        for key,label,t in targets:
            if key in seen:continue
            seen.add(key);start=max(0.,min(frames,key=lambda f:abs(f['sim_t']-(t-3)))['video_s']);path=a.out/(key+'.mp4');caption="fps=25,drawtext=fontfile=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc:text='实速回看 1倍 · "+label+"':fontcolor=white:fontsize=21:box=1:boxcolor=black@0.8:x=12:y=h-32"
            command([ffmpeg,'-hide_banner','-y','-ss',str(start),'-i',str(run/'video/full.mp4'),'-t','8','-an','-vf',caption,'-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],a.out/(key+'.log'));clips.append(dict(key=key,path=path,source_run=str(run.relative_to(ROOT)),event_sim_t=t,source_video_start_s=start,speed=1,duration_s=8))
    # This is a separate, real failed attempt, not credited task completion.
    failure=BASE/'results/S17/showcase_B3_work_20261113_201908'
    handoff_file=failure/'client/manual_handoff.json'
    if handoff_file.exists():
        handoff=read(handoff_file);rec=read(failure/'video/recording.json')
        if handoff.get('stop_confirmed') and handoff.get('reason')=='SUPERVISOR_NO_PROGRESS_60S_MANUAL_CHECK' and rec['valid']:
            frames=[json.loads(l) for l in (failure/'video/frames.jsonl').read_text().splitlines()]
            start=max(0.,min(frames,key=lambda f:abs(f['sim_t']-(handoff['t']-3)))['video_s'])
            path=a.out/'measured_stall_handoff.mp4'
            caption="fps=25,drawtext=fontfile=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc:text='实速失败回看 · 60秒无进展报警与停车 · 不计完成':fontcolor=white:fontsize=20:box=1:boxcolor=black@0.8:x=12:y=h-32"
            command([ffmpeg,'-hide_banner','-y','-ss',str(start),'-i',str(failure/'video/full.mp4'),'-t','8','-an','-vf',caption,'-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],a.out/'measured_stall_handoff.log')
            clips.append(dict(key='measured_stall_handoff',path=path,source_run=str(failure.relative_to(ROOT)),source_sha256=sha(failure/'video/full.mp4'),source_handoff_sha256=sha(handoff_file),event_sim_t=handoff['t'],source_video_start_s=start,speed=1,duration_s=8,task_credit=False))
    screenshot=OUT/'monitor_preview.png';handoff=BASE/'results/S17/showcase_repair_ackermann_20261113_181900/client/manual_handoff.json'
    if screenshot.exists() and handoff.exists():
        archived=read(handoff)
        if not (archived['requires_manual'] and archived['stop_confirmed']):raise ValueError('ARCHIVED_HANDOFF_NOT_CONFIRMED')
        path=a.out/'manual_handoff_record.mp4';caption="scale=960:650:force_original_aspect_ratio=decrease,pad=960:650:(ow-iw)/2:(oh-ih)/2,drawtext=fontfile=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc:text='人工检查记录 · 真实试运行截图（非实时画面）':fontcolor=white:fontsize=20:box=1:boxcolor=black@0.9:x=12:y=h-32"
        command([ffmpeg,'-hide_banner','-y','-loop','1','-framerate','25','-i',str(screenshot),'-t','6','-an','-vf',caption,'-c:v','libx264','-preset','fast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],a.out/'manual_handoff_record.log')
        clips.append(dict(key='manual_handoff_record',path=path,source_run=str(handoff.parent.parent.relative_to(ROOT)),source_screenshot_sha256=sha(screenshot),source_handoff_sha256=sha(handoff),kind='labeled_archived_still',duration_s=6))
    short_list=a.out/'short_concat.txt';short_list.write_text(''.join("file '"+str(p).replace("'","'\\''")+"'\n" for p in [fast]+[c['path'] for c in clips]));command([ffmpeg,'-hide_banner','-y','-f','concat','-safe','0','-i',str(short_list),'-c','copy','-movflags','+faststart',str(short)],a.out/'short_concat.log')
    dump(a.out/'manifest.json',dict(scope='INDEPENDENT_SHOWCASE_ONLY',formal_s17_credit=False,episodes=evidence,full=dict(path=str(full.relative_to(ROOT)),sha256=sha(full),nominal_wall_duration_s=total,speed=1),short=dict(path=str(short.relative_to(ROOT)),sha256=sha(short),overview_speed=speed,nominal_duration_s=total/speed+sum(c['duration_s'] for c in clips),replays=[c|dict(path=str(c['path'].relative_to(ROOT))) for c in clips]),no_generated_scene_motion=True,frame_holds_for_wall_pacing_disclosed=True))
    # The episode originals remain the immutable recording evidence. The
    # intermediate concat is byte-for-byte duplicate content, not new evidence.
    intermediate=a.out/'full_source_concat.mp4'
    if intermediate.exists():intermediate.unlink()
    print(full,short,flush=True)
if __name__=='__main__':main()
