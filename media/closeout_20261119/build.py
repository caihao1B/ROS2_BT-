"""Generate a scoped release from closed real-camera receipts, retaining full originals."""
from pathlib import Path
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import imageio_ffmpeg

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "optimization_v2/ws/src/task_platform/scripts"
sys.path.insert(0, str(SCRIPTS))
from highlights_r12 import select

MEDIA = ROOT / "media/closeout_20261119"
VALID = ROOT / "validation/closeout_20261119"
R12 = ROOT / "optimization_v2/three_tasks/results/R12"
FF = imageio_ffmpeg.get_ffmpeg_exe()
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

CASES = [
    ("spray", "spray_20261119_071627_581230", "analysis_spray_r10",
     "点喷", "前三个服务点 · 3/3 已完成 · A1 共 56 点", "工具卡滞后停止喷洒，再按反馈账本补作剩余剂量。"),
    ("inspection_leader", "inspection_20261114_161912_321381", "analysis_r08",
     "连接路巡检", "跟随专项 · 路线 20.00 m · 原连接路 88.48 m", "前车停车时跟车停稳，随后恢复前进。"),
    ("mowing_startup_perception_loss", "mowing_20261119_070346_235832", "analysis_mowing_r10",
     "阿克曼割草", "前五个执行段 · 5/5 已完成 · 安全版 B1 共 47 段", "感知中断触发取消。底盘和刀具停稳后，恢复原作业。"),
]

def read(p):
    return json.loads(p.read_text())

def rows(p):
    with p.open() as stream:
        return [json.loads(line) for line in stream]

def sha(path):
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

def run_ff(args):
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-nostdin", "-y",
                    "-filter_threads", "1", "-filter_complex_threads", "1", "-threads", "1", *args],
                   check=True, timeout=300)

def video_info(path):
    probe = subprocess.run([FF, "-hide_banner", "-nostdin", "-threads", "1", "-i", str(path),
                            "-map", "0:v:0", "-c", "copy", "-f", "null", "-"],
                           capture_output=True,text=True,timeout=60,check=True)
    hours,minutes,seconds = re.search(r"Duration: (\d+):(\d+):([\d.]+)",probe.stderr).groups()
    codec,width,height = re.search(r"Video: ([^ ,]+)[^\n]*? (\d+)x(\d+)[ ,]",probe.stderr).groups()
    if codec != "h264" or (int(width),int(height)) != (960,766):
        raise ValueError("Unexpected codec or resolution")
    return int(hours)*3600+int(minutes)*60+float(seconds),codec,[int(width),int(height)]


def verify(scope, folder, analysis_name, title, label, detail):
    p = R12 / folder
    receipt = read(p/"recording_acceptance.json")
    if not all(receipt.get(k) for k in ("passed", "task_passed", "video_valid", "mission_timeline_complete", "active_video_quality")):
        raise ValueError(f"Ineligible recording {folder}")
    analysis = read(p/(analysis_name+".json")); task = read(p/"mission_result.json"); rec = read(p/"video/recording.json")
    original = p/"video/full.mp4"
    if not (analysis["passed"] and task["passed"] and all(analysis["checks"].values()) and
            rec["valid"] and rec["encoder_exit"] == 0 and rec["no_generated_motion"] and rec["source"] == "actual Gazebo camera"
            and sha(original) == receipt["source_video_sha256"]):
        raise ValueError(f"Task, checks, encoder or hash invalid for {folder}")
    frames = rows(p/"video/frames.jsonl")
    events = rows(p/"mission_events.jsonl")
    start = next(x["t"] for x in rows(p/"control/gate.jsonl") if x["mode"]=="DRIVE" and abs(x["v"])+abs(x["w"])>.003)
    if scope=="mowing_startup_perception_loss":
        start = min(start,*(e["t"] for e in events if e["event"]=="START_WORK_REQUEST"))
    active = [f for f in frames if start<=f["sim_t"]<=task["t"]]
    maxage = max(f["image_wall_age_s"] for f in active)
    maxgap = max(b["sim_t"]-a["sim_t"] for a,b in zip(active,active[1:]))
    if (len(frames)!=rec["frames"] or not all(f["frame"]==i and abs(f["video_s"]-i/rec["fps"]) < .001 for i,f in enumerate(frames))
        or not all(b["sim_t"]>=a["sim_t"] for a,b in zip(frames,frames[1:]))
        or not (frames[0]["sim_t"]<=start and frames[-1]["sim_t"]>=task["t"]-.5)
        or len(active)<10 or maxage>2.5 or maxgap>1):
        raise ValueError(f"Timeline threshold failure for {folder}")
    excerpt = select(scope,events,frames,analysis)
    if not excerpt: raise ValueError(f"No verified behavior for {folder}")
    if scope=="spray":
        metrics=dict(selected=task["selected_points"],original=task["original_points"],completed=len(task["completed"]),
                     physical_dose_and_feedback_agree=analysis["checks"]["feedback_and_independent_physics_agree"],
                     actual_jam_and_residual_retry=analysis["checks"]["real_fault_and_residual_retry"],
                     repeated_request_no_extra_flow=analysis["checks"]["idempotent_replay_no_extra_flow"])
        public_full=MEDIA/"spray_pilot_full.mp4"
    elif scope=="inspection_leader":
        metrics=dict(selected_m=task["selected_length_m"],original_m=task["original_length_m"],
                     full_connector_selected=task["full_route_selected"],min_physical_gap_m=analysis["minimum_physical_follow_gap_m"],
                     stop_and_resume=analysis["leader_stop_assessment"])
        public_full=ROOT/"media/r12_inspection_leader_full.mp4"
    else:
        interrupted=next(e for e in events if e["event"]=="BT_INTERRUPTS_START_WORK")
        stopped=next(e for e in events if e["event"]=="BOTH_PHYSICAL_STOPS_CONFIRMED" and e["t"]>interrupted["t"])
        metrics=dict(selected=task["selected_segments"],completed=len(task["completed_segments"]),execution=task["execution_segments"],
                     old_reference=task["reference_original_segments"],confirmed_work_m2=analysis["selected_confirmed_union_m2"],
                     selected_target_m2=analysis["selected_requested_area_m2"],both_stops_after_cancel_sim_s=stopped["t"]-interrupted["t"])
        public_full=MEDIA/"mowing_pilot_full.mp4"
    if not public_full.exists():
        if scope=="inspection_leader":raise FileNotFoundError(public_full)
        shutil.copy2(original,public_full)
    if sha(public_full)!=receipt["source_video_sha256"]:raise ValueError(str(public_full))
    if scope=="spray":
        fault=next(e for e in events if e["event"]=="TOOL_RESULT" and e["reason"]=="TOOL_HEALTH")
        done=next(e for e in events if e["t"]>fault["t"] and e["event"]=="TOOL_RESULT"
                  and e["point"]==fault["point"] and e["state"]=="SUCCEEDED")
        actions=[(excerpt["start_sim"],"进入喷洒点，等待定位与停稳确认。"),
                 (fault["t"],f"卡滞后已计量 {fault['confirmed']:.3f}，待补作 {fault['remaining']:.3f} 剂量单位。"),
                 (done["t"],f"工具确认累计 {done['confirmed']:.3f} 剂量单位，继续前往下一点。")]
    elif scope=="inspection_leader":
        physical=analysis["leader_stop_assessment"]
        resume=next(e for e in events if e["event"]=="RESUME_REMAINING_ROUTE" and e["t"]>67.)
        actions=[(excerpt["start_sim"],f"最近实测车距 {metrics['min_physical_gap_m']:.2f} m。"),
                 (62.,"前车在仿真 62–67 秒内保持静止。"),
                 (64.,f"跟车在 64–67 秒内实测位移 {physical['ego_motion_64_67_m']:.2f} m。"),
                 (resume["t"],f"继续执行剩余巡检路线。72–85 秒跟车实测前进 {physical['ego_motion_72_85_m']:.2f} m。")]
    else:
        work=next(e for e in events if e["event"]=="TOOL_WORKING_CONFIRMED" and e["t"]>stopped["t"])
        actions=[(excerpt["start_sim"],"停稳后发出原条带的刀具启动请求。"),
                 (interrupted["t"],"行为树发出 CANCEL，等待实测停止。"),
                 (stopped["t"],f"收到车体与刀具停稳反馈，距取消 {metrics['both_stops_after_cancel_sim_s']:.3f} 仿真秒。"),
                 (work["t"],"工具已反馈 WORKING，重新执行原条带。")]
    annotation=[]
    for stamp,caption in actions:
        frame=next(f for f in frames if f["sim_t"]>=stamp)
        annotation.append(dict(evidence_sim_s=stamp,source_video_s=frame["video_s"],caption=caption))
    source_files=[p/"recording_acceptance.json",p/(analysis_name+".json"),p/"spec.json",p/"mission.json",
                  p/"mission_result.json",p/"mission_events.jsonl",p/"video/frames.jsonl",p/"video/recording.json",p/"control/gate.jsonl"]
    return dict(case=scope,run=folder,task=title,scope=label,description=detail,task_passed=True,
                task_terminal_sim_s=task["t"],physical_and_work_checks=metrics,
                full_recording=dict(path=str(public_full.relative_to(ROOT)),sha256=sha(public_full),wall_seconds=rec["duration_s"],
                    fps=rec["fps"],frames=len(frames),start_sim_s=frames[0]["sim_t"],end_sim_s=frames[-1]["sim_t"],
                    max_active_image_age_wall_s=maxage,max_active_frame_gap_sim_s=maxgap),
                source_evidence={str(f.relative_to(ROOT)):sha(f) for f in source_files},excerpt=excerpt,annotations=annotation)


def main():
    os.nice(12)
    MEDIA.mkdir(exist_ok=True,parents=True);VALID.mkdir(exist_ok=True,parents=True)
    episodes=[verify(*case) for case in CASES]
    with tempfile.TemporaryDirectory(prefix="r12_closeout_") as tempdir:
        temp=Path(tempdir)
        for e in episodes:
            code=e["case"].split("_")[0]; s=e["excerpt"]; speed=max(s["speed"],math.ceil(s["duration_video_s"]/32.))
            header=temp/(code+"_header.txt");header.write_text(f"{e['task']}   |   原片播放 {speed}×   |   真实 Gazebo 录像")
            caption=temp/(code+"_caption.txt");caption.write_text(e["scope"]+"\n"+e["description"])
            # Keep all three original telemetry lines except the superseded playback-rate header.
            filters=(f"setpts=(PTS-STARTPTS)/{speed},fps=10,pad=iw:ih+116:0:0:color=0x101c25,"
                "drawbox=x=0:y=0:w=iw:h=36:color=0x111b25:t=fill,"
                f"drawtext=fontfile={FONT}:textfile={header}:fontcolor=white:fontsize=20:x=14:y=5,"
                f"drawtext=fontfile={FONT}:textfile={caption}:fontcolor=white:fontsize=20:line_spacing=5:x=14:y=h-103")
            annotation=e["annotations"]
            for i,row in enumerate(annotation):
                seconds=(row["source_video_s"]-s["start_video_s"])/speed
                until=(annotation[i+1]["source_video_s"]-s["start_video_s"])/speed if i+1<len(annotation) else s["duration_video_s"]/speed
                row["excerpt_start_s"]=round(seconds,3)
                caption_file=temp/f"{code}_cue_{i}.txt";caption_file.write_text(row["caption"])
                filters+=(f",drawtext=fontfile={FONT}:textfile={caption_file}:fontcolor=0xA4E7CA:fontsize=20:x=14:y=h-38"
                          f":enable='gte(t,{seconds:.6f})*lt(t,{until:.6f})'")
            clip=temp/(code+"_excerpt.mp4")
            run_ff(["-ss",str(s["start_video_s"]),"-t",str(s["duration_video_s"]),"-i",str(ROOT/e["full_recording"]["path"]),
                    "-vf",filters,"-an","-map","0:v:0","-threads","1","-c:v","libx264","-preset","veryfast","-crf","24",
                    "-pix_fmt","yuv420p","-movflags","+faststart",str(clip)])
            dest=MEDIA/clip.name;shutil.copy2(clip,dest)
            s.update(speed=speed,path=str(dest.relative_to(ROOT)),sha256=sha(dest),duration_s=s["duration_video_s"]/speed)
        listing=temp/"list.txt";listing.write_text("".join(f"file '{ROOT/e['excerpt']['path']}'\n" for e in episodes))
        combined=temp/"three_tasks_highlights.mp4"
        run_ff(["-f","concat","-safe","0","-i",str(listing),"-map","0:v:0","-c","copy","-movflags","+faststart",str(combined)])
        run_ff(["-i",str(combined),"-map","0:v:0","-threads","1","-f","null","-"])
        target=MEDIA/combined.name;shutil.copy2(combined,target)
        intervals=[]
        for e in episodes:
            # Measure cut and combined durations directly from their encoded timebases.
            duration,_,_ = video_info(ROOT/e["excerpt"]["path"])
            start=intervals[-1]["end_s"] if intervals else 0.
            intervals.append(dict(case=e["case"],start_s=round(start,3),end_s=round(start+duration,3)))
        duration,codec,size=video_info(target)
        if abs(duration-intervals[-1]['end_s'])>.21:raise ValueError("Combined playback duration mismatch")
    manifest=dict(schema="R12-CLOSEOUT-MEDIA/1",date="2026-11-19",recording_scope="Two new bounded fault demonstrations plus prior measured leader interaction; no formal cluster/matrix credit.",
         formal_s17_s18="PARTIAL", combined=dict(path=str(target.relative_to(ROOT)),sha256=sha(target),
         seconds=duration,bytes=target.stat().st_size,video_size=size,
         codec=codec,decoder_completed=True,chapters=intervals), episodes=episodes)
    (VALID/"media_manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(manifest['combined'],ensure_ascii=False))


if __name__=="__main__":main()
