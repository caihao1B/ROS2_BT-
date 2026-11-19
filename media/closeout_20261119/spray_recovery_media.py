"""Archive independent point-spray recovery runs and export measured, event-timed clips."""
import argparse
from bisect import bisect_left
import gzip
import json
from pathlib import Path
import shutil
import tempfile
import imageio_ffmpeg
from spray_defer_media import FF, FONT, MEDIA, ROOT, ff, rows, sha, write_json

PUBLIC = ROOT / "validation/closeout_20261119"
NAMES = {"natural": ("spray_natural", "spray_natural_relocalization"),
         "last": ("spray_last_blocked", "spray_last_blocked_handoff"),
         "initial": ("spray_last_blocked_initial", None)}


def pack(case, run, out, movie):
    receipt_name = "candidate_handoff_acceptance.json"
    receipt = json.loads((run/receipt_name).read_bytes())
    if receipt["formal_credit"] or not (run/"runtime.json").is_file():
        raise ValueError("The experiment must be closed with its own recorded acceptance")
    if (case == "natural") != (json.loads((run/"scene/fixture.json").read_bytes())["physical_fixture"] is None):
        raise ValueError("Natural and physically blocked cases must be archived independently")
    out.mkdir(parents=True, exist_ok=True)
    for name in [receipt_name,"spec.json","mission.json","scene/fixture.json","mission_result.json","mission_events.jsonl",
                 "runtime.json","nav_shutdown.json","analysis_spray_r10.json","video/recording.json","tool_evaluation/result.json",
                 "decisions.jsonl","video/frames.jsonl","operator_handoff.json","operator_attention_receipt.json","operator_attention_wire.jsonl"]:
        if not (run/name).is_file() and name in ("operator_handoff.json","operator_attention_receipt.json","operator_attention_wire.jsonl"):
            continue
        raw=(run/name).read_bytes()
        if name in receipt["source_sha256"] and sha(raw)!=receipt["source_sha256"][name]:
            raise ValueError("Original receipt mismatch: "+name)
        dst=out/name
        if name in ("decisions.jsonl","video/frames.jsonl","operator_attention_wire.jsonl"):
            dst=dst.with_suffix(".jsonl.gz");raw=gzip.compress(raw,compresslevel=9,mtime=0)
        dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(raw)
    original=run/"video/full.mp4"
    if sha(original.read_bytes())!=receipt["source_sha256"]["video/full.mp4"]:raise ValueError("Camera hash differs")
    shutil.copyfile(original,movie)


def captions(case, events):
    if case=="natural":
        p="point_10_20"
        typed=[(e["t"],e["event"]) for e in events if e.get("point")==p and e["event"]=="LOCAL_DOCK_GOAL"]
        def ev(name, condition=lambda e:True):
            return next(e["t"] for e in events if e["event"]==name and condition(e))
        entries=[(typed[0][0],"第一次局部精停修正"),(typed[1][0],"再次发送局部精停目标"),
                 (ev("STOPPED_RELOCALIZATION_REQUEST",lambda e:e.get("point")==p),"停车并请求重新定位"),
                 (ev("STOPPED_PLAN_REVALIDATION",lambda e:e.get("point")==p and e["path_valid"] and e["footprint_valid"]),"原路径和车辆轮廓重新验证通过"),
                 (ev("LOCALIZATION_REVALIDATED_RESUME",lambda e:e.get("point")==p),"重定位确认，恢复同一目标"),
                 (ev("DOCKING_SETTLED_CHECK",lambda e:e.get("point")==p and e.get("stable") and e["t"]>typed[1][0]),"静止观测满足精停门限"),
                 (ev("TOOL_RESULT",lambda e:e.get("point")==p and e.get("state")=="SUCCEEDED" and not e.get("replayed")),"工具反馈确认 10_20 点的作业量")]
        if [t for t,_ in entries]!=sorted(t for t,_ in entries):raise ValueError("Unexpected navigation recovery sequence")
        return [(entries[0][0]-2,"进入原受阻点 10_20 的精停观测"),*entries],entries[-1][0]+2.5,"自然场景 · 原故障点重定位",2.
    hold=next(e for e in events if e["event"]=="HUMAN_ASSISTANCE_REQUESTED")
    dest=hold["point"]
    tries=[e for e in events if e.get("point")==dest and e["event"]=="NAV_GOAL" and e["t"]<=hold["t"]]
    if not tries:raise ValueError("Missing blocked-point navigation request")
    exit_info=[e for e in events if e["event"]=="FINAL_TOOL_STOP" and e.get("state")=="STOPPED" and e["t"]>=hold["t"]]
    if not exit_info:raise ValueError("Missing measured tool-stop confirmation after handoff")
    if not any(e["event"]=="HUMAN_HANDOFF_CLOSED" and e.get("verified") for e in events):
        raise ValueError("Missing verified manual handoff closure")
    entries=[(e["t"],f"第 {i+1} 次导航到末点 {dest.removeprefix('point_')}") for i,e in enumerate(tries)]
    entries.extend([(hold["t"],"无法安全完成末点，记录残余并请求人工核查"),
                    (exit_info[-1]["t"],"人工请求已发出，工具停机与交接关闭均已确认")])
    return entries,exit_info[-1]["t"]+3.3,"末点受阻 · 停车转人工",2.


def export(case, run):
    prefix,clipname=NAMES[case]; archive=PUBLIC/prefix
    full=MEDIA/(prefix+"_full.mp4");clip=MEDIA/(clipname+".mp4") if clipname else None
    if run:pack(case,run,archive,full)
    receipt=json.loads((archive/"candidate_handoff_acceptance.json").read_bytes())
    spec=json.loads((archive/"spec.json").read_bytes()); rec=json.loads((archive/"video/recording.json").read_bytes())
    mission=json.loads((archive/"mission_result.json").read_bytes())
    if sha(full.read_bytes())!=receipt["source_sha256"]["video/full.mp4"]:raise ValueError("Source video differs from receipt")
    fr=rows(gzip.decompress((archive/"video/frames.jsonl.gz").read_bytes()))
    ts=[r["sim_t"] for r in fr]
    if len(fr)!=rec["frames"] or ts!=sorted(ts) or not all(r["frame"]==i and abs(r["video_s"]-i/rec["fps"])<.001 for i,r in enumerate(fr)):
        raise ValueError("Source camera timeline differs")
    events=rows((archive/"mission_events.jsonl").read_bytes())
    if case=="initial":
        if receipt.get("passed") is not False or mission.get("operator_handoff"):
            raise ValueError("The initial watchdog archive is reserved for its observed failure")
        frames,duration=imageio_ffmpeg.count_frames_and_secs(str(full))
        if frames!=len(fr) or abs(duration-rec["duration_s"])>.25:raise ValueError("Failed-attempt camera timeline differs")
        attention=json.loads((archive/"operator_attention_receipt.json").read_bytes())
        decisions=rows(gzip.decompress((archive/"decisions.jsonl.gz").read_bytes()))
        human=[d for d in decisions if d["action"]=="HUMAN" and d["reason_code"]=="BOUNDED_REPLAN_EXHAUSTED" and d["manual_latched"]]
        report=dict(native_human_latched=dict(first_sim_t=human[0]["t"],dispatch_id=human[0]["dispatch_id"],observations=len(human)) if human else None,
                    schema="R10-SPRAY-INITIAL-WATCHDOG/1",case=case,session=spec["session"],formal_credit=False,mission_passed=False,
                    handoff_demonstrated=False,expected_behavior_passed=False,observed_notification=attention,closed_sim_s=mission["t"],
                    error=mission.get("error"),original_count=receipt["original_count"],selected=receipt["selected"],confirmed=receipt["completed"],
                    runtime_passed=json.loads((archive/"runtime.json").read_bytes())["passed"],
                    original=dict(path=str(full.relative_to(ROOT)),sha256=sha(full.read_bytes()),seconds=duration,frames=frames,
                                  fps=rec["fps"],unmodified=True),
                    archived_source={str(p.relative_to(archive)):sha(p.read_bytes()) for p in sorted(archive.rglob("*")) if p.is_file()})
        write_json(PUBLIC/(prefix+"_media.json"),report)
        print(json.dumps(report["original"]),mission.get("error"))
        return
    markers,stop,title,speed=captions(case,events)
    def frame(sim):
        return fr[min(len(fr)-1,bisect_left(ts,sim))]
    start_pl=frame(markers[0][0])["video_s"]; stop_pl=frame(stop)["video_s"]
    if stop_pl<=frame(markers[-1][0])["video_s"]:
        stop_pl=min(rec["duration_s"],frame(markers[-1][0])["video_s"]+1/rec["fps"])
    cues=[dict(event_sim_s=t,original_video_s=frame(t)["video_s"],excerpt_s=round((frame(t)["video_s"]-start_pl)/speed,3),text=label)
          for t,label in markers]
    with tempfile.TemporaryDirectory(prefix="spray-recovery-film-") as tmp:
        tmp=Path(tmp);head=tmp/"header.txt";head.write_text(f"独立试验  |  {title}  |  连续原片 {speed:g} 倍速")
        f=(f"setpts=(PTS-STARTPTS)/{speed},fps=10,pad=iw:ih+84:0:0:color=0x101c25,"
           f"drawtext=fontfile={FONT}:textfile={head}:fontsize=17:fontcolor=0xA4E7CA:x=16:y=h-70")
        for i,cue in enumerate(cues):
            text=tmp/f"c{i}.txt";text.write_text(cue["text"])
            until=cues[i+1]["excerpt_s"] if i+1<len(cues) else (stop_pl-start_pl)/speed+.5
            f+=f",drawtext=fontfile={FONT}:textfile={text}:fontsize=22:fontcolor=white:x=16:y=h-43:enable='gte(t,{cue['excerpt_s']})*lt(t,{until})'"
        ff(["-ss",start_pl,"-t",round(stop_pl-start_pl,4),"-i",full,"-vf",f,"-map","0:v:0","-an",
            "-c:v","libx264","-threads","1","-preset","veryfast","-crf","22","-pix_fmt","yuv420p","-movflags","+faststart",clip])
    n,duration=imageio_ffmpeg.count_frames_and_secs(str(clip));nf,sf=imageio_ffmpeg.count_frames_and_secs(str(full))
    if nf!=rec["frames"] or abs(sf-rec["duration_s"])>.25 or abs(n/10-(stop_pl-start_pl)/speed)>.2:
        raise ValueError("Full or excerpt decoder timeline differs")
    report=dict(schema="R10-EXPERIMENTAL-SPRAY-FILM/1",case=case,session=spec["session"],formal_credit=False,
                closed_sim_s=mission["t"],original_count=receipt.get("original",receipt.get("original_count")),selected=receipt["selected"],confirmed=receipt["completed"],
                mission_passed=receipt["mission_passed"],handoff_status=receipt.get("handoff_status",receipt.get("terminal")),
                experiment_behavior_passed=receipt.get("passed"),
                outstanding_selected=mission.get("failed",[]),runtime_passed=json.loads((archive/"runtime.json").read_bytes())["passed"],
                scene_task_scope=title,actual_jobs=receipt["actual_jobs"],checked_stop=mission.get("final_gate"),
                original=dict(path=str(full.relative_to(ROOT)),sha256=sha(full.read_bytes()),seconds=sf,frames=nf,fps=rec["fps"],unmodified=True),
                excerpt=dict(path=str(clip.relative_to(ROOT)),sha256=sha(clip.read_bytes()),seconds=duration,frames=n,fps=10,
                             source_clip_video_s=[start_pl,stop_pl],speed_relative_to_original_video=speed,decoded_to_eof=True,cues=cues),
                archive_files={str(p.relative_to(archive)):sha(p.read_bytes()) for p in sorted(archive.rglob('*')) if p.is_file()},
                clock_mapping="First recording frame at or beyond each event sim_s. Excerpt times use measured video_s and playback speed.")
    write_json(PUBLIC/(prefix+"_media.json"),report)
    print(json.dumps(dict(session=report["session"],case=case,selected=receipt["selected"],completed=receipt["completed"],original=report["original"],
                          excerpt={k:report["excerpt"][k] for k in ("path","sha256","seconds")},cues=cues),ensure_ascii=False))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case",choices=NAMES)
    parser.add_argument("--run",type=Path,help="Export closed original run to the release, otherwise rebuild from its packaged evidence.")
    args=parser.parse_args()
    export(args.case,args.run)
