"""Export the isolated four-point spray run and render its clock-aligned real-camera highlight."""
import argparse
from bisect import bisect_left
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import imageio_ffmpeg

ROOT = Path(__file__).resolve().parents[2]
MEDIA = Path(__file__).resolve().parent
ARCHIVE = ROOT / "validation/closeout_20261119/spray_defer"
FULL = MEDIA / "spray_defer_full.mp4"
CLIP = MEDIA / "spray_defer_highlight.mp4"
MANIFEST = ARCHIVE.parent / "spray_defer_media.json"
FONT = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FF = imageio_ffmpeg.get_ffmpeg_exe()
SESSION = "spray_defer_20261119_085422_132494"
RANGES = [(34.5,44.,2,"首个服务点完成，开始接近被设备占用的点"),
          (54.,82.,4,"三次导航尝试，停车后把受阻点保留为待作业"),
          (82.,143.,8,"选择后续原始点 9_20，继续导航、精停与喷洒"),
          (163.,178.5,3,"到达 8_20 喷洒，最后确认车体停车、工具停机")]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read_json(name):
    return json.loads((ARCHIVE/name).read_bytes())


def rows(raw):
    return [json.loads(l) for l in raw.splitlines() if l]


def write_json(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+"\n")


def archive(run):
    acceptance=json.loads((run/"candidate_acceptance.json").read_bytes())
    if acceptance["session"]!=SESSION or acceptance["formal_credit"] or not acceptance["coarse_defer_demonstrated"] or acceptance["precision_defer_demonstrated"]:
        raise ValueError("Expected the archived coarse-navigation experimental case")
    ARCHIVE.mkdir(parents=True,exist_ok=True)
    for name in ["candidate_acceptance.json","spec.json","mission.json","scene/fixture.json","mission_result.json","mission_events.jsonl",
                 "runtime.json","nav_shutdown.json","analysis_spray_r10.json","video/recording.json","tool_evaluation/result.json",
                 "decisions.jsonl","video/frames.jsonl"]:
        raw=(run/name).read_bytes()
        if name in acceptance["evidence"] and sha(raw)!=acceptance["evidence"][name]:
            raise ValueError("Original receipt mismatch for "+name)
        dst=ARCHIVE/name
        if name in ("decisions.jsonl","video/frames.jsonl"):
            dst=dst.with_suffix(".jsonl.gz"); raw=gzip.compress(raw,compresslevel=9,mtime=0)
        dst.parent.mkdir(parents=True,exist_ok=True); dst.write_bytes(raw)
    source=run/"video/full.mp4"
    if sha(source.read_bytes())!=acceptance["evidence"]["video/full.mp4"]:
        raise ValueError("Full original camera SHA-256 differs from acceptance")
    shutil.copyfile(source,FULL)


def ff(args):
    subprocess.run([FF,"-loglevel","error","-nostdin","-y","-filter_threads","1","-threads","1",*map(str,args)],check=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",type=Path,help="Export a local original. Without this option reuse the distributed evidence and MP4.")
    args=parser.parse_args()
    if args.run:
        archive(args.run)
    acceptance=read_json("candidate_acceptance.json"); result=read_json("mission_result.json");rec=read_json("video/recording.json")
    decisions=rows(gzip.decompress((ARCHIVE/"decisions.jsonl.gz").read_bytes()))
    values=rows(gzip.decompress((ARCHIVE/"video/frames.jsonl.gz").read_bytes()))
    events=rows((ARCHIVE/"mission_events.jsonl").read_bytes())
    times=[r["sim_t"] for r in values]
    fps=rec["fps"]
    if len(values)!=rec["frames"] or times!=sorted(times) or not all(r["frame"]==i and abs(r["video_s"]-i/fps)<.001 for i,r in enumerate(values)):
        raise ValueError("Original frame time map does not match recording.json")
    if sha(FULL.read_bytes())!=acceptance["evidence"]["video/full.mp4"]:
        raise ValueError("Original camera hash mismatch")
    def video_at(sim):
        return values[bisect_left(times,sim)]["video_s"]
    key=acceptance["deferred"][0]["native_dispatch_id"]
    # The thin native dispatch excerpt traces the transition, with original source rows retained in the gzip.
    matches=[d for d in decisions if d["dispatch_id"]==key and d["action"]=="DEFER"]
    if not matches or matches[0]["reason_code"]!="TRY_OTHER_OBLIGATION_RETAIN_RESIDUAL":
        raise ValueError("Missing native dispatch")
    write_json(ARCHIVE/"native_defer_excerpt.json",dict(first=matches[0],last=matches[-1],matched_rows=len(matches),full_source_sha256=acceptance["evidence"]["decisions.jsonl"]))
    clips=[];elapsed=0.
    with tempfile.TemporaryDirectory(prefix="spray-defer-video-") as tmp:
        tmp=Path(tmp)
        for i,(a,b,speed,label) in enumerate(RANGES):
            va,vb=video_at(a),video_at(b); n=round((vb-va)/speed*10)
            header=tmp/f"head{i}.txt"; desc=tmp/f"desc{i}.txt"
            header.write_text(f"独立试验  ·  差速点喷  |  原片 {speed} 倍速  |  第 {i+1} / 4 段")
            desc.write_text(label)
            vf=(f"setpts=(PTS-STARTPTS)/{speed},fps=10,pad=iw:ih+84:0:0:color=0x101c25,"
                f"drawtext=fontfile={FONT}:textfile={header}:fontsize=17:fontcolor=0xA4E7CA:x=16:y=h-70,"
                f"drawtext=fontfile={FONT}:textfile={desc}:fontsize=23:fontcolor=white:x=16:y=h-43")
            clip=tmp/f"{i}.mp4"
            ff(["-ss",va,"-t",round(vb-va,4),"-i",FULL,"-map","0:v:0","-an","-vf",vf,"-frames:v",n,"-c:v","libx264",
                "-threads","1","-preset","veryfast","-crf","22","-pix_fmt","yuv420p","-movflags","+faststart",clip])
            # Use actual decoded frame count; short trims and fps quantization may round by one frame.
            actual=imageio_ffmpeg.count_frames_and_secs(str(clip))[0]
            clips.append(dict(title=label,speed_relative_to_original_video=speed,source_video_s=[va,vb],
                              source_sim_t_window=[a,b],source_frame_range_half_open=[bisect_left(times,a),bisect_left(times,b)],
                              highlight_s=[round(elapsed,1),round(elapsed+actual/10,1)],frames=actual,
                              events=[{"t":e["t"],"event":e["event"],"point":e.get("point")} for e in events if a<=e["t"]<b]))
            elapsed+=actual/10
        listing=tmp/"concat.txt";listing.write_text("".join(f"file '{i}.mp4'\n" for i in range(len(clips))))
        ff(["-f","concat","-safe","0","-i",listing,"-c","copy","-movflags","+faststart",CLIP])
    frames,sec=imageio_ffmpeg.count_frames_and_secs(str(CLIP))
    if frames!=sum(x["frames"] for x in clips) or abs(frames/10-sec)>.15 or not 35<=sec<=50:
        raise ValueError("Encoded highlight timeline differs")
    full_frames,full_s=imageio_ffmpeg.count_frames_and_secs(str(FULL))
    if full_frames!=len(values) or abs(full_s-rec["duration_s"])>.25:raise ValueError("Full video duration/number of frames differs")
    report=dict(schema="SPRAY-DEFER-CAMERA/1",session=SESSION,formal_credit=False,behavior_result=acceptance["behavior_result"],
                behavior="COARSE_NAVIGATION_DEFER",precision_defer_demonstrated=False,full_cluster_denominator=56,
                selected=acceptance["selected"],confirmed=acceptance["completed"],outstanding=acceptance["residuals"],
                physical_jobs=acceptance["physical_jobs"],native_dispatch_id=key,native_defer_sim_s=acceptance["deferred"][0]["sim_s"],
                terminal_sim_s=result["t"],original_camera=dict(path=str(FULL.relative_to(ROOT)),sha256=sha(FULL.read_bytes()),
                frames=full_frames,seconds=full_s,fps=fps,unmodified=True),highlight=dict(path=str(CLIP.relative_to(ROOT)),sha256=sha(CLIP.read_bytes()),
                seconds=sec,fps=10,frames=frames,decoded_to_eof=True,clips=clips),
                clock_mapping="First original frame with sim_t >= the declared boundary; speed factor uses source player time. End index excluded.",
                archived_source={str(p.relative_to(ARCHIVE)):sha(p.read_bytes()) for p in sorted(ARCHIVE.rglob('*')) if p.is_file()})
    write_json(MANIFEST,report)
    print(json.dumps({k:report[k] for k in ("session","behavior","original_camera")},ensure_ascii=False),f"highlight {sec:.1f}s {sha(CLIP.read_bytes())}")


if __name__=="__main__":main()
