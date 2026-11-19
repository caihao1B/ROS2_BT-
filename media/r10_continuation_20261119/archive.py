"""Keep the actual Gazebo movie and a short continuous, time-labelled replay of a closed R10 driving experiment."""
import argparse
from bisect import bisect_left
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
MEDIA = Path(__file__).resolve().parent
PROOF = ROOT / 'validation/r10_continuation_20261119'


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def camera(run):
    info = read(run / 'video/recording.json')
    if not info.get('valid') or info.get('encoder_exit') != 0 or info.get('source') != 'actual Gazebo camera':
        raise ValueError('This session does not have a qualified real-camera recorder.')

    frames = read_events(run / 'video/frames.jsonl')
    if len(frames) != info['frames'] or len(frames) < 2:
        raise ValueError('The recorder frame count differs from the frame log.')
    for i, frame in enumerate(frames):
        if frame['frame'] != i or abs(frame['video_s'] - i / info['fps']) > .001:
            raise ValueError('The frame log and encoded video clock differ.')
        if not math.isfinite(frame['sim_t']) or (i and frames[i-1]['sim_t'] > frame['sim_t']):
            raise ValueError('The recorded simulation clock moved backwards or is invalid.')
    return info, frames


def read_events(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def capture(run, tag, title, watch_events, first_t=None, last_t=None):
    import imageio_ffmpeg

    if not re.fullmatch(r'[a-z][a-z0-9_]+', tag):
        raise ValueError('Use a short ASCII output tag.')
    run = run.resolve()
    spec = read(run / 'spec.json')
    task = read(run / 'mission_result.json')
    runtime = read(run / 'runtime.json')
    if spec.get('session') != run.name or spec.get('task') not in ('spray', 'mowing'):
        raise ValueError('The run and selected experiment differ.')
    if not isinstance(task.get('t'), (int, float)):
        raise ValueError('Experiment has no terminal simulation time.')

    info, frames = camera(run)
    timestamps = [f['sim_t'] for f in frames]
    watch = set(watch_events)
    selected = [
        event for event in read_events(run / 'mission_events.jsonl')
        if event.get('event') in watch and isinstance(event.get('t'), (int, float))
    ]
    if not selected:
        raise ValueError('Requested events were not observed.')
    first = selected[0]['t'] if first_t is None else first_t
    last = selected[-1]['t'] if last_t is None else last_t
    if first > last or first < timestamps[0] or last > timestamps[-1] + .5:
        raise ValueError('Requested clip is outside the recorded camera clock.')

    def index_at(t):
        return min(bisect_left(timestamps, t), len(timestamps) - 1)

    fps = info['fps']
    src = run / 'video/full.mp4'
    a = max(0, index_at(first) - round(fps * 3))
    b = min(len(frames), index_at(last) + round(fps * 4) + 1)
    if b <= a:
        raise ValueError('Empty source clip.')

    # Keep one continuous excerpt at a fixed speed so camera cues remain comparable with the raw file.
    duration = (b - a) / fps
    speed = max(1, math.ceil(duration / 78))
    out_fps = 10
    expected = max(1, round(duration / speed * out_fps))
    raw = MEDIA / (tag + '_full.mp4')
    short = MEDIA / (tag + '_short.mp4')
    full_hash = sha(src)
    if raw.exists() and sha(raw) != full_hash:
        raise ValueError('This output name already refers to another recording.')
    if not raw.exists():
        shutil.copyfile(src, raw)
    font = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    with tempfile.TemporaryDirectory(prefix='r10-film-') as tmp:
        tmp = Path(tmp)
        caption = tmp / 'caption.txt'
        caption.write_text(f'{title}  |  原始录像 {speed} 倍速  |  现场连续节选')
        vf=(f'trim=start_frame={a}:end_frame={b},setpts=(PTS-STARTPTS)/{speed},'
            f'fps={out_fps},tpad=stop_mode=clone:stop_duration=1,trim=end_frame={expected},'
            f'pad=iw:ih+52:0:52:color=0x142a30,drawtext=fontfile={font}:textfile={caption}:'
            "fontcolor=white:fontsize=21:x=12:y=10")
        staged=tmp/'excerpt.mp4'
        cmd=[ffmpeg,'-loglevel','error','-nostdin','-threads','1','-filter_threads','1','-i',str(src),'-vf',vf,
             '-an','-c:v','libx264','-threads','1','-preset','veryfast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(staged)]
        subprocess.run(cmd,check=True)
        decoded=imageio_ffmpeg.count_frames_and_secs(str(staged))
        if decoded[0]!=expected or abs(decoded[1]-expected/out_fps)>.15:
            raise ValueError('The excerpt failed frame-count or full-decoding verification.')
        if short.exists() and sha(short) != sha(staged):
            raise ValueError('This tag already identifies a different excerpt. Choose a new tag.')
        if not short.exists():
            shutil.copyfile(staged, short)
    full_decoded=imageio_ffmpeg.count_frames_and_secs(str(raw))
    if full_decoded[0]!=info['frames'] or abs(full_decoded[1]-info['frames']/fps)>.15:
        raise ValueError('The unmodified H.264 recording failed full-decoding verification.')
    cues=[]
    for event in selected:
        i=index_at(event['t'])
        cues.append(dict(event=event['event'],point=event.get('point'),segment=event.get('segment'),
                         direction=event.get('direction'),simulation_s=event['t'],raw_video_s=i/fps,
                         clip_s=max(0,round((i-a)/fps/speed,3)) if a<=i<b else None))
    archive = PROOF / 'films' / tag
    archive.mkdir(exist_ok=True, parents=True)
    files = {}
    for name in ['spec.json','mission_result.json','mission_events.jsonl','runtime.json','video/recording.json','video/frames.jsonl']:
        src_file = run / name
        dest = archive / name
        dest.parent.mkdir(exist_ok=True, parents=True)
        if name.endswith('.jsonl'):
            dest=dest.with_suffix('.jsonl.gz')
            dest.write_bytes(gzip.compress(src_file.read_bytes(),6,mtime=0))
        else:
            shutil.copyfile(src_file,dest)
        files[str(dest.relative_to(archive))]=dict(source=name,raw_sha256=sha(src_file),published_sha256=sha(dest))
    manifest=dict(schema='r10-continuation-camera/1',session=run.name,task=spec['task'],case=spec.get('case'),
                  formal_score_in_this_media_file=False,mission_passed=task.get('passed'),mission_terminal=task.get('terminal'),
                  mission_error=task.get('error'),closed_sim_s=task['t'],runtime_clean=runtime.get('passed'),
                  title=title,raw=dict(path=str(raw.relative_to(ROOT)),sha256=full_hash,frames=info['frames'],fps=fps,
                                      seconds=full_decoded[1],original_and_decoded=True),
                  clip=dict(path=str(short.relative_to(ROOT)),sha256=sha(short),frames=expected,fps=out_fps,
                            seconds=expected/out_fps,source_start_frame=a,source_end_frame_exclusive=b,
                            wall_camera_speed=speed,actual_continuous_frames=True,decoded_to_eof=True),
                  event_bookmarks=cues,source_files=files)
    write(archive/'media.json',manifest)
    return dict(session=run.name,raw=str(raw.relative_to(ROOT)),short=str(short.relative_to(ROOT)),seconds=expected/out_fps,events=len(cues))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--tag', required=True)
    parser.add_argument('--title', required=True)
    parser.add_argument('--event',action='append',required=True)
    parser.add_argument('--first-sim', type=float)
    parser.add_argument('--last-sim', type=float)
    args=parser.parse_args()
    print(json.dumps(capture(args.run,args.tag,args.title,args.event,args.first_sim,args.last_sim),ensure_ascii=False))
