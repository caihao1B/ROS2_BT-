"""Build the download-and-open event index from the measured recorder time mappings."""
from hashlib import file_digest
import json
import gzip
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
PROOF=ROOT/'validation/closeout_20261119'
def read(name):return json.loads((PROOF/name).read_text(encoding='utf-8'))

def add(rows,title,path,sha,description,events,session=None):
    with (HERE/path).open('rb') as stream:
        if file_digest(stream,'sha256').hexdigest()!=sha:
            raise ValueError('Video bytes differ from receipt: '+path)
    rows.append(dict(title=title,path=path,summary=description,events=events,session=session))

def build():
    recordings=read('media_manifest.json')
    current=read('media_full_runs.json')
    rows=[]
    short=recordings['combined']
    add(rows,'三任务精简展示',Path(short['path']).name,short['sha256'],'78 秒，三种作业的实测录像。',
        [[0,None,'点喷工具故障与补量'],[22.3,None,'巡检跟随与停车'],[52.8,None,'割草感知中断与刀具恢复']])
    for run in sorted(current['runs'],key=lambda r:r['case'],reverse=True):
        add(rows,('A1 点喷' if run['case'].startswith('spray') else 'B1 割草')+' · 完整选择',
            Path(run['published_path']).name,run['published_sha256'],run['summary_zh'],
            [[e['video_s'],e['sim_s'],e['label_zh']] for e in run['video_bookmarks']],run['session'])
    clips={'spray':'spray_pilot_full.mp4','inspection_leader':'../r12_inspection_leader_full.mp4','mowing_startup_perception_loss':'mowing_pilot_full.mp4'}
    for run in recordings['episodes']:
        path=clips[run['case']]
        add(rows,run['task']+' · 专项原片',path,run['full_recording']['sha256'],run['scope'],
            [[e['source_video_s'],e['evidence_sim_s'],e['caption']] for e in run['annotations']],Path(run['run']).name)
    defer = read('spray_defer_media.json')
    scope = '四个原始点，完成三个。一个导航受阻点停车留账，随后服务下一原始点。'
    for key, label, timing in [('highlight', '重选实录 · 47 秒', 'highlight_s'),
                               ('original_camera', '重选实录 · 完整原片', 'source_video_s')]:
        asset = defer[key]
        chapters = [[clip[timing][0], clip['source_sim_t_window'][0], clip['title']]
                    for clip in defer['highlight']['clips']]
        add(rows, label, Path(asset['path']).name, asset['sha256'], scope, chapters, defer['session'])
    examples = [('spray_natural_media.json','同点重定位 · 连续节选','自然四点 · 完整原片',
                 '自然四点实测。原故障点 10_20 重新定位后完成喷洒，9_20 工具取消后剩余 2.410。'),
                ('spray_last_blocked_media.json','末点转人工 · 连续节选','末点转人工 · 完整原片',
                 '末点 10_20 被设备占用。两次导航失败后行为树锁存人工请求，独立订阅器收到报警和停车确认。')]
    for source, excerpt_title, original_title, scope in examples:
        proof = read(source)
        for key, label, timing in [('excerpt',excerpt_title,'excerpt_s'),('original',original_title,'original_video_s')]:
            asset = proof[key]
            chapters = [[cue[timing],cue['event_sim_s'],cue['text']] for cue in proof['excerpt']['cues']]
            add(rows,label,Path(asset['path']).name,asset['sha256'],scope,chapters,proof['session'])
    initial = read('spray_last_blocked_initial_media.json')
    asset = initial['original']
    with gzip.open(PROOF/'spray_last_blocked_initial/video/frames.jsonl.gz','rt',encoding='utf-8') as stream:
        frames = [json.loads(line)['sim_t'] for line in stream]
    observed = [(initial['native_human_latched']['first_sim_t'],'原生行为树锁存人工处理'),
                (initial['closed_sim_s'],'上层等待超时后停机，现场未收到通知')]
    marks = [[next((i for i,t in enumerate(frames) if t>=sim),len(frames)-1)/asset['fps'],sim,label]
             for sim,label in observed]
    add(rows,'末点受阻 · 修复前记录',Path(asset['path']).name,asset['sha256'],
        '选取两个原点。原生行为树已请求人工，执行器等待超时。独立订阅器收到 0 条通知。',marks,initial['session'])
    data=json.dumps(rows,ensure_ascii=False).replace('<','\\u003c')
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>三任务实录 · 事件选片</title><style>
:root{color-scheme:dark}body{margin:auto;max-width:1150px;padding:20px clamp(12px,3vw,40px);font:16px/1.6 system-ui,"Microsoft YaHei",sans-serif;background:#101a23;color:#e8eff1}h1{font-size:26px;margin:0}h2{font-size:16px}p{color:#b6c8cd}select,button{font:inherit;background:#233543;border:1px solid #445e68;border-radius:6px;color:inherit;padding:6px 10px}video{background:#060a0d;width:100%;max-height:72vh;border-radius:8px}.layout{display:grid;grid-template-columns:2fr 1fr;gap:18px}button{display:block;text-align:left;width:100%;margin:.4em 0;cursor:pointer}button:hover,button:focus-visible{border-color:#65d5bd}button small{display:block;color:#a7d2c8}a{color:#7fdac5}@media(max-width:760px){.layout{display:block}}label{display:block;margin:14px 0 5px}</style>
<h1>三任务仿真实录</h1><label for="choice">选择影片</label><select id="choice"></select><p id="scope"></p>
<div class="layout"><div><video id="film" controls preload="metadata" playsinline></video><p id="status" role="status"></p></div>
<div><h2>跳到实际事件</h2><div id="marks"></div><a id="evidence" href="../../validation/closeout_20261119/replay.html">在观测记录中查看 BT、定位与控制</a></div></div>
<script type="application/json" id="data">DATA</script><script>
const rows=JSON.parse(document.getElementById('data').textContent),choice=document.getElementById('choice'),film=document.getElementById('film'),marks=document.getElementById('marks'),statusText=document.getElementById('status');
const fmt=t=>Math.floor(t/60)+':'+String(Math.floor(t%60)).padStart(2,'0');
for(const r of rows){const o=document.createElement('option');o.textContent=r.title;choice.add(o)}
function evidenceLink(session,sim){
 const a=document.getElementById('evidence'),base='../../validation/closeout_20261119/replay.html';
 a.href=session?base+'#'+new URLSearchParams({run:session,...(sim==null?{}:{t:sim})}):base;
 a.textContent=sim==null?'在观测记录中查看 BT、定位与控制':'查看这次事件的 BT、定位与控制'
}
function show(){const r=rows[choice.selectedIndex];evidenceLink(r.session,null);film.pause();film.src=r.path;document.getElementById('scope').textContent=r.summary;marks.replaceChildren();statusText.textContent='';
 for(const [clock,sim,label] of r.events){const b=document.createElement('button'),hint=document.createElement('small');b.textContent=label;hint.textContent='影片 '+fmt(clock)+(sim===null?'':' · 仿真 '+sim.toFixed(3)+' 秒');b.append(hint);b.onclick=()=>{const jump=()=>{film.currentTime=clock;evidenceLink(r.session,sim);statusText.textContent=label+' · '+hint.textContent};if(film.readyState>=1)jump();else film.addEventListener('loadedmetadata',jump,{once:true})};marks.append(b)}}
film.addEventListener('error',()=>{statusText.textContent='无法载入本地影片。请先解压整个项目并保留 media 目录结构，再打开此页。'});
choice.onchange=show;show();</script></html>'''
    (HERE/'index.html').write_text(page.replace('>DATA<','>'+data+'<'),encoding='utf-8')

if __name__=='__main__':
    build()
