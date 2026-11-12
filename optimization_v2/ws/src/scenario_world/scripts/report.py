"""Offline evaluation report; this file and its embedded private data are NOT online inputs."""
import json,html,shutil
from pathlib import Path
from validate_scene import read
def render(out,data,case):
    out=Path(out);case=Path(case);scene=case/'scene';payload={}
    if scene.exists():
        payload=dict(zones=read(scene/'maps/prior/workspaces.geojson'),road=read(scene/'maps/prior/connector.geojson'),permanent=read(scene/'maps/prior/permanent.geojson'),coarse=read(scene/'maps/prior/terrain_coarse.geojson'),truth=read(scene/'evaluation/truth_manifest.json'))
        payload['observed']=read(case/'online/observed_points.json') if (case/'online/observed_points.json').exists() else []
        payload['trajectory']=read(case/'evaluation/result.json')['trajectory'] if (case/'evaluation/result.json').exists() else []
    (out/'viewer_data.json').write_text(json.dumps(payload,ensure_ascii=False,separators=(',',':')))
    images=[]
    for p,label in [(case/'online/camera.png','实际 RGB 相机帧：传感器观测'),(case/'camera_evaluated.png','离线评估叠加：用真值投影检查可见性，不是视觉识别算法'),(out/'overview/gazebo.png','实际 Gazebo 俯视窗口'),(out/'ground/gazebo.png','实际 Gazebo 地面窗口'),(case/'gazebo.png','本次 Gazebo 窗口')]:
        if p.exists():images.append(f'<figure><img src="{p.relative_to(out)}"><figcaption>{label}</figcaption></figure>')
    checks=''.join(f'<li>{html.escape(k)}：<b>{html.escape(v)}</b></li>' for k,v in data.get('checks',{}).items())
    metrics=html.escape(json.dumps(data.get('metrics',{}),ensure_ascii=False,indent=2))
    template='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>S02 连续三维世界与先验分层</title>
<style>body{margin:0;background:#edf2f5;color:#203742;font:16px/1.7 system-ui,sans-serif}main{max-width:1200px;margin:32px auto;padding:0 24px}h1{font-size:30px;line-height:1.3}section{background:white;border-radius:12px;padding:24px;margin:20px 0}canvas{width:100%;background:#f3f1e7;border:1px solid #c9d3d5}label{display:inline-block;margin:6px 20px 6px 0}img{max-width:100%;height:auto}figure{margin:20px 0}figcaption,.note{color:#576e7b;font-size:14px}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px}button{padding:8px 15px;cursor:pointer}input[type=range]{width:60%}.status{color:#147b74;font-size:20px}.legend{padding:10px;background:#f2f7f8}</style>
<main><h1>第 2 步 · 连续不规则三维世界与先验分层</h1><p class="status">STATUS · RUN</p>
<p>两个完整工作区 → 后续算法分簇。三种地图变体；当前未生成簇，未验证真实底盘净空。</p>
<section><h2>可交互分层地图</h2><p class="note">这是离线评估页。真值层只在此页显示；在线进程读不到该页的数据流。蓝点是固定探针实际量测，空白表示未观测，不表示可通行。</p>
<label><input id="prior" type="checkbox" checked>规划先验</label><label><input id="observation" type="checkbox" checked>传感器观测（待发现物体的实际回波）</label><label><input id="truth" type="checkbox">仅评估真值</label>
<canvas id="map" width="1120" height="784"></canvas><div><button id="play">播放实测轨迹</button> <input id="time" type="range" min="0" max="1" value="0"><span id="stamp"></span></div>
<p class="legend">浅绿 A：喷洒 · 深绿 B：割草 · 米灰：连接路 · 灰：已知设施 · 虚线：粗地形 · 蓝点：三维回波投影 · 橙/红：仅评估的未知物体/运动位置。单位 m，局部 ENU；尚未绑定真实地理坐标。</p></section>
<section><h2>验收状态</h2><ul>CHECKS</ul><p>人工查看：NOT_CHECKED。NEXT</p><p>仅验证场景与数据隔离；30 分钟任务预算在 S12 评估，机器人、定位、导航与工具动作尚未在此场景验收。</p></section>
<section><h2>真实运行图像</h2>IMAGES</section><section><h2>实测数值</h2><pre>METRICS</pre><a href="result.json">结果 JSON</a> · <a href="viewer_data.json">离线评估层数据</a><p>完整原始点云快照、每个进程的命令/退出码、轨迹与接触记录保存在对应案例目录。</p></section></main>
<script>const D=PAYLOAD;const c=document.getElementById('map'),x=c.getContext('2d'),s=7;const on=id=>document.getElementById(id).checked;function xy(p){return[p[0]*s,(112-p[1])*s]}function polygon(g,color,stroke){x.beginPath();for(const ring of g.coordinates){ring.forEach((p,i)=>{let q=xy(p);i?x.lineTo(...q):x.moveTo(...q)});x.closePath()}x.fillStyle=color;x.fill('evenodd');x.strokeStyle=stroke||color;x.stroke()}
function draw(){x.clearRect(0,0,c.width,c.height);x.strokeStyle='#dae0d9';x.lineWidth=1;for(let k=0;k<=160;k+=10){x.beginPath();x.moveTo(k*s,0);x.lineTo(k*s,784);x.stroke()}for(let k=0;k<=112;k+=10){x.beginPath();x.moveTo(0,k*s);x.lineTo(1120,k*s);x.stroke()}if(!D.zones)return;
if(on('prior')){D.zones.features.forEach((f,i)=>polygon(f.geometry,i?'#74b68e':'#c2d48c','#467c64'));polygon(D.road.features[0].geometry,'#c8bea5','#ad9f7b');D.permanent.features.forEach(f=>polygon(f.geometry,'#788993'));x.setLineDash([5,4]);D.coarse.features.forEach(f=>polygon(f.geometry,'#c5b38255','#a0894c'));x.setLineDash([]);x.fillStyle='#193f3c';x.font='bold 17px sans-serif';D.zones.features.forEach((f,i)=>{let q=xy(i?[126,50]:[32,43]);x.fillText(f.properties.id+' · '+f.properties.area_m2.toFixed(1)+' m²',...q)})}
if(on('observation')){x.fillStyle='#087eae';for(const p of D.observed){const q=xy(p);x.fillRect(q[0]-1,q[1]-1,2,2)}}
let r=D.trajectory[+document.getElementById('time').value];if(on('truth')){for(const o of D.truth.low_obstacles){let q=xy([o.center[0]-o.size[0]/2,o.center[1]+o.size[1]/2]);x.fillStyle='#e97025';x.fillRect(...q,o.size[0]*s,o.size[1]*s)}D.truth.terrain.filter(a=>a.prior==='unknown').forEach(a=>{let q=xy([a.origin[0],a.origin[1]+a.width]);x.strokeStyle='#d95535';x.strokeRect(...q,a.length*s,a.width*s)});if(r){for(const [id,p] of Object.entries(r.poses)){let q=xy(p);x.fillStyle='#d3374c';x.beginPath();x.arc(...q,4,0,Math.PI*2);x.fill();x.font='11px sans-serif';x.fillText(id,q[0]+5,q[1]-5)}}}document.getElementById('stamp').textContent=r?' t = '+r.t.toFixed(2)+' s（实测）':'无实测轨迹'}
document.getElementById('time').max=Math.max(0,(D.trajectory||[]).length-1);for(const id of ['prior','observation','truth','time'])document.getElementById(id).addEventListener('input',draw);let timer=null;document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;return}timer=setInterval(()=>{let t=document.getElementById('time');t.value=(+t.value+1)%(+t.max+1);draw()},100)};draw();</script></html>'''
    replacements={'STATUS':data['status'],'RUN':html.escape(data['run_id']),'CHECKS':checks,'NEXT':html.escape(data.get('next_step_verdict','演示不改变验收状态')),'IMAGES':''.join(images),'METRICS':metrics,'PAYLOAD':json.dumps(payload,ensure_ascii=False,separators=(',',':')).replace('</',r'<\/')}
    for k,v in replacements.items():template=template.replace(k,v)
    (out/'index.html').write_text(template)
