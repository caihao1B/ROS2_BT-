"""Offline interactive review of measured cells, tracks and clearing; no CDN."""
from common09 import *
import html,itertools

def render(out,d):
 out=Path(out);payload={}
 for name,c in d['cases'].items():
  p=ROOT/c['path'];plot=read(p/'plot_data.json');images=sorted((p/'fusion').glob('camera_*.jpg'));plot['images']=[dict(t=int(x.stem.split('_')[-1]),path=os.path.relpath(x,out)) for x in images];plot['nav2']={}
  for chassis in ['diff','ackermann']:
   series=[]
   for file in sorted((p/'online').glob('nav2_'+chassis+'_*.npz'))[::3]:
    a=np.load(file);v=a['cost'];series.append(dict(t=float(a['t']),origin=a['origin'].tolist(),resolution=float(a['resolution']),width=v.shape[1],rle=[[int(k),sum(1 for _ in group)] for k,group in itertools.groupby(v.reshape(-1).tolist())]))
   plot['nav2'][chassis]=series
  plot['metrics']=c['metrics'];plot['path']=os.path.relpath(p,out);payload[name]=plot
 table=''.join('<tr><td>'+html.escape(name)+'</td><td>'+str(c['metrics']['passed'])+'</td><td>'+html.escape(', '.join(k for k,v in c['metrics']['checks'].items() if not v) or '全部通过')+'</td></tr>' for name,c in d['cases'].items())
 text='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>S09 三维感知融合与二维通行代价</title><style>
 body{font:16px system-ui;background:#eef2f5;color:#172b3a;margin:24px;max-width:1400px}h1{font-size:26px}section{background:white;padding:18px;border-radius:10px;margin:16px 0}table{border-collapse:collapse;width:100%}td,th{padding:8px;border-bottom:1px solid #ddd;text-align:left}button,select{padding:8px}input{width:50%}canvas{background:#17212c;max-width:100%;border-radius:6px}img{max-width:100%;border-radius:6px}.pair{display:grid;grid-template-columns:1fr 1fr;gap:16px}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px}a{color:#006eb8}.note{color:#624c0d;background:#fff4cb;padding:12px}
 </style><h1>S09 · 三维观测 → 地形风险 / 目标状态 → Nav2 二维代价</h1>
 <p>RUNID · 自动状态 STATUS · 用户人工状态 NOT_CHECKED</p><p class="note">真实 Gazebo 观测回放。车辆驻停进行感知验收；这里的色图与 Nav2 代价输出不代表已完成闭环绕障。灰色未观测区域保持未知。轨迹 ID 是短时关联假设。</p>
 <section><table><tr><th>案例</th><th>通过</th><th>失败项</th></tr>TABLE</table><p><a href="result.json">总指标/命令</a> · <a href="../../../../docs/optimization_v2/S09/MANUAL_CHECK.md">人工操作说明</a></p></section>
 <section><select id="case"></select> <select id="profile"><option value="8">差速</option><option value="9">阿克曼</option></select> <select id="layer"><option value="terrain">观测地形风险</option><option value="nav2">实际 Nav2 代价</option></select> <button id="play">播放</button> <input id="time" type="range" min="0" value="0" step="1"> <span id="clock"></span><p>绿色：观测地面；黄色：坡面代价；红色：不可通行；空白：未知。目标框标注实际轨迹 ID 和类别。坐标为 odom：X 向右、Y 向上，每个观测格 0.15 m；白块为驻停起点附近。</p>
 <div class="pair"><canvas id="map" width="640" height="520"></canvas><div><img id="camera" alt="同案例实际相机检测帧"><p id="caption"></p></div></div><pre id="sample"></pre></section>
 <section><h2>移除 8 cm 障碍后的原始地形格数</h2><canvas id="clear" width="1100" height="200"></canvas><p>红：占用；绿：重新观察为空闲；蓝：未知。膨胀层仍可因邻近真实障碍保持较高代价。</p></section>
 <section><h2>本案例实测指标</h2><pre id="metrics"></pre><a id="raw">本案例原始指标</a></section>
 <script>const data=DATA;const by=id=>document.getElementById(id);let timer=null;for(const k of Object.keys(data)){const o=document.createElement('option');o.value=k;o.text=k;by('case').add(o)}
 const nearest=(a,t)=>a.reduce((best,x)=>Math.abs(x.t-t)<Math.abs(best.t-t)?x:best,a[0]);
 function draw(){let d=data[by('case').value];if(!d)return;let rows=d.terrain;if(!rows.length)return;by('time').max=rows.length-1;let r=rows[Math.min(+by('time').value,rows.length-1)],t=r.t;by('clock').textContent=t.toFixed(1)+' 仿真秒';let ctx=by('map').getContext('2d');ctx.fillStyle='#17212c';ctx.fillRect(0,0,640,520);let X=x=>100+x*38,Y=y=>260-y*38;
 if(by('layer').value==='terrain'){for(const a of r.cells){let cost=a[+by('profile').value];ctx.fillStyle=cost>=100?'#fa635c':cost<0?'#475766':cost>0?'#efbd45':'#53b89b';ctx.fillRect(X(a[0])-3,Y(a[1])-3,6,6)}}else{let g=nearest(d.nav2[by('profile').value==='8'?'diff':'ackermann'],t);if(g){let i=0;for(const [cost,count] of g.rle){ctx.fillStyle=cost>=100?'#fa635c':cost<0?'#17212c':cost>0?'#efbd45':'#53b89b';for(let n=0;n<count;n++,i++){if(cost<0)continue;let x=g.origin[0]+(i%g.width+.5)*g.resolution,y=g.origin[1]+(Math.floor(i/g.width)+.5)*g.resolution;ctx.fillRect(X(x)-3,Y(y)-3,6,6)}}}}
 ctx.fillStyle='#fff';ctx.fillRect(X(0)-8,Y(0)-10,16,20);ctx.fillText('base / odom',X(0)+10,Y(0));let tracks=nearest(d.objects,t);if(tracks){for(const o of tracks.objects){let x=X(o.position[0]),y=Y(o.position[1]);ctx.strokeStyle='#6ebcff';ctx.strokeRect(x-o.size[0]*19,y-o.size[1]*19,o.size[0]*38,o.size[1]*38);ctx.fillStyle='#fff';ctx.fillText(o.id+' '+o.classification,x+5,y-8)}by('sample').textContent=JSON.stringify({t,ground_plane:r.ground_plane,terrain_ms:r.compute_ms,camera_valid:tracks.camera_valid,stereo:tracks.stereo_available,detector_ms:tracks.detector_ms,tracked:tracks.objects.length},null,2)}
 if(d.images.length){let im=nearest(d.images,t);by('camera').src=im.path;by('caption').textContent='真实检测图 '+im.t+' s（最邻近留存帧；不代表逐帧视频）'}
 by('metrics').textContent=JSON.stringify(d.metrics,null,2);by('raw').href=d.path+'/metrics.json';let c=by('clear').getContext('2d');c.clearRect(0,0,1100,200);c.fillStyle='#17212c';c.fillRect(0,0,1100,200);const a=d.clearing,ma=Math.max(1,...a.map(x=>x.total));for(const [key,color] of [['lethal','#fa635c'],['free','#53b89b'],['unknown','#6ebcff']]){c.beginPath();c.strokeStyle=color;a.forEach((p,i)=>{let x=20+1060*p.t/Math.max(1,a.at(-1).t),y=180-150*p[key]/ma;i?c.lineTo(x,y):c.moveTo(x,y)});c.stroke()}c.fillStyle='#fff';c.fillText('0 s → '+(a.at(-1)?.t||0).toFixed(1)+' s | 数值上限 '+ma,20,18);document.body.dataset.rendered='true'}
 by('case').onchange=()=>{by('time').value=0;draw()};by('profile').onchange=draw;by('layer').onchange=draw;by('time').oninput=draw;by('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;by('play').textContent='播放'}else{by('play').textContent='暂停';timer=setInterval(()=>{by('time').value=(+by('time').value+1)%(+by('time').max+1);draw()},350)}};draw();</script></html>'''
 text=text.replace('RUNID',html.escape(d['run_id'])).replace('STATUS',html.escape(d['status'])).replace('TABLE',table).replace('DATA',json.dumps(payload,ensure_ascii=False).replace('</','<\\/'));(out/'index.html').write_text(text)
