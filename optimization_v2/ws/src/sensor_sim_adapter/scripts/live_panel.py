"""Local read-only dashboard. Serves only noisy online observations and two images."""
import argparse,json,signal,threading
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from common import Path
PAGE='''<!doctype html><meta charset="utf-8"><title>S04 实时多传感器面板</title><style>body{font:15px system-ui;background:#eef3f6;color:#15394b;margin:24px}main{max-width:1320px;margin:auto}.grid{display:grid;grid-template-columns:1fr 1fr;gap:15px}section{background:white;padding:14px;border-radius:10px}img{width:100%}canvas{width:100%;height:220px}h1{font-size:25px}h2{font-size:17px;margin:0 0 8px}#status{font-weight:bold;padding:12px;background:#dcebe7;margin-bottom:15px}</style><main><h1>S04 · 实时观测（在线数据，无世界真值）</h1><div id="status">等待数据</div><div id="rates" style="padding:10px 0">实收频率：等待下一次采样</div><div class="grid"><section><h2>左目 · 640×360</h2><img id="left"></section><section><h2>右目 · 640×360</h2><img id="right"></section><section><h2>3D 点云投影 · 按高度着色，米</h2><canvas id="cloud" width="620" height="220"></canvas></section><section><h2>GNSS ENU 轨迹 · 米</h2><canvas id="gnss" width="620" height="220"></canvas></section><section><h2>轮速 · m/s</h2><canvas id="wheel" width="620" height="220"></canvas></section><section><h2>IMU 角速度 z · rad/s</h2><canvas id="imu" width="620" height="220"></canvas></section></div><p>另一个 RViz 窗口提供可旋转的原始三维点云与外参坐标树。fixed 不保证位置正确；GNSS 跳变或断流应观察诊断变化。当前运行结束后，请用 stepctl open S04 查看正式离线报告。</p></main><script>
function plot(id,points,color='#167e9b'){const c=document.getElementById(id),ctx=c.getContext('2d');ctx.clearRect(0,0,620,220);if(!points.length)return;let xs=points.map(p=>p[0]),ys=points.map(p=>p[1]),xmin=Math.min(...xs),xmax=Math.max(...xs),ymin=Math.min(...ys),ymax=Math.max(...ys);let dx=Math.max(xmax-xmin,.01),dy=Math.max(ymax-ymin,.01);ctx.strokeStyle=color;ctx.beginPath();points.forEach((p,i)=>{let x=25+570*(p[0]-xmin)/dx,y=195-170*(p[1]-ymin)/dy;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();ctx.fillStyle='#456';ctx.fillText(xmin.toFixed(2)+' → '+xmax.toFixed(2),30,214);ctx.fillText(ymin.toFixed(3)+' → '+ymax.toFixed(3),30,14)}
let previous=null;async function update(){try{let d=await (await fetch('live.json?t='+Date.now())).json();document.getElementById('status').textContent='仿真时间 '+d.t.toFixed(2)+' s · '+(d.diagnostics.join(' / ')||'观测持续更新');for(let s of ['left','right'])document.getElementById(s).src=s+'_latest.png?t='+Date.now();if(previous&&d.t>previous.t){let names={points:'激光',left:'左目',right:'右目',imu:'IMU',wheel:'轮速',gnss:'GNSS'};document.getElementById('rates').textContent='实收频率（最近更新区间，Hz）：'+Object.entries(names).map(([k,v])=>v+' '+((d.counts[k]-(previous.counts[k]||0))/(d.t-previous.t)).toFixed(1)).join(' · ')}previous=d;let r=d.recent;plot('wheel',r.filter(x=>x.key==='wheel').map(x=>[x.t,x.v]));plot('imu',r.filter(x=>x.key==='imu').map(x=>[x.t,x.gyro[2]]));plot('gnss',r.filter(x=>x.key==='gnss').map(x=>x.enu.slice(0,2)));let c=document.getElementById('cloud'),ctx=c.getContext('2d');ctx.clearRect(0,0,620,220);for(let p of d.cloud||[]){let x=310+7*(p[0]-p[1]),y=160-3*(p[0]+p[1])-30*p[2];ctx.fillStyle='hsl('+Math.max(0,Math.min(240,160-70*p[2]))+',70%,45%)';ctx.fillRect(x,y,2,2)}}catch(e){document.getElementById('status').textContent='等待观测，或本次试验已结束。'}}setInterval(update,1000);update();</script>'''
def main():
    p=argparse.ArgumentParser();p.add_argument('--online',type=Path,required=True);a=p.parse_args()
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            name=self.path.split('?')[0].strip('/')
            if name=='':data=PAGE.encode();mime='text/html; charset=utf-8'
            elif name in ['live.json','left_latest.png','right_latest.png'] and (a.online/name).exists():data=(a.online/name).read_bytes();mime='application/json' if name.endswith('json') else 'image/png'
            else:self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
        def log_message(self,*a):pass
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler);(a.online/'live_url.txt').write_text('http://127.0.0.1:'+str(server.server_port)+'/');stop=[False]
    def end(s,f):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);server.timeout=.2
    try:
        while not stop[0]:server.handle_request()
    finally:server.server_close()
if __name__=='__main__':main()
