"""Offline measured plots and interactive trajectory replay, never online inputs."""
import html,json,math
from pathlib import Path
import numpy as np
from analyze import rows,read,physical
def render(out,data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out=Path(out);datasets={};pictures=[]
    for case in sorted(out.iterdir()):
        if not case.is_dir() or not (case/'metrics.json').exists():continue
        m=read(case/'metrics.json')
        if not (case/'evaluation/truth.jsonl').exists():continue
        truth=physical(rows(case/'evaluation/truth.jsonl'));online=rows(case/'online/online.jsonl');c=read(case/'scene/vehicle.json')
        if not truth:continue
        route=read(case/'online/target_path.json') if (case/'online/target_path.json').exists() else []
        datasets[case.name]=dict(points=[[r['t'],*r['xyz'][:2],r['yaw']] for r in truth[::5]],target=route,footprint=c['footprint'],metrics=m)
        if m.get('case') in ['connector','slope','bench']:
            fig,axs=plt.subplots(2,2,figsize=(12,7));ts=[r['t'] for r in truth];xy=np.array([r['xyz'][:2] for r in truth]);axs[0,0].plot(xy[:,0],xy[:,1],label='physical trajectory')
            if route:ar=np.array(route);axs[0,0].plot(ar[:,0],ar[:,1],'--',label='reference')
            if online:
                od=[r for r in online if r['odom']];ix,iy,ih=c['initial_pose'];ox=np.array([r['odom'][:2] for r in od]);co,si=math.cos(ih),math.sin(ih);gx=ix+co*ox[:,0]-si*ox[:,1];gy=iy+si*ox[:,0]+co*ox[:,1];axs[0,0].plot(gx,gy,':',label='encoder odometry')
                tt=[r['t'] for r in online];axs[0,1].plot(tt,[r['v'] for r in online],label='requested v');axs[0,1].plot(ts,[r['v'] for r in truth],label='measured v')
                er=[r for r in online if r['encoders'] and r['encoders']['valid']];et=[r['t'] for r in er];axs[1,0].plot(et,[r['encoders']['velocity'][0] for r in er],label='left wheel');axs[1,0].plot(et,[r['encoders']['velocity'][1] for r in er],label='right wheel')
                if c['id']=='ackermann':
                    axs[1,1].plot(et,[r['encoders']['q'][2] for r in er],label='left steering');axs[1,1].plot(et,[r['encoders']['q'][3] for r in er],label='right steering');axs[1,1].set_ylabel('steering / rad')
                else:axs[1,1].plot(ts,[r['w'] for r in truth],label='physical yaw rate');axs[1,1].set_ylabel('yaw rate / rad/s')
            axs[0,0].set_aspect('equal');axs[0,0].set_xlabel('world x / m');axs[0,0].set_ylabel('world y / m');axs[0,1].set_ylabel('speed / m/s');axs[1,0].set_ylabel('wheel speed / rad/s')
            for ax in axs.flat:ax.grid(alpha=.25);ax.legend(fontsize=8)
            for ax in [axs[0,1],axs[1,0],axs[1,1]]:ax.set_xlabel('simulation seconds')
            fig.suptitle(case.name+' | measured, no truth feedback to controller');fig.tight_layout();fig.savefig(case/'curves.svg');plt.close(fig)
            pictures.append(f'<figure><img src="{case.name}/curves.svg"><figcaption>{case.name}：实际轨迹、编码器里程计、轮速/转角；曲线按仿真时间对齐。</figcaption></figure>')
        if (case/'gazebo.png').exists():pictures.append(f'<figure><img src="{case.name}/gazebo.png"><figcaption>{case.name} · 实际 Gazebo 窗口</figcaption></figure>')
        if 'fault' in m.get('stops',{}):
            event=read(case/'online/result.json')['events'][0]
            reset=1 if m.get('case')=='reset' else 0
            origin=0 if reset else event['t']
            samples=[r for r in truth if r['resets']==reset and r['t']>=origin-1]
            fig,axes=plt.subplots(1,2,figsize=(10,2.8))
            for ax,key,threshold in zip(axes,['v','w'],[.01,.02]):
                ax.plot([r['t']-origin for r in samples],[abs(r[key]) for r in samples],label='physical '+key)
                ax.axhline(threshold,color='#bc4358',linestyle='--',label='stop threshold')
                ax.axvline(0,color='gray',linestyle=':');ax.set_xlabel('simulation seconds after event/reset');ax.grid(alpha=.25);ax.legend()
            fig.suptitle(case.name);fig.tight_layout();fig.savefig(case/'stop_curves.svg');plt.close(fig)
            pictures.append(f'<details><summary>{case.name}：实际停车速度曲线</summary><img src="{case.name}/stop_curves.svg"></details>')
    # Stop curves explicitly show physical speed after command/fault, not commanded zero.
    stops=[]
    for key,entry in datasets.items():
        for name,s in entry['metrics'].get('stops',{}).items():stops.append((key+'/'+name,s))
    if stops:
        fig,ax=plt.subplots(figsize=(12,max(4,len(stops)*.19)));labels=[n for n,_ in stops];values=[d.get('distance_m',0) for _,d in stops];ax.barh(labels,values,color=['#238c77' if s['passed'] else '#bf4c4c' for _,s in stops]);ax.set_xlabel('Measured travel after stop/fault / m');ax.grid(axis='x',alpha=.3);fig.tight_layout();fig.savefig(out/'stopping_distances.svg');plt.close(fig);pictures.append('<figure><img src="stopping_distances.svg"><figcaption>每种停止工况的实际位移；红色表示未通过。</figcaption></figure>')
    payload=json.dumps(datasets,ensure_ascii=False,separators=(',',':')).replace('</',r'<\/');checks=''.join(f'<li>{k}：<b>{v}</b></li>' for k,v in data['checks'].items());table=''.join(f'<tr><td>{html.escape(k)}</td><td>{"PASS" if m.get("passed") else "FAIL"}</td><td>{html.escape(str({x:y for x,y in m.get("checks",{}).items() if not y}))}</td></tr>' for k,m in data.get('cases',{}).items())
    body='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>S03 两种轻量底盘验收</title><style>body{font:16px/1.7 system-ui;background:#edf2f5;color:#17384a;margin:0}main{max-width:1180px;margin:32px auto;padding:0 24px}section{background:white;padding:24px;border-radius:12px;margin:20px 0}h1{font-size:30px}img,canvas{max-width:100%}canvas{width:100%;background:#f4f3eb}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px}table{width:100%;border-collapse:collapse}td{padding:7px;border-bottom:1px solid #dde5e8}select,button{padding:8px}input[type=range]{width:60%}.note{color:#5c7380;font-size:14px}figure{margin:24px 0}</style><main>
<h1>第 3 步 · 差速与阿克曼轻量底盘</h1><p>STATUS · RUN</p><p>NEXT · 人工查看：NOT_CHECKED</p>
<section><h2>真实轨迹回放</h2><p class="note">蓝色为物理实际轨迹，虚线为公开目标路线。红色车框含工具包络。真值仅用于此离线评分页面；路线控制使用编码器与临时前向测距。</p><select id="case"></select> <button id="play">播放</button><input id="time" type="range" value="0" min="0"><span id="stamp"></span><canvas id="map" width="1100" height="540"></canvas></section>
<section><h2>检查与资格范围</h2><ul>CHECKS</ul><p>最高速度只在已通过的模型/平地工况内成立；坡面资格仅 5°、0.4 m/s。模型使用 Gazebo 关节速度执行器，非实车制动或电机温升认证。工具为几何接口，喷洒/割草动作留 S10。当前已知起点和编码器跟踪不代表 SLAM 已完成。</p><details><summary>展开已固化的参数与工具包络</summary><pre>QUALIFIED</pre></details><table><tr><td>实际案例</td><td>状态</td><td>未通过检查</td></tr>TABLE</table><a href="result.json">完整实测 JSON</a> · <a href="commands.jsonl">构建/回归命令</a></section>
<section><h2>实际图像与时间曲线</h2>IMAGES</section><section><h2>停止与失败边界</h2><p>实际停车阈值保持 v&lt;0.01 m/s、|ω|&lt;0.02 rad/s，停止延迟&lt;2 s；命令 0.4 s、反馈 0.5 s、底层 0.35 s watchdog。取消、暂停/复位或故障后需要明确重新 ARM，禁止自动恢复旧速度。</p><p>开发试验否定了未经校准直接使用名义轮距的方案：已分开记录物理轮距与有效驱动轮距。S03 的实际爬坡还发现并修复 S02 坡面面绕序，S02 重新验收后才继续本步。</p></section></main>
<script>const data=PAYLOAD,sel=document.getElementById('case'),slider=document.getElementById('time'),canvas=document.getElementById('map'),ctx=canvas.getContext('2d');for(const k of Object.keys(data)){let o=document.createElement('option');o.textContent=o.value=k;sel.appendChild(o)}let timer=null;function draw(){const d=data[sel.value];if(!d)return;slider.max=d.points.length-1;const xy=d.points.map(p=>p.slice(1,3)).concat(d.target);let xmin=Math.min(...xy.map(p=>p[0]))-3,xmax=Math.max(...xy.map(p=>p[0]))+3,ymin=Math.min(...xy.map(p=>p[1]))-3,ymax=Math.max(...xy.map(p=>p[1]))+3;let scale=Math.min(1040/(xmax-xmin),480/(ymax-ymin));const project=p=>[30+(p[0]-xmin)*scale,510-(p[1]-ymin)*scale];ctx.clearRect(0,0,1100,540);function line(points,color,dash=[]){if(!points.length)return;ctx.beginPath();points.forEach((p,i)=>{const q=project(p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([])}line(d.target,'#b69252',[6,5]);line(d.points.map(p=>p.slice(1,3)),'#2786a5');const p=d.points[Math.min(+slider.value,d.points.length-1)];const foot=d.footprint.map(q=>[p[1]+Math.cos(p[3])*q[0]-Math.sin(p[3])*q[1],p[2]+Math.sin(p[3])*q[0]+Math.cos(p[3])*q[1]]);line([...foot,foot[0]],'#c54655');document.getElementById('stamp').textContent='t='+p[0].toFixed(2)+' s'}sel.onchange=()=>{slider.value=0;draw()};slider.oninput=draw;document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null}else timer=setInterval(()=>{slider.value=(+slider.value+1)%(+slider.max+1);draw()},60)};sel.value=data.diff_connector?'diff_connector':Object.keys(data)[0];draw();</script></html>'''
    for k,v in {'STATUS':data['status'],'RUN':data['run_id'],'NEXT':html.escape(data['next_step_verdict']),'CHECKS':checks,'QUALIFIED':html.escape(json.dumps(data.get('qualified_profiles',{}),ensure_ascii=False,indent=2)),'TABLE':table,'IMAGES':''.join(pictures),'PAYLOAD':payload}.items():body=body.replace(k,v)
    body=body.replace('最高速度只在已通过的模型/平地工况内成立；','S03 隔离底盘测试将动态对象停放到试验区外，保留原静态障碍、坡面与道路；无碰撞通行不代表动态避障通过。最高速度只在已通过的模型/平地工况内成立；')
    (out/'index.html').write_text(body)
