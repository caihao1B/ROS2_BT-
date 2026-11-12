"""Offline report: real ROS observations, injected errors, resources and RViz capture."""
import html,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from common import *
from analyze import rows
def render(out,data):
    out=Path(out);payload={};figures=[];table=[]
    for name,m in data['cases'].items():
        case=out/name
        if not (case/'online/observations.jsonl').exists():continue
        rs=rows(case/'online/observations.jsonl');gen=[]
        for p in [case/'generation/generation.jsonl',case/'vision/generation.jsonl']:
            if p.exists():gen+=rows(p)
        imu=[r for r in rs if r['key']=='imu'];wheel=[r for r in rs if r['key']=='wheel'];gnss=[r for r in rs if r['key']=='gnss'];truth=[r for r in gen if r['sensor']=='gnss' and r['event']=='generated']
        fig,ax=plt.subplots(2,2,figsize=(12,7))
        ax[0,0].plot([r['t'] for r in imu],[r['gyro'][2] for r in imu],label='observed gyro z');ax[0,0].set_ylabel('rad/s')
        ax[0,1].plot([r['t'] for r in wheel],[r['v'] for r in wheel],label='observed wheel speed');clean=[r for r in gen if r['sensor']=='wheel' and r['event']=='generated'];ax[0,1].plot([r['t'] for r in clean],[r['shaft_v'] for r in clean],alpha=.6,label='shaft reference (evaluation only)');ax[0,1].set_ylabel('m/s')
        if gnss:ax[1,0].plot([r['enu'][0] for r in gnss],[r['enu'][1] for r in gnss],'.-',label='observed GNSS')
        if truth:ax[1,0].plot([r['raw_enu'][0] for r in truth],[r['raw_enu'][1] for r in truth],label='simulator reference (evaluation only)')
        ax[1,0].set_xlabel('ENU east / m');ax[1,0].set_ylabel('ENU north / m');ax[1,0].axis('equal')
        for key in ['imu','left','points']:
            part=[r for r in rs if r['key']==key];ax[1,1].plot([r['t'] for r in part],[r['arrival_sim']-r['t'] for r in part],label=key,alpha=.7)
        ax[1,1].set_ylabel('arrival age / s')
        for a in [ax[0,0],ax[0,1],ax[1,1]]:a.set_xlabel('simulation time / s')
        for a in ax.flat:a.grid(alpha=.25);a.legend(fontsize=8)
        fig.suptitle(name+' | real recorded observations');fig.tight_layout();fig.savefig(case/'curves.svg');plt.close(fig)
        clouds=[]
        for f in sorted((case/'online').glob('cloud_*.npz')):
            d=np.load(f);xyz=d['xyz'];clouds.append(dict(t=float(d['t']),xyz=np.round(xyz[::max(1,len(xyz)//2500)],3).tolist()))
        payload[name]=dict(clouds=clouds,gnss=[[r['t'],*r['enu']] for r in gnss],faults=m.get('faults',{}))
        if clouds:
            xyz=np.array(clouds[0]['xyz']);fig=plt.figure(figsize=(9,5));ax3=fig.add_subplot(projection='3d');sc=ax3.scatter(xyz[:,0],xyz[:,1],xyz[:,2],c=xyz[:,2],s=2,cmap='viridis');ax3.set(xlabel='sensor x / m',ylabel='sensor y / m',zlabel='sensor z / m');ax3.set_xlim(-20,20);ax3.set_ylim(-20,20);ax3.set_zlim(-1.5,4);fig.colorbar(sc,ax=ax3,label='height / m');fig.tight_layout();fig.savefig(case/'cloud.svg');plt.close(fig)
        figs=f'<h3>{name} · {"PASS" if m.get("passed") else "FAIL"}</h3><img src="{name}/curves.svg">'
        if clouds:figs+=f'<img src="{name}/cloud.svg">'
        for f in ['left_0.png','right_0.png']:
            if (case/'online'/f).exists():figs+=f'<img class="half" src="{name}/online/{f}" alt="actual stereo image">'
        if (case/'rviz.png').exists():figs+=f'<details><summary>实际 RViz 窗口</summary><img src="{name}/rviz.png"></details>'
        figs+=f'<details><summary>实测指标与失败原因</summary><pre>{html.escape(json.dumps(m,ensure_ascii=False,indent=2))}</pre></details>'
        figures.append(figs)
        for key,v in m.get('rates',{}).items():table.append(f'<tr><td>{name}</td><td>{key}</td><td>{v["target_hz"]}</td><td>{v["sim_hz"]:.2f}</td><td>{100*v["missing_fraction"]:.2f}%</td><td>{1000*v["age_p95_s"]:.1f}</td></tr>')
    body='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>S04 多传感器验收</title><style>body{font:16px/1.7 system-ui;background:#eef3f6;color:#183b4c;margin:0}main{max-width:1180px;margin:28px auto;padding:0 22px}section{background:white;border-radius:12px;padding:24px;margin:20px 0}h1{font-size:29px}h2{font-size:22px}img,canvas{max-width:100%}.half{width:49%}table{width:100%;border-collapse:collapse;font-size:14px}td,th{padding:7px;border-bottom:1px solid #d8e4e9}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px}button,select{padding:8px}canvas{background:#182b38}input{width:45%}</style><main><h1>第 4 步 · 多传感器模拟与真实误差注入</h1><p>STATUS · RUN</p><p>NEXT · 人工目视：NOT_CHECKED</p><section><h2>原始观测回放</h2><p>以下是实际采集的三维返回点，颜色表示高度。旋转只改变查看角度，不改变点数据。GNSS 故障的实际坐标与基准对比见下方曲线。</p><select id="case"></select> 时间片 <input id="time" type="range" min="0" value="0"><span id="stamp"></span><br>旋转 <input id="rotation" type="range" min="-180" max="180" value="35"><canvas id="cloud" width="1100" height="480"></canvas></section><section><h2>验收与频率</h2><ul>CHECKS</ul><p>频率按采集仿真时钟统计；真实时间性能由 RTF 单独记录。故障案例的频率表只使用故障前基线，故障效果另行检验。missing 为实收频率与配置频率的缺口估计，不能分辨所有 DDS 与源端损失。</p><table><tr><th>案例</th><th>流</th><th>目标 Hz</th><th>实收 Hz</th><th>频率缺口</th><th>年龄 P95 ms</th></tr>TABLE</table><a href="result.json">全部指标与真实命令结果</a></section><section><h2>实际图像、误差与时序</h2>FIGURES</section><section><h2>资格边界</h2><p>本步没有实现定位融合、SLAM 或语义避障。激光为瞬时扫描，没有逐点时间，不能宣称已做去畸变。双目为理想标定的全局快门相机；GNSS fixed/float 是误差模型，不模拟载波解算。轮滑是轮式里程计等效失配，不等同轮胎物理降摩擦。消息时间戳保留采集时间；延迟恢复后被新数据超越的旧包会丢弃。</p><p>原始仿真输入在 ROS 121，评估真值在 120，在线观测在 119。通过实际图审计与显式桥接白名单隔离；这不是操作系统权限沙箱。噪声协方差不覆盖所有未建模系统误差，错误 fixed 不会被标签自动揭穿。</p></section></main><script>
const data=PAYLOAD,sel=document.getElementById('case'),slider=document.getElementById('time'),rot=document.getElementById('rotation'),canvas=document.getElementById('cloud'),ctx=canvas.getContext('2d');for(let key of Object.keys(data)){let o=document.createElement('option');o.value=o.textContent=key;sel.appendChild(o)}function draw(){let d=data[sel.value];if(!d||!d.clouds.length)return;slider.max=d.clouds.length-1;let f=d.clouds[Math.min(+slider.value,d.clouds.length-1)],a=+rot.value*Math.PI/180;ctx.clearRect(0,0,1100,480);for(let p of f.xyz){let x=p[0]*Math.cos(a)-p[1]*Math.sin(a),y=p[0]*Math.sin(a)+p[1]*Math.cos(a);ctx.fillStyle='hsl('+Math.max(0,Math.min(250,180-80*p[2]))+',75%,58%)';ctx.fillRect(550+x*15,280-y*7-p[2]*45,2.5,2.5)}document.getElementById('stamp').textContent='t='+f.t.toFixed(2)+' s'}sel.onchange=()=>{slider.value=0;draw()};slider.oninput=draw;rot.oninput=draw;draw();</script></html>'''
    for k,v in {'STATUS':data['status'],'RUN':data['run_id'],'NEXT':html.escape(data['next_step_verdict']),'CHECKS':''.join(f'<li>{k}：{v}</li>' for k,v in data['checks'].items()),'TABLE':''.join(table),'FIGURES':''.join(figures),'PAYLOAD':json.dumps(payload,separators=(',',':')).replace('</',r'<\/')}.items():body=body.replace(k,v)
    (out/'index.html').write_text(body)
