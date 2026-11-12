'use strict';
// All numbers in the interactive labs are synthetic teaching inputs.
window.learningErrors = [];
window.addEventListener('error', e => window.learningErrors.push(e.message));
const el = id => document.getElementById(id);
const write = (id, value) => { el(id).textContent = value; };
const modes = {
  local: {nodes:[['轮速 + IMU + 激光','观测、时间与质量审核'],['普通 15 状态 EKF','估计车体速度与倾斜姿态'],['连续局部积分','odom → base_link']],note:'相机与 GNSS 没有进入 S05 EKF。公开 odom 由滤波后的速度积分得到。'},
  mapping: {nodes:[['双目 + 激光 + S05 odom','视觉地点识别与几何约束'],['RTAB-Map + GTSAM','三维位姿图与回环优化'],['三维地图 / 数据库','S07 冻结后用于重定位']],note:'相机生成地点候选；几何验证决定是否添加约束。IMU 通过局部里程计间接提供重力方向。'},
  regional: {nodes:[['当前点云 + 相对里程计','冻结区域地图与粗先验'],['SE(2) 区域匹配','鲁棒 ICP + 留出几何检查'],['S08 来源监督','GNSS 对照；map → odom']],note:'当前 S14 注册区域分支不在线重建 RTAB-Map。区域匹配不订阅图像，但共享输入预处理仍依赖双目同步。'}
};
function flow(key) {
  const m=modes[key];
  el('flow-nodes').replaceChildren(...m.nodes.map(([title,sub])=>{
    const node=document.createElement('div');node.className='flow-node';node.textContent=title;
    const small=document.createElement('small');small.textContent=sub;node.append(small);return node;
  }));
  write('flow-note',m.note);
  document.querySelectorAll('[data-flow]').forEach(b=>{const active=b.dataset.flow===key;b.setAttribute('aria-selected',String(active));b.tabIndex=active?0:-1;});
  el('flow-panel').setAttribute('aria-labelledby','tab-'+key);
}
const tabs=[...document.querySelectorAll('[data-flow]')];
tabs.forEach((b,i)=>{
  b.addEventListener('click',()=>flow(b.dataset.flow));
  b.addEventListener('keydown',e=>{
    let next=null;
    if(e.key==='ArrowRight')next=(i+1)%tabs.length;
    if(e.key==='ArrowLeft')next=(i+tabs.length-1)%tabs.length;
    if(e.key==='Home')next=0;
    if(e.key==='End')next=tabs.length-1;
    if(next!==null){e.preventDefault();flow(tabs[next].dataset.flow);tabs[next].focus();}
  });
});
flow('local');

function velocity() {
  const r=10**Number(el('wheel-r').value), gain=.008/(.008+r), v=.416+gain*(.7-.416);
  write('wheel-r-value',r.toFixed(r<.01?5:3)+' (m/s)²');
  write('velocity-result',v.toFixed(3));write('wheel-gain',(gain*100).toFixed(2)+'%');
  el('gain-bar').style.width=(gain*100)+'%';
  el('velocity-result').dataset.value=v;el('wheel-gain').dataset.value=gain;
}
el('wheel-r').addEventListener('input',velocity);
el('wheel-normal').addEventListener('click',()=>{el('wheel-r').value=Math.log10(.0025);velocity();});
el('wheel-slip').addEventListener('click',()=>{el('wheel-r').value=Math.log10(4);velocity();});
velocity();

function graph() {
  const w=Number(el('loop-weight').value),x1=(1.1+2*w)/(1+2*w),x2=2*x1;
  write('loop-value',w.toFixed(1));write('graph-x1',x1.toFixed(4));write('graph-x2',x2.toFixed(4));write('graph-error',Math.abs(x2-2).toFixed(4));
  const px=x=>145+x*190;
  let svg='<text x="0" y="51">里程计</text><text x="0" y="116">回环优化</text>';
  for(const [y,xs,color] of [[45,[0,1.1,2.2],'#738ca8'],[110,[0,x1,x2],'#087565']]){
    svg+=`<line x1="${px(0)}" y1="${y}" x2="${px(xs[2])}" y2="${y}" stroke="${color}" stroke-width="3"/>`;
    xs.forEach((x,i)=>{svg+=`<circle cx="${px(x)}" cy="${y}" r="6" fill="${color}"/><text text-anchor="middle" x="${px(x)}" y="${y+28}">x${i}=${x.toFixed(3)}</text>`;});
  }
  svg+='<text x="145" y="162">同一组观测：误差依据约束权重重新分配</text>';
  el('graph-svg').innerHTML=svg;el('graph-x2').dataset.value=x2;
}
el('loop-weight').addEventListener('input',graph);graph();

function transform() {
  const deg=Number(el('tf-yaw').value),a=deg*Math.PI/180,c=Math.cos(a),s=Math.sin(a);
  const path=[[0,0],[.4,.05],[.8,.2],[1.2,.55],[1.6,.85],[2,1]],mapped=path.map(([x,y])=>[c*x-s*y+.4,s*x+c*y-.2]);
  const pt=([x,y])=>[65+x*160,195-y*100];
  let svg='';
  for(let x=0;x<=3;x+=.5){const [px]=pt([x,0]);svg+=`<line x1="${px}" y1="15" x2="${px}" y2="220" stroke="#e2eae5"/>`;}
  for(let y=-.0;y<=1.5;y+=.5){const [,py]=pt([0,y]);svg+=`<line x1="35" y1="${py}" x2="610" y2="${py}" stroke="#e2eae5"/>`;}
  for(const [points,color] of [[path,'#6f88a8'],[mapped,'#087565']]){
    svg+=`<polyline points="${points.map(p=>pt(p).join(',')).join(' ')}" fill="none" stroke="${color}" stroke-width="3"/>`;
    const end=pt(points.at(-1));svg+=`<circle cx="${end[0]}" cy="${end[1]}" r="6" fill="${color}"/>`;
  }
  svg+='<text x="65" y="248">蓝：odom 原始坐标</text><text x="340" y="248">绿：变换后的 map 坐标</text>';
  el('tf-svg').innerHTML=svg;write('tf-yaw-value',deg+'°');write('tf-x',mapped.at(-1)[0].toFixed(3));write('tf-y',mapped.at(-1)[1].toFixed(3));
}
el('tf-yaw').addEventListener('input',transform);transform();

function gnss() {
  const cm=Number(el('gnss-offset').value),d=cm/100,nis=d*d/.0041,distOK=d<=.15,nisOK=nis<=9.21;
  write('gnss-offset-value',cm+' cm');write('distance-state',distOK?'通过':'超限');write('nis-value',nis.toFixed(2));write('nis-state',nisOK?'NIS 通过':'NIS 超限');write('gnss-result',distOK&&nisOK?'通过':'拒绝');
  el('gnss-result').style.color=distOK&&nisOK?'#196340':'#973c1e';el('gnss-result').dataset.accepted=String(distOK&&nisOK);
}
el('gnss-offset').addEventListener('input',gnss);gnss();

document.querySelectorAll('.quiz-q').forEach(q=>q.querySelectorAll('button').forEach(b=>b.addEventListener('click',()=>{
  const ok=b.dataset.choice===q.dataset.answer,out=q.querySelector('output');
  out.textContent=(ok?'判断正确。':'再想一想。')+q.dataset.feedback;out.style.color=ok?'#196340':'#973c1e';out.dataset.correct=String(ok);
})));

function filterSources(){
  const query=el('source-search').value.toLowerCase().trim();let count=0;
  const rows=[...el('sources').querySelectorAll('tbody tr')];
  rows.forEach(row=>{row.hidden=!row.textContent.toLowerCase().includes(query);if(!row.hidden)count++;});
  write('source-count',`显示 ${count} / ${rows.length} 个源码入口`);
}
el('source-search').addEventListener('input',filterSources);filterSources();
let toastTimer;
function toast(text){write('toast',text);el('toast').classList.add('show');clearTimeout(toastTimer);toastTimer=setTimeout(()=>el('toast').classList.remove('show'),2200);}
async function copyText(text){
  if(navigator.clipboard){try{await navigator.clipboard.writeText(text);return true;}catch(e){/* file:// fallback below */}}
  const t=document.createElement('textarea');t.value=text;t.style.position='fixed';t.style.opacity='0';document.body.append(t);t.select();const ok=document.execCommand('copy');t.remove();return ok;
}
document.querySelectorAll('.copy').forEach(b=>b.addEventListener('click',async()=>{
  const ok=await copyText(b.parentElement.querySelector('pre code').textContent);toast(ok?'命令已复制':'浏览器未允许复制，可选中命令手工复制');
}));
el('print').addEventListener('click',()=>window.print());
const links=[...document.querySelectorAll('.sidebar nav a')];
const observer=new IntersectionObserver(items=>{for(const item of items){if(item.isIntersecting){links.forEach(a=>a.classList.toggle('active',a.hash==='#'+item.target.id));}}},{rootMargin:'-12% 0px -70% 0px'});
document.querySelectorAll('h2[id]').forEach(h=>observer.observe(h));
window.learningReady=true;
