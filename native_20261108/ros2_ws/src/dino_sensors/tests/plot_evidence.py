#!/usr/bin/env python3
import json,pathlib,sys,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
A=pathlib.Path(sys.argv[1]);O=A/'figures';O.mkdir(exist_ok=True)
views=json.loads((A/'dual01/scan_views.json').read_text());fig,axes=plt.subplots(1,3,figsize=(15,5),layout='constrained');summary={}
for j,(key,view) in enumerate(views.items()):
    raw=np.array([np.nan if x is None else x for x in view['scan']['ranges']]);filtered=np.array([np.nan if x is None else x for x in view['filtered']['ranges']]);angles=np.array(view['world_angles']);x,y=view['world_origin'];mask=np.array(view['self_mask'])
    axes[0].scatter(x+filtered*np.cos(angles),y+filtered*np.sin(angles),s=2,label=key+' retained')
    axes[0].scatter(x+raw[mask]*np.cos(angles[mask]),y+raw[mask]*np.sin(angles[mask]),s=4,color='crimson')
    axes[0].scatter([x],[y],marker='x',s=60);axes[0].annotate(key,(x,y),xytext=(5,8),textcoords='offset points')
    ax=axes[j+1];az=np.degrees(view['scan']['angle_min']+np.arange(len(raw))*view['scan']['angle_increment']);ax.plot(az,raw,color='.75',lw=1,label='raw');ax.plot(az,filtered,color='#0072B2',lw=1,label='retained');ax.scatter(az[mask],raw[mask],s=4,color='crimson',label='body removed')
    ax.set(title=key+' scan (static)',xlabel='Azimuth in sensor frame (deg)',ylabel='Range (m)',xlim=(-180,180),ylim=(0,8.2));ax.legend(fontsize=8);ax.grid(alpha=.2)
    blocks=[];start=None
    for i,flag in enumerate(list(mask)+[False]):
        if flag and start is None:start=i
        elif not flag and start is not None:blocks.append([float(az[start]),float(az[i-1])]);start=None
    summary[key]={'samples':len(raw),'self_occluded_fraction':float(mask.mean()),'self_occluded_angular_bins_deg':blocks,'sensor_origin_in_initial_world_m':[x,y]}
axes[0].add_patch(Rectangle((-1.391168,-1.386727),2.782336,2.773454,facecolor='gray',alpha=.15,edgecolor='black',label='Collision AABB'))
axes[0].set(title='Two separate scan origins',xlabel='World x (m)',ylabel='World y (m)',xlim=(-6.4,6.4),ylim=(-6.4,6.4));axes[0].set_aspect('equal');axes[0].legend(fontsize=8,loc='upper left');axes[0].grid(alpha=.2)
fig.suptitle('Dino: Ogre2 software rendering / 1440 samples / 5 Hz; body mask is NOT free space');fig.savefig(O/'lidar_layout.png',dpi=160);plt.close(fig)
(A/'lidar_layout.json').write_text(json.dumps(summary,indent=2))
rows=[json.loads(s) for s in (A/'trajectory01/samples.jsonl').read_text().splitlines()];allowed={'static','straight','stop','left','stop_left','right','stop_right'}
series={kind:[r for r in rows if r['kind']==kind and r['phase'] in allowed] for kind in ('truth','wheel','fused')};truth=series['truth'];tt=np.array([r['t'] for r in truth]);gtx=np.array([r['x'] for r in truth]);gty=np.array([r['y'] for r in truth]);gth=np.unwrap([r['yaw'] for r in truth]);fig,axes=plt.subplots(1,3,figsize=(15,5),layout='constrained')
for kind,color in [('truth','black'),('wheel','#D55E00'),('fused','#0072B2')]:
    data=series[kind];t=np.array([r['t'] for r in data]);x=np.array([r['x'] for r in data]);y=np.array([r['y'] for r in data]);h=np.unwrap([r['yaw'] for r in data]);valid=(t>=tt[0]+.5)&(t<=tt[-1]);t,x,y,h=t[valid],x[valid],y[valid],h[valid]
    dx=x-x[0];dy=y-y[0];cs,ss=math.cos(h[0]),math.sin(h[0]);lx=cs*dx+ss*dy;ly=-ss*dx+cs*dy;axes[0].plot(lx,ly,label=kind,color=color,lw=1.5)
    if kind!='truth':
        gx=np.interp(t,tt,gtx);gy=np.interp(t,tt,gty);gh=np.interp(t,tt,gth);dx=gx-gx[0];dy=gy-gy[0];cs,ss=math.cos(gh[0]),math.sin(gh[0]);ex=cs*dx+ss*dy;ey=-ss*dx+cs*dy
        axes[1].plot(t-t[0],np.hypot(lx-ex,ly-ey),label=kind,color=color);axes[2].plot(t-t[0],np.degrees((h-h[0])-(gh-gh[0])),label=kind,color=color)
axes[0].set(title='Straight + left + right trajectory',xlabel='Local x (m)',ylabel='Local y (m)');axes[0].set_aspect('equal');axes[1].set(title='Cumulative position error',xlabel='Simulation time (s)',ylabel='Error (m)');axes[2].set(title='Cumulative heading error',xlabel='Simulation time (s)',ylabel='Error (deg)')
for ax in axes:ax.grid(alpha=.25);ax.legend()
fig.suptitle('Ground truth for evaluation only; align initial pose once (no segment resets)');fig.savefig(O/'local_trajectory.png',dpi=160);plt.close(fig)
print(json.dumps({'status':'PASS','figures':[str(p) for p in O.glob('*.png')],'layout':summary},indent=2))
