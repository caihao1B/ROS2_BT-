"""S03 test-only trajectory client. Uses public route + encoder odometry + range scan.
Not Nav2 or a production navigator; known initial pose is a test fixture alignment.
"""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
def service(req):
    r=subprocess.run(['gz','service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','4000','--req',req],capture_output=True,text=True,timeout=6)
    if r.returncode or 'true' not in r.stdout:raise RuntimeError('world control failed: '+r.stdout+r.stderr)
def phases(c):
    items=[]
    for speed in c['qualification_candidates']['transit_mps']:items += [(f'straight_{speed}','ARM',.5,0,0),(f'straight_{speed}','DRIVE',6,speed,0),(f'stop_{speed}','STOP',3,0,0)]
    turns=[('left',.6,.24,28),('right',.6,-.24,28)]
    if c['id']=='ackermann':turns.extend([('minimum_radius',.6,.30,24),('minimum_radius_right',.6,-.30,24)])
    for name,v,w,duration in turns+[('reverse',-.25,0,6),('s_bend',.6,0,18),('zero_spin',0,.4,4)]:items += [(name,'ARM',.5,0,0),(name,'DRIVE',duration,v,w),('stop_'+name,'STOP',3,0,0)]
    return items
def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--case',required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--seconds',type=float,required=True);p.add_argument('--wall',type=float,required=True);p.add_argument('--pids',type=Path,required=True);p.add_argument('--route',type=Path,required=True);a=p.parse_args()
    c=json.loads(a.config.read_text());a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=rclpy.create_node('s03_test_driver');pub=n.create_publisher(String,'/s03/request',1)
    state=dict(t=0.,total=0.,epoch=0,odom=None,encoders=None,status=None,scan=None,scan_wall=-1e9,phase='startup',mode='STOP',v=0.,w=0.,stopped=False,events=[],completed=False)
    logs=(a.out/'online.jsonl').open('w');start=time.monotonic();seq=0;last_save=-1;last_send=0;fault=False;fault_total=None;intruder=None;done_since=None;path_idx=0
    def end(s,f):state['stopped']=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
    def clk(m):
        t=m.clock.sec+m.clock.nanosec*1e-9;state['total']+=max(0,t-state['t']);state['t']=t
    def enc(m):state['encoders']=json.loads(m.data);state['epoch']=state['encoders']['epoch']
    def status(m):state['status']=json.loads(m.data)
    def odom(m):
        q=m.pose.pose.orientation;state['odom']=[m.pose.pose.position.x,m.pose.pose.position.y,2*math.atan2(q.z,q.w),m.twist.twist.linear.x,m.twist.twist.angular.z]
    def scan(m):state['scan']=dict(ranges=list(m.ranges),angle_min=m.angle_min,angle_increment=m.angle_increment);state['scan_wall']=time.monotonic()
    n.create_subscription(Clock,'/clock',clk,qos_profile_sensor_data);n.create_subscription(String,'/s03/encoders',enc,qos_profile_sensor_data);n.create_subscription(String,'/s03/drive/status',status,10);n.create_subscription(Odometry,'/s03/wheel/odometry',odom,10);n.create_subscription(LaserScan,'/s03/front_scan',scan,qos_profile_sensor_data)
    path=None
    if a.case=='connector':path=np.array(json.loads(a.route.read_text())['features'][1]['geometry']['coordinates'])
    elif a.case=='slope':path=np.array([[37,63.5],[49,63.5]])
    elif a.case=='dock':path=np.array([[70,12],[72,12]])
    if path is not None:
        path=path[np.r_[True,np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-6]]
        # Densify line and connector to <= 0.15 m for monotonic pure pursuit.
        dense=[]
        for x,y in zip(path[:-1],path[1:]):dense.extend(np.linspace(x,y,max(2,int(np.linalg.norm(y-x)/.15)+1))[:-1])
        path=np.array(dense+[path[-1]]);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
        (a.out/'target_path.json').write_text(json.dumps(path.tolist()))
    bench=phases(c);bench_end=sum(x[2] for x in bench)+2
    (a.out/'driver_ready').write_text('ready')
    try:
        while not state['stopped'] and time.monotonic()-start<a.wall and state['total']<a.seconds:
            rclpy.spin_once(n,timeout_sec=.01);now=time.monotonic()
            if now-last_send<.02:continue
            last_send=now;t=state['t'];elapsed=state['total'];mode='STOP';v=w=0.;phase='startup';send=True
            if elapsed>=2:
                if a.case=='bench':
                    cursor=2
                    for name,m,d,sv,sw in bench:
                        if elapsed<cursor+d:
                            phase=name;mode=m;v=sv;w=sw
                            if name=='s_bend' and mode=='DRIVE':w=.18*math.sin(2*math.pi*(elapsed-cursor)/18)
                            break
                        cursor+=d
                    else:state['completed']=True;break
                elif path is not None:
                    phase=a.case
                    if elapsed<2.5:mode='ARM'
                    elif state['odom']:
                        ox,oy,heading,ov,ow=state['odom'];ix,iy,ih=c['initial_pose'];co,si=math.cos(ih),math.sin(ih);xy=np.array([ix+co*ox-si*oy,iy+si*ox+co*oy]);yaw=heading+ih
                        nearest=path_idx+int(np.argmin(np.linalg.norm(path[path_idx:]-xy,axis=1)));path_idx=max(path_idx,nearest);remaining=arc[-1]-arc[path_idx];distance=float(np.linalg.norm(xy-path[-1]))
                        state['path_progress']=float(arc[path_idx]);state['goal_error_odom']=distance
                        if remaining<.2 and distance<(.025 if a.case=='dock' else .06):
                            mode='STOP';phase='goal_stop'
                            if done_since is None:done_since=elapsed
                            if elapsed-done_since>2 and abs(ov)<.01 and abs(ow)<.02:state['completed']=True;break
                        else:
                            lookahead=.7 if a.case=='dock' else 1.2;target=min(len(path)-1,int(np.searchsorted(arc,arc[path_idx]+lookahead)));delta=path[target]-xy;lx=math.cos(yaw)*delta[0]+math.sin(yaw)*delta[1];ly=-math.sin(yaw)*delta[0]+math.cos(yaw)*delta[1];k=2*ly/max(.15,float(delta@delta));k=max(-.5,min(.5,k))
                            cap=.4 if a.case in ['slope','dock'] else c['qualification_candidates']['connector_mps'];v=min(cap,max(.07,.7*distance));v=min(v,.8 if abs(k)>.05 else cap);w=v*k;mode='DRIVE'
                            if a.case=='connector':
                                if now-state['scan_wall']>.3:v=w=0.;phase='range_unavailable'
                                else:
                                    z=state['scan'];rr=np.array(z['ranges']);aa=z['angle_min']+np.arange(len(rr))*z['angle_increment'];xx=rr*np.cos(aa);yy=rr*np.sin(aa);mask=np.isfinite(rr)&(xx>0)&(np.abs(yy)<.68)
                                    nearest_range=float(np.min(xx[mask])) if mask.any() else 8.;state['nearest_forward_m']=nearest_range
                                    braking=.35*abs(ov)+ov*ov/(2*.8)+.65
                                    if nearest_range<braking:v=w=0.;phase='range_wait'
                else:
                    phase='fault_drive'
                    if elapsed<2.5:mode='ARM'
                    elif elapsed<7.5:mode='DRIVE';v=.6
                    else:
                        mode='DRIVE';v=.6;phase=a.case
                        if not fault:
                            fault=True;fault_total=elapsed;state['events'].append(dict(name=a.case,t=t,total=elapsed,wall=now))
                            if a.case in ['adapter_loss','actuator_loss','feedback_loss']:
                                role={'adapter_loss':'drive','actuator_loss':'request_bridge','feedback_loss':'feedback_bridge'}[a.case];d=json.loads(a.pids.read_text())[role]
                                from session_owner import identity
                                live=identity(d['pid'])
                                if live['start_ticks']!=d['start_ticks']:raise RuntimeError('owned PID identity changed')
                                # A crashed arbiter must not get a graceful final STOP.
                                os.kill(d['pid'],signal.SIGKILL if a.case=='adapter_loss' else signal.SIGINT)
                            elif a.case=='pause':service('pause: true');time.sleep(.8);service('pause: false')
                            elif a.case=='reset':service('reset: {all: true}')
                            elif a.case=='duplicate_owner':intruder=rclpy.create_node('s03_intruder');rogue=intruder.create_publisher(String,'/s03/request',1)
                        if a.case=='command_loss':send=False
                        elif a.case=='cancel':mode='STOP';v=0.
                        if elapsed-fault_total>5:state['completed']=True;break
            state.update(phase=phase,mode=mode,v=v,w=w)
            if send:
                seq+=1;msg=String();msg.data=json.dumps(dict(sequence=seq,epoch=state['epoch'],stamp=t-1 if fault and a.case=='stale_timeout' else t,mode=mode,v=v,w=w));pub.publish(msg)
            if state['total']-last_save>=.04:
                last_save=state['total'];row={k:state[k] for k in ['t','total','epoch','odom','encoders','status','phase','mode','v','w']};row['wall']=now
                for k in ['path_progress','goal_error_odom','nearest_forward_m']:
                    if k in state:row[k]=state[k]
                logs.write(json.dumps(row,separators=(',',':'))+'\n')
    except Exception as exc:state['error']=repr(exc)
    finally:
        # Best effort explicit STOP before process teardown; native watchdog is independent.
        for _ in range(8):
            seq+=1;m=String();m.data=json.dumps(dict(sequence=seq,epoch=state['epoch'],stamp=state['t'],mode='STOP',v=0.,w=0.));pub.publish(m);rclpy.spin_once(n,timeout_sec=.02)
        state['graph']={name:types for name,types in n.get_topic_names_and_types()};state['tf_publishers']=[i.node_name for i in n.get_publishers_info_by_topic('/tf')];state['actuator_publishers']=[i.node_name for i in n.get_publishers_info_by_topic('/s03/actuator/request')];state['read_inputs']=[str(a.config),str(a.route)]
        state.pop('scan',None);logs.close();(a.out/'result.json').write_text(json.dumps(state,indent=2)+'\n');n.destroy_node()
        if intruder:intruder.destroy_node()
        rclpy.try_shutdown()
    return 0 if state['completed'] and not state['stopped'] and 'error' not in state else 1
if __name__=='__main__':
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'project_baseline/scripts'))
    raise SystemExit(main())
