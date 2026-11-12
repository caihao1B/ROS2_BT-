"""ROS observation transport with one bounded asynchronous inference worker."""
from common15 import *
import argparse,os,signal,time,subprocess,selectors,collections,math
import yaml
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--method',choices=['no_prediction','imm','learned'],required=True);ap.add_argument('--fault',default='none',choices=['none','timeout','uncertainty']);a=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='119':raise ValueError('Online observation domain required')
    a.out.mkdir(parents=True,exist_ok=True);cfg=yaml.safe_load((BASE/'config/prediction.yaml').read_text())
    log=(a.out/'forecasts.jsonl').open('w');events=(a.out/'events.jsonl').open('w');err=(a.out/'worker.log').open('w')
    worker_env=os.environ.copy();worker_env.pop('PYTHONPATH',None);worker_env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    worker=subprocess.Popen([str(BASE/'venvs/prediction/bin/python'),str(PKG/'scripts/model_worker15.py'),'--method',a.method],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err,bufsize=0,env=worker_env)
    sel=selectors.DefaultSelector();sel.register(worker.stdout,selectors.EVENT_READ);buffer=b'';deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        if worker.poll() is not None:raise RuntimeError('Prediction worker startup failed')
        if sel.select(.1):
            buffer+=os.read(worker.stdout.fileno(),65536)
            if b'\n' in buffer:break
    line,sep,buffer=buffer.partition(b'\n')
    if not sep or not json.loads(line).get('ready'):raise TimeoutError('Prediction worker ready timeout')
    os.set_blocking(worker.stdin.fileno(),False)
    write_buffer=b''
    rclpy.init();n=rclpy.create_node('s15_motion_prediction',parameter_overrides=[Parameter('use_sim_time',value=True)])
    pub=n.create_publisher(String,'/s15/forecasts',5);markers=n.create_publisher(MarkerArray,'/s15/predictions/markers',2)
    st=dict(stop=False,pending=None,last_send=-10.,id=0,last_t=-1.);queue=collections.deque(maxlen=60)
    n.create_subscription(String,'/s09/objects',lambda m:queue.append(json.loads(m.data)),30)
    def emit(value):
        pub.publish(String(data=json.dumps(value,allow_nan=False)));log.write(json.dumps(value,allow_nan=False)+'\n');log.flush();dump(a.out/'live.json',value)
        ma=MarkerArray();d=Marker();d.action=Marker.DELETEALL;ma.markers.append(d);i=0
        for target in value.get('forecasts',[]):
            for mode in target['modes']:
                if mode['probability']<.015:continue
                m=Marker();m.header.frame_id='odom';m.header.stamp=n.get_clock().now().to_msg();m.ns='prediction_modes';m.id=i;i+=1;m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=.02+.06*mode['probability'];m.color.r=.9 if a.method=='learned' else .1;m.color.g=.5;m.color.b=.9;m.color.a=.3+.7*mode['probability'];m.points=[Point(x=float(x),y=float(y),z=.15) for x,y in mode['mean']];ma.markers.append(m)
            best=max(target['modes'],key=lambda x:x['probability']);m=Marker();m.header.frame_id='odom';m.header.stamp=n.get_clock().now().to_msg();m.ns='prediction_probabilities';m.id=i;i+=1;m.type=Marker.TEXT_VIEW_FACING;m.pose.position=Point(x=best['mean'][-1][0],y=best['mean'][-1][1],z=1.);m.pose.orientation.w=1.;m.scale.z=.18;m.color.r=m.color.g=m.color.b=m.color.a=1.;m.text=f"{target['id']} {target['algorithm']} p={best['probability']:.2f}";ma.markers.append(m)
        markers.publish(ma)
    def stop(s,f):st['stop']=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    (a.out/'ready').write_text('fixed actual predictor loaded; observation domain119 only')
    try:
        while not st['stop']:
            rclpy.spin_once(n,timeout_sec=.005);t=n.get_clock().now().nanoseconds*1e-9
            if worker.poll() is not None:raise RuntimeError('Prediction worker lost')
            if sel.select(0):
                buffer+=os.read(worker.stdout.fileno(),1048576)
                while b'\n' in buffer:
                    line,buffer=buffer.split(b'\n',1);reply=json.loads(line)
                    if st['pending'] is None or reply['id']!=st['pending']['id']:continue
                    value=reply['result'];value['transport_wall_ms']=(time.monotonic()-st['pending']['wall'])*1000
                    fault=a.fault=='timeout' and 18<=t<23
                    late=time.monotonic()-st['pending']['wall']>cfg['inference_timeout_wall_s']
                    if fault or late:
                        emit(dict(t=t,frame='odom',method=a.method,status='INFERENCE_TIMEOUT',forecasts=[],injected_fault=fault,request_id=reply['id']))
                        events.write(json.dumps(dict(t=t,event='timeout',id=reply['id'],injected=fault))+'\n');events.flush()
                        st['pending']=None
                    else:
                        if a.fault=='uncertainty' and 18<=t<23:
                            for target in value.get('forecasts',[]):
                                for mode in target['modes']:mode['covariance']=[[[100.,0.],[0.,100.]] for _ in mode['mean']]
                            value['injected_fault']='HIGH_UNCERTAINTY'
                        emit(value);st['pending']=None
            if st['pending'] and time.monotonic()-st['pending']['wall']>cfg['inference_timeout_wall_s'] and not st['pending'].get('timed_out'):
                emit(dict(t=t,frame='odom',method=a.method,status='INFERENCE_TIMEOUT',forecasts=[],request_id=st['pending']['id']))
                events.write(json.dumps(dict(t=t,event='timeout',id=st['pending']['id']))+'\n');events.flush();st['pending']['timed_out']=True
            if st['pending'] and time.monotonic()-st['pending']['wall']>2.:
                raise TimeoutError('Unresponsive owned worker: end case on baseline safety')
            if write_buffer:
                try:
                    written=os.write(worker.stdin.fileno(),write_buffer);write_buffer=write_buffer[written:]
                except BlockingIOError:pass
            if not st['pending'] and queue and t-st['last_send']>=cfg['inference_period_sim_s']:
                frames=[x for x in queue if x['t']>st['last_t']];queue.clear()
                if not frames:continue
                st['last_t']=frames[-1]['t'];st['last_send']=t;st['id']+=1
                payload=dict(id=st['id'],observations=frames)
                write_buffer=(json.dumps(payload)+'\n').encode()
                st['pending']=dict(id=st['id'],wall=time.monotonic())
    finally:
        try:os.write(worker.stdin.fileno(),b'{"stop":true}\n');worker.wait(timeout=2)
        except (BrokenPipeError,BlockingIOError,subprocess.TimeoutExpired):worker.terminate();worker.wait(timeout=2)
        dump(a.out/'worker_exit.json',dict(exit_code=worker.returncode));sel.close();log.close();events.close();err.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
