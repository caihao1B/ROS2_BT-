"""Image semantics are optional; geometric tracking does not wait for classification."""
import sys,argparse,signal,collections,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'terrain_perception/scripts'))
from common09 import *
import rclpy,cv2
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from detector import Detector,project_box,iou,unique_semantics
from tracker import Tracker

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--case',required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);cfg=yaml.safe_load((BASE/'config/detector_lock.yaml').read_text());detector=Detector(ROOT/cfg['path'],cfg['sha256']);tracker=Tracker();rclpy.init();n=rclpy.create_node('s09_object_fusion');images={x:collections.deque(maxlen=12) for x in ['left','right']};odoms=collections.deque(maxlen=100);geometry=collections.deque(maxlen=3);st=dict(stop=False,last=-1.,snapshot=-10.,fx=None);f=(a.out/'tracks.jsonl').open('w');pub=n.create_publisher(String,'/s09/objects',10);markers=n.create_publisher(MarkerArray,'/s09/objects/markers',2);annotated=n.create_publisher(Image,'/s09/camera/annotated',2)
 for side in images:n.create_subscription(Image,'/s04/stereo/'+side+'/image_rect',lambda m,s=side:images[s].append(m),qos_profile_sensor_data)
 n.create_subscription(Odometry,'/s05/odometry',lambda m:odoms.append(m),100)
 n.create_subscription(String,'/s09/geometry',lambda m:geometry.append(json.loads(m.data)),10)
 def info(m):st['fx']=m.k[0];st['cx']=m.k[2];st['cy']=m.k[5]
 from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
 n.create_subscription(CameraInfo,'/s04/stereo/left/camera_info',info,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
 stereo=cv2.StereoSGBM_create(minDisparity=0,numDisparities=96,blockSize=5,P1=8*25,P2=32*25,disp12MaxDiff=1,uniquenessRatio=8,speckleWindowSize=80,speckleRange=2)
 def process():
  if not geometry:return
  d=geometry.popleft();t=d['t']
  if t<=st['last']:return
  st['last']=t;paired=[m for m in images['left'] if any(abs(stamp(m)-stamp(r))<1e-6 for r in images['right'])];candidates=paired if paired and min(abs(stamp(m)-t) for m in paired)<=.08 else images['left'];image=min(candidates,key=lambda m:abs(stamp(m)-t)) if candidates else None;camera_ok=image is not None and abs(stamp(image)-t)<=.08 and st['fx'] is not None and not(a.case=='camera_loss' and 25<=t<45);detections=[];latency=0.;preview=None;disparity=None;image_t=stamp(image) if image else None;sync=None;op=None
  if camera_ok and odoms:
   op=min(odoms,key=lambda m:abs(stamp(m)-image_t));camera_ok=abs(stamp(op)-image_t)<=.05
  if camera_ok and op:
   rgb=np.frombuffer(image.data,np.uint8).reshape(image.height,image.step)[:,:image.width*3].reshape(image.height,image.width,3).copy();detections,latency=detector.infer(rgb);preview=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR);right=next((m for m in images['right'] if abs(stamp(m)-image_t)<1e-6),None)
   if right:
    ri=np.frombuffer(right.data,np.uint8).reshape(right.height,right.step)[:,:right.width*3].reshape(right.height,right.width,3);disparity=stereo.compute(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY),cv2.cvtColor(ri,cv2.COLOR_RGB2GRAY)).astype(float)/16;sync=abs(stamp(right)-image_t)
  observed=[]
  for o in d['objects']:
   o=dict(o);o.update(classification='unknown_obstacle',confidence=0.,semantic=dict(camera_valid=bool(camera_ok),image_t=image_t,stereo_pair_delta_s=sync))
   if camera_ok and op:
    pos,q=pose(op);projection=project_box(o,Rotation.from_quat(q).as_matrix(),pos,st['fx'],st['cx'],st['cy'])
    if projection:
     box,depth=projection;best=max(detections,key=lambda r:iou(box,r['box']),default=None);o['semantic'].update(projected_box=box,lidar_depth_m=depth,epoch_difference_s=abs(image_t-t));stereo_depth=None
     if disparity is not None:
      x1,y1,x2,y2=map(int,[np.clip(box[0],0,image.width),np.clip(box[1],0,image.height),np.clip(box[2],0,image.width),np.clip(box[3],0,image.height)]);part=disparity[y1:y2,x1:x2];valid=part[(part>2)&(part<96)]
      if valid.size>=12:stereo_depth=float(st['fx']*.12/np.median(valid))
     agreement=stereo_depth is None or abs(stereo_depth-depth)<.6+.15*depth;o['semantic'].update(stereo_depth_m=stereo_depth,depth_consistent=bool(agreement))
     if best and iou(box,best['box'])>=.12 and agreement:o.update(classification=cfg['classes'].get(best['class_id'],'other_visual_class'),confidence=best['score']);o['semantic']['detector_box']=best['box']
   # Exposure/LiDAR epoch residual increases XY uncertainty (2 m/s unmodelled target bound).
   cov=np.array(o['covariance']).reshape(3,3);cov[:2,:2]+=np.eye(2)*(2*abs(image_t-t))**2 if camera_ok else np.eye(2)*.0064;o['covariance']=cov.reshape(-1).tolist();observed.append(o)
  tracks,ambiguous=tracker.update(t,unique_semantics(observed));record=dict(t=t,frame='odom',objects=tracks,camera_valid=bool(camera_ok),detector_ms=latency,detections=detections,ambiguous_measurements=ambiguous,stereo_available=disparity is not None);msg=String();msg.data=json.dumps(record);pub.publish(msg);f.write(msg.data+'\n');f.flush();ma=MarkerArray();delete=Marker();delete.action=Marker.DELETEALL;ma.markers.append(delete)
  for tr in tracks:
   m=Marker();m.header.frame_id='odom';m.header.stamp.sec=int(t);m.header.stamp.nanosec=int((t-int(t))*1e9);m.ns='S09_tracks';m.id=tr['id'];m.type=Marker.CUBE;m.action=Marker.ADD;m.pose.position.x,m.pose.position.y,m.pose.position.z=tr['position'];m.pose.orientation.w=1.;m.scale.x,m.scale.y,m.scale.z=[max(.08,x) for x in tr['size']];m.color.r=1.;m.color.g=.5 if tr['classification']=='person' else .2;m.color.a=.4;ma.markers.append(m)
   text=Marker();text.header=m.header;text.ns='S09_labels';text.id=tr['id'];text.type=Marker.TEXT_VIEW_FACING;text.pose=copy.deepcopy(m.pose);text.pose.position.z+=tr['size'][2]/2+.2;text.scale.z=.25;text.color.r=text.color.g=text.color.b=text.color.a=1.;text.text=str(tr['id'])+' '+tr['classification']+' '+tr['state'];ma.markers.append(text)
  markers.publish(ma)
  if preview is not None:
   for det in detections:
    x1,y1,x2,y2=map(int,det['box']);cv2.rectangle(preview,(x1,y1),(x2,y2),(0,220,100),2);cv2.putText(preview,cfg['classes'].get(det['class_id'],str(det['class_id']))+' %.2f'%det['score'],(x1,max(15,y1-4)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
   im=Image();im.header=image.header;im.height,im.width=preview.shape[:2];im.encoding='bgr8';im.step=im.width*3;im.data=preview.tobytes();annotated.publish(im)
   if t-st['snapshot']>=5:st['snapshot']=t;cv2.imwrite(str(a.out/f'camera_{int(t):03}.jpg'),preview);dump(a.out/f'camera_{int(t):03}.json',record)
  dump(a.out/'live.json',record)
 def end(s,f):st['stop']=True
 signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('actual ONNX inference, geometric association; no truth subscriptions')
 try:
  while not st['stop']:
   rclpy.spin_once(n,timeout_sec=.01)
   for _ in range(30):rclpy.spin_once(n,timeout_sec=0.)
   process()
 finally:f.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
