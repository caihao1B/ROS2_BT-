"""OpenCV Zoo NanoDet ONNX adapter, RGB letterbox and calibrated projection."""
import sys,time,hashlib
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'vendor/nanodet'))
from nanodet import NanoDet
class Detector:
 def __init__(self,model,sha):
  if hashlib.sha256(Path(model).read_bytes()).hexdigest()!=sha:raise ValueError('model SHA mismatch')
  cv2.setNumThreads(2);self.net=NanoDet(str(model),.35,.6)
 def infer(self,rgb):
  start=time.monotonic();h,w=rgb.shape[:2];scale=min(416/w,416/h);nw,nh=int(w*scale),int(h*scale);left=(416-nw)//2;top=(416-nh)//2;im=cv2.copyMakeBorder(cv2.resize(rgb,(nw,nh)),top,416-nh-top,left,416-nw-left,cv2.BORDER_CONSTANT,value=0);raw=self.net.infer(im);result=[]
  for r in raw:
   box=[float(np.clip((r[0]-left)/scale,0,w)),float(np.clip((r[1]-top)/scale,0,h)),float(np.clip((r[2]-left)/scale,0,w)),float(np.clip((r[3]-top)/scale,0,h))];result.append(dict(box=box,score=float(r[4]),class_id=int(r[5])))
  return result,(time.monotonic()-start)*1000

def project_box(obj,odom_R,odom_p,fx,cx,cy):
 lo=obj['minimum'];hi=obj['maximum'];world=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]]);body=(world-np.array(odom_p))@odom_R;optical=np.c_[-(body[:,1]-.06),-(body[:,2]-.66),body[:,0]-.25];valid=optical[:,2]>.2
 if valid.sum()<4:return None
 v=optical[valid];u=fx*v[:,0]/v[:,2]+cx;vv=fx*v[:,1]/v[:,2]+cy;return [float(u.min()),float(vv.min()),float(u.max()),float(vv.max())],float(np.median(v[:,2]))
def iou(a,b):
 x=max(0,min(a[2],b[2])-max(a[0],b[0]));y=max(0,min(a[3],b[3])-max(a[1],b[1]));inter=x*y;return inter/max(1e-9,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter)


def unique_semantics(objects):
 """One visual box can label at most one geometry cluster; all remain obstacles."""
 choices={}
 for i,o in enumerate(objects):
  b=o.get('semantic',{}).get('detector_box')
  if b is None:continue
  if o['classification']=='person' and not (.6<o['size'][2]<2.3 and max(o['size'][:2])<1.5):
   o.update(classification='unknown_obstacle',confidence=0.);continue
  key=tuple(b);score=iou(o['semantic']['projected_box'],b)
  if key not in choices or score>choices[key][0]:choices[key]=(score,i)
 chosen={i for _,i in choices.values()}
 for i,o in enumerate(objects):
  if o.get('semantic',{}).get('detector_box') is not None and i not in chosen:o.update(classification='unknown_obstacle',confidence=0.);o['semantic']['rejection']='DUPLICATE_OR_GEOMETRY_INCOMPATIBLE'
 return objects
