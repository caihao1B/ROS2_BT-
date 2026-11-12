import sys,math,json,hashlib,yaml,time,os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';PKG=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'ws/src/global_pose_manager/scripts'))
from s07_common import read,dump,digest,stamp,pose,Rotation,np,verify_bundle
def validate():
 m=verify_bundle();a=yaml.safe_load((BASE/'config/gnss_alignment.yaml').read_text())
 for k,mk in [('map_id','map_id'),('frame_epoch','frame_epoch'),('map_revision','revision'),('calibration_revision','calibration_revision')]:
  if a[k]!=m[mk]:raise ValueError('alignment '+k+' mismatch')
 return a
