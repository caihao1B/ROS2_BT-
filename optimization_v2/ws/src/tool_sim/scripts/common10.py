import sys,os,json,math,time,hashlib,yaml,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'optimization_v2'
PKG=Path(__file__).resolve().parents[1]
LEDGER=BASE/'ws/src/work_ledger'
sys.path.insert(0,str(BASE/'ws/src/terrain_perception/scripts'))
from common09 import read,dump,digest,stamp,pose,Rotation,np,verify_bundle
sys.path.insert(0,str(PKG/'scripts'))
sys.path.insert(0,str(LEDGER/'scripts'))
def canonical(value):
 return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def contract(kind):
 c=yaml.safe_load((BASE/'config/tool_contract.yaml').read_text());c['vehicle']=kind
 c['tool']=yaml.safe_load((BASE/'config/tool_profiles.yaml').read_text())['spray' if kind=='diff' else 'mower']
 c['tool_revision']=canonical(dict(contract=c,geometry_sha=digest(BASE/'config/tool_profiles.yaml')))
 return c
