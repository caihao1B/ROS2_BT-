from pathlib import Path
import sys,json,hashlib,yaml
PKG=Path(__file__).resolve().parents[1];BASE=PKG.parents[2];ROOT=BASE.parent
P15=PKG.parent/'motion_prediction';sys.path.insert(0,str(P15/'scripts'))
from common15 import read,dump,sha
FEATURE_VERSION='context16.1-current-ego-corridor'
def config():return yaml.safe_load((BASE/'config/memory.yaml').read_text())
def versions():
 # Binding the effective registered atlas and the entire frozen predictor configuration.
 d=read(BASE/'results/S14/latest.json')
 return dict(map=hashlib.sha256(json.dumps(d['dependency_receipts'],sort_keys=True).encode()+sha(BASE/'results/S14/latest.json').encode()).hexdigest(),model=sha(BASE/'config/model_lock.yaml')+':'+sha(BASE/'config/prediction.yaml'),feature=FEATURE_VERSION)
