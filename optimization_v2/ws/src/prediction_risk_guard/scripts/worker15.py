"""Original single Nav2/action owner with S15 task wrapper."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'motion_prediction/scripts'))
from common15 import add14,S14
once=add14()
from worker14 import adapted_source
if __name__=='__main__':
    source=once(adapted_source(),'from mission14 import Mission','from mission15 import Mission')
    exec(compile(source,str(Path(__file__).resolve()),'exec'),{'__name__':'__main__','__file__':str(Path(__file__).resolve())})
