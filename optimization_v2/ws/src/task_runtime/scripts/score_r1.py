"""Independent old task scoring kept separate from new lifecycle acceptance."""
from common_r1 import *
import argparse
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('out',type=Path);a=ap.parse_args()
    if a.out.resolve().parent.parent!=APP/'results':raise ValueError('SCORE_SCOPE')
    add17();from evaluate17 import evaluate
    evaluate(a.out,read(a.out/'spec.json'),BASE/'build_s17/monitor')
