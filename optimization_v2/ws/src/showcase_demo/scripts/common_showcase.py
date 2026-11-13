"""Separate demonstration tasks and receipts; never formal S17 credit."""
from pathlib import Path
import sys,os,json,hashlib,time
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';PKG=BASE/'ws/src/showcase_demo';OUT=BASE/'showcase'
sys.path.insert(0,str(BASE/'ws/src/acceptance_runner/scripts'))
from common17 import read,dump,sha,canonical
