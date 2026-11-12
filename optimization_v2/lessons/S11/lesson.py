"""Production graph/repair/validator on a small synthetic notched polygon."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws/src/coverage_partition/scripts'))
from geometry11 import *
from partition11 import *
c=parameters();domain=box(0,0,12,9).difference(box(5,5,7,10));g=build_graph(domain,c)
raw=kmeans(g,3,41);labels,events=repair(g,raw,3,c);v=verify(domain,g,labels,3,c)
assert v['passed'];assert not verify(domain,g,[0]*len(labels),3,c)['passed']
print('PASS: three connected regions, no missing/overlap area; empty-cluster result refused.')
print(json.dumps(v,ensure_ascii=False,indent=2))
