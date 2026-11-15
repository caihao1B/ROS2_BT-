"""Recorded costmap evidence only; no ground truth feeds the navigation stack."""
from pathlib import Path
from bisect import bisect_right
import math,json
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
run=ROOT/'optimization_v2/three_tasks/results/R12/mowing_20261115_033846_289066'
refs=[json.loads(s) for s in (run/'reference/reference.jsonl').read_text().splitlines()]
refs=[r for r in refs if 'transform' in r];times=[r['t'] for r in refs]
rows=[]
for line in (run/'diagnostics/costmap.jsonl').open():
    r=json.loads(line)
    if not 385<=r['t']<=416:continue
    ref=refs[max(0,bisect_right(times,r['t'])-1)]
    if abs(r['t']-ref['t'])>.10:continue
    ox,oy,oyaw=r['pose'];mx,my,myaw=ref['transform']
    byaw=oyaw+myaw;bx=mx+math.cos(myaw)*ox-math.sin(myaw)*oy+28;by=my+math.sin(myaw)*ox+math.cos(myaw)*oy+14
    cells=[]
    for x,y,cost in r['bad']:
        if cost!=100:continue
        wx=bx+math.cos(byaw)*x-math.sin(byaw)*y;wy=by+math.sin(byaw)*x+math.cos(byaw)*y
        cells.append([wx,wy])
    if cells:rows.append(dict(t=r['t'],reference_age_s=r['t']-ref['t'],base_world=[bx,by,byaw],lethal_cells_world=cells,world_y_max=max(y for x,y in cells),nearest_abs_body_y=min(abs(y) for x,y,cost in r['bad'] if cost==100)))
report=dict(source=str(run.relative_to(ROOT)),scope='Offline projection of recorded near-body lethal grid samples using contemporaneous estimated map-to-odom; no ground truth. Not a complete late-run costmap or an independent terrain classifier.',source_frame='recorded costmap body coordinates -> recorded odom -> estimated map -> fixed ENU/world offset [28,14]',rows=rows,observed_max_y_m=max(r['world_y_max'] for r in rows),known_mesh_edge_y_m=47.,v1_prior_buffer_m=.15,v2_prior_buffer_m=.60,resolution_m=.15,interpretation='Observed band exceeded V1 static buffer. V2 adds planning room only; no numeric online safety gate changed and no universal uncertainty bound is claimed.')
(OUT/'observed_ramp_band.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
