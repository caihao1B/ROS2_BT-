"""Run the qualified private Nav2 obstacle probe on independently versioned path gears in map coordinates."""
import argparse
import inspect
import json
import math
from pathlib import Path

from common import sha
from recovery_mowing_r10 import gear_runs
from mowing_yaw_candidate_r10 import replace_once
import probe_mowing_transfer_nav_r10 as original


def run_file(path, domain=179, gear_prefix=None):
    path=path.resolve()
    payload=json.loads(path.read_text())
    if gear_prefix is None:
        segments=payload['segments']
    else:
        segments=[dict(id=f'{gear_prefix}:{index}',direction=part['direction'],poses=part['poses'])
                  for index,part in enumerate(gear_runs(payload['points']))]
    if not segments or not 1<=len(segments)<=30 or not 169<=domain<=200 or domain in (173,174,178):
        raise ValueError('A separate diagnostic domain and 1–30 declared gear paths are required')
    points=[]
    for seg in segments:
        if not isinstance(seg.get('id'),str) or seg.get('direction') not in (-1,1) or len(seg.get('poses',[]))<2:
            raise ValueError('Each gear must have id, signed direction and map-frame poses')
        for pose in seg['poses']:
            if len(pose)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in pose):
                raise ValueError('Every map pose must be finite x,y,yaw')
            points.append(pose)
    resolution=.15
    x0=math.floor((min(p[0] for p in points)-4.)/resolution)*resolution
    y0=math.floor((min(p[1] for p in points)-4.)/resolution)*resolution
    width=math.ceil((max(p[0] for p in points)+4.-x0)/resolution)
    height=math.ceil((max(p[1] for p in points)+4.-y0)/resolution)
    if max(width,height)>1000 or len(points)>50000:
        raise ValueError('Fixture requires a larger map than this bounded component test')
    plan=dict(segments=segments,qualified_original_entry=segments[0]['poses'][0])
    # Reuse the exact original service lifecycle, clear/block/clear protocol and
    # shutdown. Only bind this separate session, extent, plan and declared count.
    source=inspect.getsource(original.exercise)
    source=replace_once(source,'domain=178','domain=fixture_domain')
    source=replace_once(source,'g.info.width=160;g.info.height=160;g.info.origin.position.x=72.;g.info.origin.position.y=22.',
                        'g.info.width=grid_width;g.info.height=grid_height;g.info.origin.position.x=grid_x;g.info.origin.position.y=grid_y')
    source=replace_once(source,'ix=int((bx-72.)/.15);iy=int((by-22.)/.15)','ix=int((bx-grid_x)/.15);iy=int((by-grid_y)/.15)')
    source=replace_once(source,'val[iy0*160+ix0]=100','val[iy0*grid_width+ix0]=100')
    source=replace_once(source,'two_exact_gears=len(results)==2','all_declared_gears=len(results)==len(plan["segments"])')
    object_scope={**original.exercise.__globals__,'checked_plan':lambda:plan,'TRANSFER':path,
                  'fixture_domain':domain,'grid_x':x0,'grid_y':y0,'grid_width':width,'grid_height':height,'__file__':__file__}
    exec(compile(source,str(Path(original.__file__).resolve()),'exec'),object_scope)
    result=object_scope['exercise']()
    result.update(input_filename=path.name,declared_gears=[s['id'] for s in segments],
                  original_private_harness_sha256=sha(Path(original.__file__)),input_revision=payload.get('revision'),
                  gear_split_sha256=sha(Path(inspect.getfile(gear_runs))),fixture_grid_origin=[x0,y0],fixture_grid_shape=[width,height])
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan',type=Path);parser.add_argument('--domain',type=int,default=179);parser.add_argument('--out',type=Path)
    parser.add_argument('--gear-prefix',help='Split a points array with the original signed gear algorithm and this ID prefix')
    args=parser.parse_args();report=run_file(args.plan,args.domain,args.gear_prefix);text=json.dumps(report,indent=2)+'\n'
    if args.out:args.out.write_text(text)
    print(text,end='');raise SystemExit(0 if report['passed'] else 2)
