"""Independent recorded-trajectory audit of the complete inspection obligation.

A list of completed action names is insufficient: real segment endpoints must
be reached in order, and the physical footprint must remain in the authorized
work/connector polygons. Truth is read only after the simulation has stopped.
"""
import math
import numpy as np
from shapely.geometry import shape, Polygon
from shapely.ops import unary_union
from common17 import BASE, read
import yaml

def audit(contract, truth, admitted, vehicle):
    trajectory=[r for r in truth if admitted is not None and r['t']>=admitted]
    segments=[s for s in contract.get('original_route',{}).get('segments',[]) if s['kind']!='ROTATE']
    if not trajectory or not segments:return dict(passed=False,reason='NO_TRAJECTORY_OR_ROUTE')
    xy=np.array([r['xyz'][:2] for r in trajectory]); cursor=0; endpoints=[]
    # Original navigation tolerance plus the independent localization target;
    # the controller's tighter goal check remains active during execution.
    tolerance=.025+.15
    for segment in segments:
        target=segment['poses'][-1][:2]; distances=np.linalg.norm(xy[cursor:]-target,axis=1)
        hits=np.flatnonzero(distances<=tolerance)
        if len(hits):cursor+=int(hits[0])
        endpoints.append(dict(target_world=target,reached=bool(len(hits)),minimum_error_m=float(distances.min()),time_s=trajectory[cursor]['t'] if len(hits) else None))
    features=[]
    for filename in ['workspaces.geojson','connector.geojson']:
        features += [shape(f['geometry']) for f in read(BASE/'maps/prior'/filename)['features'] if f['geometry']['type'] in ['Polygon','MultiPolygon']]
    features += [shape(f['geometry']) for f in read(BASE/'config/perception_test_area.geojson')['features']]
    allowed=unary_union(features)
    footprint=np.array(yaml.safe_load((BASE/'config/traversability.yaml').read_text())['profiles'][vehicle]['footprint'])
    outside=[]
    # Use every retained evaluator pose, without interpolated boundary credit.
    for row in trajectory:
        h=row['rpy'][2];rotation=np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])
        polygon=Polygon(footprint@rotation.T+row['xyz'][:2]); area=polygon.difference(allowed).area
        if area>1e-9:outside.append(dict(t=row['t'],outside_area_m2=area))
    return dict(passed=all(e['reached'] for e in endpoints) and not outside,endpoint_tolerance_m=tolerance,endpoints=endpoints,footprint_samples=len(trajectory),outside_samples=len(outside),maximum_outside_area_m2=max((r['outside_area_m2'] for r in outside),default=0.),first_outside=outside[:1],scope='ordered real segment endpoints and physical footprint in authorized polygons; temporary chunk goals earn no work credit')
