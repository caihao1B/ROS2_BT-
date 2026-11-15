"""Offline footprint reserve for raster collision checks and tracking variation.

This is a design envelope, not a new runtime accuracy gate or proof that every
future tracking error is bounded by the declared design perturbations.
"""
import math

def extrema(a,b,angle):
    angles=[-angle,angle]
    stationary=math.atan2(b,a)
    angles.extend(stationary+k*math.pi for k in range(-2,3) if -angle<=stationary+k*math.pi<=angle)
    values=[a*math.cos(t)+b*math.sin(t) for t in angles]
    return min(values),max(values)

def envelope(footprint,padding,resolution,xy_reserve=.25,yaw_reserve=.12):
    if not all(math.isfinite(x) and x>=0 for x in [padding,resolution,xy_reserve,yaw_reserve]) or yaw_reserve>math.pi/2:
        raise ValueError('INVALID_ENVELOPE_DESIGN_INPUT')
    xmin=min(x for x,y in footprint);xmax=max(x for x,y in footprint);half=max(abs(y) for x,y in footprint)
    corners=[(xmin-padding,-half-padding),(xmax+padding,-half-padding),(xmax+padding,half+padding),(xmin-padding,half+padding)]
    xb=[extrema(x,-y,yaw_reserve) for x,y in corners];yb=[extrema(y,x,yaw_reserve) for x,y in corners]
    raster=resolution*math.sqrt(2)/2;reserve=xy_reserve+raster
    low=min(x[0] for x in xb)-reserve;high=max(x[1] for x in xb)+reserve;wide=max(abs(v) for row in yb for v in row)+reserve
    # Keep the prior longitudinal stopping reserve; round lateral room outward.
    low=min(low,xmin-.7);high=max(high,xmax+.7)
    extra=math.ceil((wide-half-1e-12)/.05)*.05;wide=half+extra
    polygon=[[low,-wide],[high,-wide],[high,wide],[low,wide]]
    policy=dict(nav2_footprint_padding_m=padding,costmap_resolution_m=resolution,raster_half_diagonal_m=raster,position_design_reserve_m=xy_reserve,yaw_design_reserve_rad=yaw_reserve,minimum_longitudinal_extra_m=.7,lateral_extra_m=extra,scope='Offline padded footprint under declared position/yaw perturbations plus grid half-diagonal. Runtime collision, localization and arrival gates remain unchanged; actual tracking qualification is still required.')
    return polygon,policy
