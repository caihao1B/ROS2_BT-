"""Configuration-derived collision envelope, independent of scene obstacles.

Full 3D localization is deliberately not filtered. This is only the vertical
extent relevant to ground vehicle collision projection, relative to ground.
"""
from common14 import BASE, PKG, yaml, math

INPUTS = [BASE/'config/sensors.yaml', BASE/'config/tool_profiles.yaml',
          BASE/'config/traversability.yaml'] + [BASE/f'config/vehicles/{k}.yaml' for k in ['diff','ackermann']]

def envelope():
    sensors,tools,terrain,diff,ack = [yaml.safe_load(p.read_text()) for p in INPUTS]
    # Conservative mounting allowance, including the virtual sensor housing.
    # A future taller tool/mast must regenerate the bound map before motion.
    mounting_allowance = .10
    rows = {}
    for kind,vehicle,tool in [('diff',diff,tools['spray']),('ackermann',ack,tools['mower'])]:
        tops = [vehicle['body']['center'][2]+vehicle['body']['size'][2]/2,
                2*vehicle['wheel']['radius'], tool['mount_xyz'][2]+tool['collision_size'][2]/2]
        tops += [s['xyz'][2]+mounting_allowance for s in sensors.values() if isinstance(s,dict) and 'xyz' in s]
        radius=max(math.hypot(*p) for p in vehicle['footprint'])
        slope=terrain['profiles'][kind]['max_slope_deg']
        uncertainty=3*terrain['terrain_height_sigma_m']+terrain['ground_distance_m']
        top=max(tops); tilt=radius*math.sin(math.radians(slope))
        rows[kind]=dict(body_and_payload_top_m=top,tilt_allowance_m=tilt,ground_uncertainty_m=uncertainty,
                        ceiling_m=math.ceil((top+tilt+uncertainty)*20)/20)
    return dict(ceiling_m=max(r['ceiling_m'] for r in rows.values()),vehicles=rows,
                mounting_allowance_m=mounting_allowance,scope='ground-relative collision projection only; full SLAM cloud retained')

def hazardous(normal_distance, normal_z, ceiling):
    # Preserve the original low-obstacle threshold and negative terrain hits.
    return (abs(normal_distance)>.045) & (normal_distance/normal_z<=ceiling)
