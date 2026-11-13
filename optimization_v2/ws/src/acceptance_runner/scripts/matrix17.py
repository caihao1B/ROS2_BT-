"""Freeze requirements before test; never infer case success from its name."""
from common17 import *
import yaml

def make():
    seeds = dict(schema='S17-SEEDS/1', root_seed=41,
                 formal=[41001+i*97 for i in range(10)],
                 qualification=[17001+i for i in range(24)],
                 paired=[42001,42098,42195],
                 policy='Disjoint from S15/S16 training/validation; no selection on formal outcome.')
    matrix = dict(schema='S17-MATRIX/1', frozen_seed_root=41,
        scope='All six original S12 resized task domains; does not claim coverage of out_of_scope large-field residual.',
        simulation_limit_s=1800, nominal_plan_limit_s=1440,
        prediction_default='no_prediction', memory_default=False,
        thresholds=dict(position_p95_m=.15,yaw_p95_rad=.05,spray_fraction=1.,mow_fraction=.95,obstacle_contacts=0),
        formal=[dict(id=f'{c}_seed{s}',cluster=c,seed=s,vehicle='diff' if c[0]=='A' else 'ackermann',controller='DWB' if c[0]=='A' else 'RPP',geometry='base',fault='none',scope='FULL_CLUSTER',requirement=['R03','R04','R05','R06','R08']) for s in seeds['formal'] for c in ['A1','A2','A3','B1','B2','B3']],
        geometries=['base','notch','hole'],
        connector=[dict(id=f'connector_{k}',vehicle=k,requirement=['R01','R03']) for k in ['diff','ackermann']],
        faults=[dict(id=f'fault_{f}',fault=f,seed=seeds['paired'][0],requirement=['R04','R05','R07']) for f in ['dynamic_crossing','persistent_block','low_obstacle','slope','gnss_false_fixed','gnss_occlusion','localization_loss','sensor_delay','tool_loss','mission_restart']],
        ablations=[dict(id='partition',groups=['kmeans_repaired','darp','metis_time'],level='offline_geometry_and_time'),
                   dict(id='controller_diff',groups=['DWB','RPP','MPPI'],level='paired_closed_loop'),
                   dict(id='controller_ackermann',groups=['RPP','MPPI'],level='paired_closed_loop'),
                   dict(id='localization',groups=['wheel_imu','lidar','visual','gnss'],level='paired_closed_loop'),
                   dict(id='prediction',groups=['none','imm','learned'],level='paired_closed_loop'),
                   dict(id='memory',groups=['full','event','cache'],level='paired_closed_loop'),
                   dict(id='coverage',groups=['offline_repair','pure_online'],level='paired_closed_loop')],
        requirements=dict(R01='Continuous irregular geometry, holes/notches and two chassis connector inspection',R02='Three connected workload clusters with paired algorithm comparison',R03='Footprint, curvature, speed and nominal time',R04='Independent localization errors and failure stop',R05='Feedback-only idempotent work ledger and original residual',R06='60 complete cluster trials; stop included in 1800 seconds',R07='Observed injected faults and safe bounded response',R08='Hashes, fixed seeds, resumable nonduplicating score and hardware/RTF'))
    return seeds,matrix

def freeze():
    seeds,matrix=make()
    for path,value in [(EVAL/'seeds.json',seeds),(EVAL/'acceptance_matrix.yaml',matrix)]:
        if path.exists():
            old=yaml.safe_load(path.read_text())
            if old!=value: raise ValueError('FROZEN_MATRIX_CHANGED '+str(path))
        elif path.suffix=='.json':dump(path,value)
        else:path.parent.mkdir(parents=True,exist_ok=True);path.write_text(yaml.safe_dump(value,sort_keys=False,allow_unicode=True))
    return seeds,matrix
if __name__=='__main__': freeze()
