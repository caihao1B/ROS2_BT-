"""Compare the complete original/indexed terrain callbacks on saved, held-out combined LiDAR clouds."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
from tempfile import TemporaryDirectory

import numpy as np
from common import APP, PKG, ROOT, dump, read, sha
from b1_terrain_timing import SOURCE, build, paired, write_changed
from mowing_yaw_candidate_r10 import replace_once

HARNESS=PKG/'r10_backend/b1_terrain_replay.inc'


def instrument(source):
    start=source.index('int main(int argc')
    source=source[:start]+'\n#include "'+str(HARNESS)+'"\n'
    source=replace_once(source,'class Terrain:public rclcpp::Node{','''class Terrain:public rclcpp::Node{
public:
 std::vector<int8_t> replay_bytes;
 J replay_messages = J::object();
 void replay_grid(const nav_msgs::msg::OccupancyGrid& grid) {
   replay_bytes.insert(replay_bytes.end(), grid.data.begin(), grid.data.end());
 }''')
    source=replace_once(source,'grids_[k]->publish(grid);for(', 'replay_grid(grid);grids_[k]->publish(grid);for(')
    source=replace_once(source,'confirmed_grids_[k]->publish(grid);','replay_grid(grid);confirmed_grids_[k]->publish(grid);')
    source=replace_once(source,'m.data=d.dump();p->publish(m);','''m.data=d.dump();p->publish(m);
   J invariant=d; invariant.erase("compute_ms");
   replay_messages[p==health_?"health":p==objects_?"objects":"cells"]=invariant;
''')
    return source


def replay_binaries():
    build()
    binaries={}
    for name,directory,actual in (('original',paired.OUT,paired.BINARY.name),('indexed',SOURCE,'b1_indexed_terrain')):
        target='replay_b1_terrain_'+name
        src=APP/'build'/(target+'_source')
        dest=APP/'build'/target
        src.mkdir(parents=True,exist_ok=True)
        write_changed(src/'terrain.cpp',instrument((directory/'terrain.cpp').read_text()))
        write_changed(src/'CMakeLists.txt',(directory/'CMakeLists.txt').read_text().replace(actual,target))
        subprocess.run(['cmake','-S',str(src),'-B',str(dest)],stdout=subprocess.DEVNULL,check=True)
        subprocess.run(['cmake','--build',str(dest),'--target',target,'--parallel','1'],check=True)
        binaries[name]=dest/target
    return binaries


def raw_inputs(captures):
    frames=[]
    for path in captures:
        with np.load(path,allow_pickle=False) as data:
            cloud=data['combined'].astype('<f4')
        meta=read(path.with_suffix('.json'))
        info=dict(npz=str(path.relative_to(ROOT)),sha256=sha(path),metadata_sha256=sha(path.with_suffix('.json')),
                  xyz_sha256=hashlib.sha256(cloud.tobytes()).hexdigest(),points=len(cloud),
                  original_sensor_stamp=meta['combined_t'],planar_odom=meta['odom_pose'])
        frames.append((cloud,info))
    return frames


def archive_inputs(path, frames):
    metadata=json.dumps([info for _,info in frames],sort_keys=True)
    inputs={f'cloud_{i}':cloud for i,(cloud,_) in enumerate(frames)}
    np.savez_compressed(path,metadata=np.array(metadata),**inputs)


def read_archive(path):
    with np.load(path,allow_pickle=False) as data:
        result=[]
        for i,info in enumerate(json.loads(str(data['metadata']))):
            cloud=data[f'cloud_{i}']
            if hashlib.sha256(cloud.tobytes()).hexdigest()!=info['xyz_sha256'] or len(cloud)!=info['points']:
                raise ValueError('Archived input does not match its trace binding')
            result.append((cloud,info))
    return result


def evaluate(frames, repeats=2):
    binaries=replay_binaries()
    fixtures=[]
    with TemporaryDirectory() as temporary:
        work=Path(temporary)
        manifest=[]
        tick=100
        for i,(raw,info) in enumerate(frames):
            if raw.ndim!=2 or raw.shape[1]!=3 or raw.dtype!=np.dtype('<f4'):
                raise ValueError('Expected captured little-endian Float32 XYZ clouds')
            data=work/f'{i}.bin'
            raw.tofile(data)
            fixtures.append(info)
            # Repeated measured scan + pose allow the existing temporal ground
            # confirmation logic to run. Both variants see identical 10 Hz timing.
            for _ in range(3):
                manifest.append(dict(path=str(data),points=len(raw),odom=info['planar_odom'],ns=tick*100000000))
                tick+=1
            tick+=20
        dump(work/'trace.json',manifest)
        results=[]
        for i in range(repeats):
            order=['original','indexed'] if i%2==0 else ['indexed','original']
            for name in order:
                path=work/f'{name}-{i}.bin'
                env=dict(os.environ,ROS_DOMAIN_ID='182',ROS_LOCALHOST_ONLY='1',
                    B1_REPLAY_INPUT=str(work/'trace.json'),B1_REPLAY_OUTPUT=str(path))
                process=subprocess.run([str(binaries[name]),'--ros-args','-p',
                    'config_file:='+str(APP/'editions/R10/mowing/terrain.json')],env=env,capture_output=True,
                    timeout=180,check=True,text=True)
                measured=[json.loads(line) for line in process.stdout.splitlines() if line.startswith('{')]
                if len(measured)!=len(manifest) or not all(row['valid'] for row in measured):
                    raise ValueError('Replay did not produce valid terrain data for every capture')
                results.append(dict(variant=name,round=i,ms=[r['ms'] for r in measured],output_bytes=path.stat().st_size,
                                    output_sha256=sha(path),grid_bytes=sum(r['published_grid_bytes'] for r in measured)))
        first=work/'original-0.bin'
        identical=all(first.read_bytes()==(work/f'{r["variant"]}-{r["round"]}.bin').read_bytes() for r in results)
    median={name:statistics.median(x for r in results if r['variant']==name for x in r['ms']) for name in binaries}
    return dict(schema='B1-TERRAIN-INDEPENDENT-REPLAY/1',passed=identical,
                fixtures=fixtures,calls_per_round=len(manifest),rounds=repeats,results=results,
                identical_risk_grids_and_geometry=identical,callback_median_ms=median,
                median_wall_reduction_percent=100*(1-median['indexed']/median['original']),
                scope='Offline process invocation on retained combined LiDAR arrays and recorded planar odometry. '
                'Three repeats of each input at synthetic 10 Hz exercise ground confirmation. '
                'Compares raw bytes of all four risk grids and identical normalized geometry/terrain/health JSON '
                'except timing. Callback timing includes identical instrumentation. No vehicle or live success measured.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    inputs=p.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--captures',type=Path,nargs='+')
    inputs.add_argument('--bundle',type=Path)
    p.add_argument('--export-inputs',type=Path)
    args=p.parse_args()
    frames=raw_inputs([x.resolve() for x in args.captures]) if args.captures else read_archive(args.bundle)
    if args.export_inputs:
        archive_inputs(args.export_inputs,frames)
    result=evaluate(frames)
    if args.bundle or args.export_inputs:
        result['input_bundle_sha256']=sha(args.bundle or args.export_inputs)
    dump(args.output,result)
    print(json.dumps({k:result[k] for k in ['passed','callback_median_ms','median_wall_reduction_percent']}))
    raise SystemExit(0 if result['passed'] else 2)
