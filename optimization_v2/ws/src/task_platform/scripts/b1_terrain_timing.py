"""Construct an isolated B1 terrain executable with indexed neighbors and per-scan calibration."""
import re
import subprocess
from pathlib import Path
from contextlib import contextmanager

from common import APP, PKG
from mowing_yaw_candidate_r10 import replace_once
import terrain_pairing_mowing_r10 as paired
from terrain_pairing_r10 import write_changed

BUILD=APP/'build/b1_indexed_terrain'
SOURCE=APP/'build/b1_indexed_terrain_source'
TARGET='b1_indexed_terrain'
BINARY=BUILD/TARGET
original_build=paired.build
original_bindings=paired.bindings


def indexed_source(original):
    # Both the grid origin and the qualified height map are fixed for this callback.
    # Copy the presence separately: unknown, zero and non-finite are distinct.
    original=replace_once(original,'  J rows=J::array();int lethal=0;', '''  std::vector<double> heights(cells_.size());
  std::vector<uint8_t> observed(cells_.size(), 0);
  for (const auto& [index, height] : h) {
    heights[index] = height;
    observed[index] = 1;
  }
  J rows=J::array();int lethal=0;''')
    original=replace_once(original,'if(h.count(j)&&std::abs(h[j]-h[i])<.25)near.push_back(center(j,h[j]));',
                          'if(observed[j]&&std::abs(heights[j]-heights[i])<.25)near.push_back(center(j,heights[j]));')
    begin=original.index(' void cloud(const sensor_msgs::msg::PointCloud2&m){')
    end=original.index('\npublic:',begin)
    code=original[begin:end]
    load=['  // Read unchanged calibration once per scan, outside the point and cell loops.']
    for name in sorted(set(re.findall(r'val\("([a-z_0-9]+)"\)',code))):
        expression=f'val("{name}")'
        code=code.replace(expression,'param_'+name)
        load.append(f'  const double param_{name} = {expression};')
    expression='V(cfg_["lidar_xyz"][0],cfg_["lidar_xyz"][1],cfg_["lidar_xyz"][2])'
    code=replace_once(code,expression,'lidar_translation')
    load.append('  const V lidar_translation = '+expression+';')
    expression='cfg_["clear_frames"].get<int>()'
    code=code.replace(expression,'clear_frames')
    load.append('  const int clear_frames = '+expression+';')
    code=replace_once(code,'  auto begin=', '\n'.join(load)+'\n  auto begin=')
    return original[:begin]+code+original[end:]


def sources():
    return [Path(__file__).resolve(),SOURCE/'terrain.cpp',SOURCE/'CMakeLists.txt',BINARY,*original_bindings()]


def build():
    original_build()
    SOURCE.mkdir(parents=True,exist_ok=True)
    write_changed(SOURCE/'terrain.cpp',indexed_source((paired.OUT/'terrain.cpp').read_text()))
    write_changed(SOURCE/'CMakeLists.txt',(paired.OUT/'CMakeLists.txt').read_text().replace(paired.BINARY.name,TARGET))
    subprocess.run(['cmake','-S',str(SOURCE),'-B',str(BUILD)],check=True,stdout=subprocess.DEVNULL)
    subprocess.run(['cmake','--build',str(BUILD),'--target',TARGET,'--parallel','1'],check=True)
    return BINARY


@contextmanager
def for_full_candidate():
    # The frozen runtime looks up the terrain provider while composing its launcher.
    # A fresh candidate CLI process owns this override and restores it before exit.
    old_build, old_bindings = paired.build, paired.bindings
    paired.build, paired.bindings = build, sources
    try:
        yield
    finally:
        paired.build, paired.bindings = old_build, old_bindings


if __name__=='__main__':
    print(build())
