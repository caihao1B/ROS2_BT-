"""Build the R09 Ackermann terrain node with the qualified short sensor-pairing queue."""
import subprocess
from pathlib import Path
from common import APP,PKG,ROOT,sha
from spray_defer_fixture_r10 import once
from spray_precision_session_r10 import terrain_version
from terrain_pairing_r10 import write_changed

SOURCE=APP/'build/r09_source/terrain.cpp'
CMAKE=SOURCE.with_name('CMakeLists.txt')
OUT=APP/'build/mowing_terrain_pairing_source'
BINARY=APP/'build/mowing_terrain_pairing/r10_mowing_terrain_pairing'


def build():
    _,session=terrain_version()
    OUT.mkdir(parents=True,exist_ok=True)
    include=OUT/'terrain_pairing_session.inc'
    write_changed(include,session.INCLUDE.read_text())
    src=SOURCE.read_text()
    if 'r09_backend/self_filter.hpp' not in src or 's14::riskAt(c.cost[1]' not in src:
        raise ValueError('Expected the Ackermann R09 geometry source')
    marker=' void cloud(const sensor_msgs::msg::PointCloud2&m){'
    src=once(src,marker,'#include "'+str(include)+'"\n'+marker)
    src=once(src,'while(odom_.size()>250)odom_.pop_front();','while(odom_.size()>250)odom_.pop_front();try_pair();')
    begin=src.index('  cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>')
    end=src.index('\n',begin)
    src=once(src,src[begin:end],'''  pairing_timer_=create_wall_timer(std::chrono::milliseconds(8),[this](){try_pair();});
  cloud_sub_=create_subscription<Cloud>("/s04/perception/points",rclcpp::SensorDataQoS(),[this](Cloud::SharedPtr m){receive_cloud(std::move(m));});''')
    src=once(src,'{"odom_delta_s",dt}','{"odom_delta_s",dt},{"pair_wait_sim_s",pair_wait_},{"paired_after_defer",pending_deferred_}')
    write_changed(OUT/'terrain.cpp',src)
    cmake=CMAKE.read_text().split('add_executable(r09_self_filter_test')[0]
    write_changed(OUT/'CMakeLists.txt',cmake.replace('r09_terrain',BINARY.name))
    subprocess.run(['cmake','-S',str(OUT),'-B',str(BINARY.parent)],check=True,stdout=subprocess.DEVNULL)
    subprocess.run(['cmake','--build',str(BINARY.parent),'--target',BINARY.name,'-j1'],check=True)
    return BINARY


def bindings():
    return [Path(__file__).resolve(),PKG/'scripts/terrain_pairing_r10.py',PKG/'scripts/spray_precision_session_r10.py',
            PKG/'r10_backend/terrain_pairing.inc',PKG/'r09_backend/self_filter.hpp',SOURCE,CMAKE,
            OUT/'terrain.cpp',OUT/'CMakeLists.txt',OUT/'terrain_pairing_session.inc',BINARY]


if __name__=='__main__':
    print(build())
    print({str(p.relative_to(ROOT)):sha(p) for p in bindings()})
