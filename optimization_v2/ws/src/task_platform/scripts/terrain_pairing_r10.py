"""Build an isolated A1 terrain executable from the original R08 geometry source."""
import subprocess
from common import APP, PKG
from spray_defer_fixture_r10 import once

SOURCE=APP/'build/r08_source/terrain.cpp'
CMAKE=SOURCE.with_name('CMakeLists.txt')
INCLUDE=PKG/'r10_backend/terrain_pairing.inc'
OUT=APP/'build/spray_terrain_pairing_source'
BINARY=APP/'build/spray_terrain_pairing/r10_terrain_pairing'


def write_changed(path,data):
    if not path.exists() or path.read_text()!=data:
        path.write_text(data)


def build():
    OUT.mkdir(parents=True,exist_ok=True)
    src=once(SOURCE.read_text(), ' void cloud(const sensor_msgs::msg::PointCloud2&m){',
             '#include "'+str(INCLUDE)+'"\n void cloud(const sensor_msgs::msg::PointCloud2&m){')
    src=once(src,'while(odom_.size()>250)odom_.pop_front();','while(odom_.size()>250)odom_.pop_front();try_pair();')
    begin=src.index('  cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>')
    finish=src.index('\n',begin)
    original=src[begin:finish]
    src=once(src,original,'''  pairing_timer_=create_wall_timer(std::chrono::milliseconds(8),[this](){try_pair();});
  cloud_sub_=create_subscription<Cloud>("/s04/perception/points",rclcpp::SensorDataQoS(),[this](Cloud::SharedPtr m){receive_cloud(std::move(m));});''')
    src=once(src,'{"odom_delta_s",dt}', '{"odom_delta_s",dt},{"pair_wait_sim_s",pair_wait_}')
    write_changed(OUT/'terrain.cpp',src)
    cmake=CMAKE.read_text().split('add_executable(r08_policy_test')[0].replace('r08_terrain','r10_terrain_pairing')
    write_changed(OUT/'CMakeLists.txt',cmake)
    binary_dir=BINARY.parent
    subprocess.run(['cmake','-S',str(OUT),'-B',str(binary_dir)],check=True,stdout=subprocess.DEVNULL)
    subprocess.run(['cmake','--build',str(binary_dir),'--target',BINARY.name,'-j1'],check=True)
    return BINARY


if __name__=='__main__':
    print(build())
