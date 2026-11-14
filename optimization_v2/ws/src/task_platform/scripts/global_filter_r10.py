"""Isolated global manager derivation: original raw guards, then correction smoothing."""
from common import *

def derive(source):
    marker='void tick(){double t=now().seconds();'
    assert source.count(marker)==1
    source=source.replace(marker,marker+'double correction_dt=t-last_clock_;')
    marker='else correction_=next;}'
    assert source.count(marker)==1
    source=source.replace(marker,'else correction_=r10::smoothCorrection(correction_,next,correction_dt,c("correction_filter_tau_s"));}')
    source=source.replace('{"correction",values(correction_)}','{"correction",values(correction_)},{"raw_map_correction",values(map_)},{"correction_filter_tau_s",c("correction_filter_tau_s")}')
    return '#include "'+str(PKG/'r10_backend/correction_filter.hpp')+'"\n'+source

def build():
    out=APP/'build/r10_global_source';out.mkdir(parents=True,exist_ok=True)
    (out/'global_manager.cpp').write_text(derive((APP/'build/r08_source/global_manager.cpp').read_text()))
    cmake='''cmake_minimum_required(VERSION 3.16)
project(r10_global)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O2 -Wall -Wextra)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
'''
    deps='rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2_ros'
    cmake+=''.join('find_package('+p+' REQUIRED)\n' for p in deps.split())
    cmake+='add_executable(r10_global_manager global_manager.cpp)\nament_target_dependencies(r10_global_manager '+deps+')\ntarget_link_libraries(r10_global_manager Eigen3::Eigen nlohmann_json::nlohmann_json)\n'
    (out/'CMakeLists.txt').write_text(cmake)
    subprocess.run(['cmake','-S',str(out),'-B',str(APP/'build/r10_global')],check=True)
    subprocess.run(['cmake','--build',str(APP/'build/r10_global'),'--parallel','2'],check=True)
