"""Build an owned tool bridge around installed converters; never replace exit codes."""
from common import *
def build():
 source=APP/'build/r10_bridge_source';source.mkdir(parents=True,exist_ok=True)
 cmake='''cmake_minimum_required(VERSION 3.16)
project(r10_bridge)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
set(Protobuf_PROTOC_EXECUTABLE /usr/bin/protoc CACHE FILEPATH "Gazebo ABI" FORCE)
find_package(Protobuf MODULE REQUIRED)
find_package(ros_gz_bridge REQUIRED)
'''
 cmake+='add_executable(r10_tool_bridge '+str(PKG/'r10_backend/tool_bridge.cpp')+')\nament_target_dependencies(r10_tool_bridge rclcpp ros_gz_bridge)\n'
 (source/'CMakeLists.txt').write_text(cmake);target=APP/'build/r10_bridge'
 subprocess.run(['cmake','-S',str(source),'-B',str(target)],check=True);subprocess.run(['cmake','--build',str(target),'--parallel','2'],check=True)
 return target/'r10_tool_bridge'
if __name__=='__main__':build()
