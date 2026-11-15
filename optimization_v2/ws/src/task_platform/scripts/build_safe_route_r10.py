"""Build an isolated offline connector backend; does not touch running ROS binaries."""
from common import APP,PKG,ROOT,subprocess

def build():
    source=APP/'build/r10_safe_route_source';target=APP/'build/r10_safe_route';source.mkdir(parents=True,exist_ok=True)
    cmake=(PKG/'r03_backend/CMakeLists.txt').read_text().replace('r03_path_backend','r10_safe_route_backend').replace('r03_reeds_shepp reeds_shepp.cpp','r10_safe_route safe_route.cpp').replace('target_include_directories(r03_reeds_shepp','target_include_directories(r10_safe_route').replace('target_link_libraries(r03_reeds_shepp','target_link_libraries(r10_safe_route')
    (source/'CMakeLists.txt').write_text(cmake);(source/'safe_route.cpp').write_bytes((PKG/'r10_backend/safe_route.cpp').read_bytes())
    subprocess.run(['cmake','-S',str(source),'-B',str(target),'-DCMAKE_BUILD_TYPE=Release'],check=True,cwd=ROOT)
    subprocess.run(['cmake','--build',str(target),'--parallel','1'],check=True,cwd=ROOT)
    return target/'r10_safe_route'

if __name__=='__main__':build()
