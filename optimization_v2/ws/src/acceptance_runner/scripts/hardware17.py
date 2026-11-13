"""Read actual host identifiers and tool versions without modifying the host."""
from common17 import *
import platform,subprocess

def collect():
    result=dict(platform=platform.platform(),cpu_count=os.cpu_count(),memory_total_bytes=os.sysconf('SC_PAGE_SIZE')*os.sysconf('SC_PHYS_PAGES'),ros_distro=os.environ.get('ROS_DISTRO'),wall_started=time.time())
    for key,args in [('cpu',['lscpu','-J']),('gpu',['nvidia-smi','--query-gpu=name,memory.total,driver_version','--format=csv,noheader']),('gazebo',['gz','sim','--force-version','8','--versions'])]:
        try:
            p=subprocess.run(args,capture_output=True,text=True,timeout=8);result[key]=dict(exit_code=p.returncode,stdout=p.stdout.strip(),stderr=p.stderr.strip())
        except Exception as e:result[key]=dict(error=str(e))
    return result
