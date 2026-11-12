"""S13-only loopback discovery, without the RMW LOCALHOST 32-peer cap."""
from pathlib import Path
def local_dds(environment):
    env=environment.copy()
    profile=str(Path(__file__).resolve().parents[1]/'config/fastdds_local.xml')
    env.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp',ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT',ROS_STATIC_PEERS='',FASTDDS_DEFAULT_PROFILES_FILE=profile,FASTRTPS_DEFAULT_PROFILES_FILE=profile)
    env.pop('ROS_LOCALHOST_ONLY',None)
    return env
