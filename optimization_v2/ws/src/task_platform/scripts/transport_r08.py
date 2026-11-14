"""Per-session Fast DDS buffers for real organized clouds, no global system edits."""
PROFILE='''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
 <transport_descriptors>
  <transport_descriptor><transport_id>task_shm</transport_id><type>SHM</type><segment_size>67108864</segment_size><maxMessageSize>8388608</maxMessageSize></transport_descriptor>
  <transport_descriptor><transport_id>task_udp</transport_id><type>UDPv4</type></transport_descriptor>
 </transport_descriptors>
 <participant profile_name="task_cloud_transport" is_default_profile="true"><rtps>
  <userTransports><transport_id>task_shm</transport_id><transport_id>task_udp</transport_id></userTransports>
  <useBuiltinTransports>false</useBuiltinTransports>
 </rtps></participant>
</profiles>
'''
def configure_transport(env,out):
 p=out/'fastdds.xml';p.write_text(PROFILE)
 env['R08_CLOUD_TRANSPORT_PROFILE']=str(p)
 # Keep lifecycle/action/control participants on their qualified default
 # transport. Only actual large-cloud writers need a large send segment.
 for key in ['FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE']:env.pop(key,None)

CLOUD_WRITERS={'raw_bridge','generator_vision','fanout','mapping_inputs'}
def transport_environment(role,inherited):
 env=dict(inherited)
 if role in CLOUD_WRITERS:
  profile=env['R08_CLOUD_TRANSPORT_PROFILE']
  env.update(FASTRTPS_DEFAULT_PROFILES_FILE=profile,FASTDDS_DEFAULT_PROFILES_FILE=profile)
 return env
