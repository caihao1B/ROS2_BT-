import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from transport_video_r12 import transport_environment
from transport_r08 import transport_environment as original_environment


class VideoTransport(unittest.TestCase):
 def test_only_large_observer_writer_changes(self):
  env=dict(ROS_DOMAIN_ID='120',R08_CLOUD_TRANSPORT_PROFILE='/session/fastdds.xml')
  for role in ['controller_server','bt','tool_server','ledger','reference','video_recorder','raw_bridge','mapping_inputs']:
   with self.subTest(role=role):self.assertEqual(transport_environment(role,env),original_environment(role,env))
  result=transport_environment('video_bridge',env)
  self.assertEqual(result['FASTRTPS_DEFAULT_PROFILES_FILE'],env['R08_CLOUD_TRANSPORT_PROFILE'])
  self.assertEqual(result['ROS_DOMAIN_ID'],'120');self.assertNotIn('FASTRTPS_DEFAULT_PROFILES_FILE',env)
