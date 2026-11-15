import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from highlights_r12 import select

class ActualRejoinExcerpt(unittest.TestCase):
 def test_endpoint_excerpt_needs_same_segment_actual_recovery_motion(self):
  frames=[dict(sim_t=i,video_s=i*3.) for i in range(101)]
  stall=dict(t=20,event='ENDPOINT_POSE_STALL',segment='turn1')
  done=dict(t=50,event='ENDPOINT_RECOVERY_SUCCESS',segment='turn1')
  self.assertIsNone(select('mowing_full_B1',[stall,done],frames,{}))
  gear=dict(t=25,event='RECOVERY_GEAR_ARMED')
  self.assertIn('有界恢复',select('mowing_full_B1',[stall,gear,done],frames,{})['label'])
  done['segment']='turn2'
  self.assertIsNone(select('mowing_full_B1',[stall,gear,done],frames,{}))
 def test_spray_recovery_request_without_resume_is_not_success(self):
  frames=[dict(sim_t=i,video_s=i*2.) for i in range(101)]
  request=dict(t=20,event='STOPPED_RELOCALIZATION_REQUEST',point='p1')
  self.assertIsNone(select('spray_full_A1',[request],frames,{}))
  resumed=dict(t=30,event='LOCALIZATION_REVALIDATED_RESUME',point='p1')
  excerpt=select('spray_full_A1',[request,resumed],frames,{})
  self.assertEqual(excerpt['start_sim'],16);self.assertEqual(excerpt['end_sim'],35)
  self.assertIn('路径重验',excerpt['label'])
 def test_spray_docking_requires_same_point_real_work(self):
  frames=[dict(sim_t=i,video_s=i*2.) for i in range(101)]
  dock=dict(t=20,event='LOCAL_DOCK_STOP_FOR_GLOBAL_OBSERVATION',point='p1')
  done=dict(t=30,event='TOOL_RESULT',point='p2',state='SUCCEEDED',replayed=False)
  self.assertIsNone(select('spray_full_A1',[dock,done],frames,{}))
  done['point']='p1';done['replayed']=True
  self.assertIsNone(select('spray_full_A1',[dock,done],frames,{}))
  done['replayed']=False
  self.assertIn('工具反馈',select('spray_full_A1',[dock,done],frames,{})['label'])
 def test_only_confirmed_rejoin_can_be_labeled(self):
  frames=[dict(sim_t=i,video_s=i*3.) for i in range(101)]
  search=dict(t=20,event='ORIGINAL_WORK_WINDOW_RECHECK',segment='strip12')
  self.assertIsNone(select('mowing_full_B1',[search],frames,{}))
  done=dict(t=60,event='COVERAGE_REJOIN_CONFIRMED',segment='strip12')
  excerpt=select('mowing_full_B1',[search,done],frames,{})
  self.assertEqual(excerpt['start_sim'],15);self.assertEqual(excerpt['end_sim'],66)
  self.assertIn('保留残余',excerpt['label'])
  self.assertGreater(excerpt['speed'],1)

if __name__=='__main__':unittest.main()
