import sys,unittest,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from follow18 import LeaderFollower

def track(t=1.,**kw):
    return dict(id=7,observation_t=t,position=[3.2,0.,.5],velocity=[.06,0.],size=[.5,.5,1.],
        covariance=[.0001,0,0,0,0,.0001,0,0,0,0,.01,0,0,0,0,.01],
        state='OBSERVED',confirmed=True,frame='odom',**kw)
EGO=dict(pose=[0.,0.,0.],speed=.08,footprint=[[-.3,-.52],[1.3,-.52],[1.3,.52],[-.3,.52]],frame='odom')
class Following(unittest.TestCase):
    def test_same_direction_and_measured_body_gap(self):
        f=LeaderFollower();r=f.update(1.,EGO,[track()],observations_valid=True)
        self.assertEqual(r['state'],'FOLLOW_LEADER');self.assertAlmostEqual(r['gap_m'],1.63)
        self.assertLessEqual(r['speed_cap_mps'],.12)
    def test_crossing_object_not_selected_as_leader(self):
        x=track();x['velocity']=[0.,.2]
        self.assertIsNone(LeaderFollower().update(1.,EGO,[x],observations_valid=True)['selected_id'])
    def test_stationary_leader_causes_wait(self):
        f=LeaderFollower();f.update(1.,EGO,[track()],observations_valid=True)
        x=track(1.1);x['velocity']=[0.,0.];x['position'][0]=2.9
        self.assertEqual(f.update(1.1,EGO,[x],observations_valid=True)['state'],'WAIT_LEADER')
    def test_lost_leader_never_replaced_by_other_id(self):
        f=LeaderFollower();f.update(1.,EGO,[track()],observations_valid=True)
        x=track(1.1);x['id']=8
        r=f.update(1.1,EGO,[x],observations_valid=True)
        self.assertEqual(r['state'],'TARGET_UNCERTAIN_STOP');self.assertEqual(r['selected_id'],7)
    def test_stale_ambiguous_and_invalid_observation_stop(self):
        for mode in ['stale','ambiguous','observation']:
            f=LeaderFollower();f.update(1.,EGO,[track()],observations_valid=True);x=track(1.5)
            if mode=='stale':x['observation_t']=1.
            if mode=='ambiguous':x['state']='AMBIGUOUS_COAST'
            self.assertEqual(f.update(1.5,EGO,[x],observations_valid=mode!='observation')['speed_cap_mps'],0.)
    def test_clearance_requires_hysteresis_and_observation(self):
        f=LeaderFollower();f.update(1.,EGO,[track()],observations_valid=True)
        x=track(2.);x['position'][1]=3.
        self.assertEqual(f.update(2.,EGO,[x],observations_valid=True)['state'],'CLEARANCE_CHECK_REQUIRED')
        x['observation_t']=2.1
        self.assertEqual(f.update(2.1,EGO,[x],observations_valid=True,path_clear=True)['state'],'CLEAR_HYSTERESIS_WAIT')
        x['observation_t']=3.2
        self.assertEqual(f.update(3.2,EGO,[x],observations_valid=True,path_clear=True)['state'],'RESUME_TASK')
    def test_too_close_and_clock_reset_stop(self):
        f=LeaderFollower();x=track();x['position'][0]=2.
        self.assertEqual(f.update(1.,EGO,[x],observations_valid=True)['speed_cap_mps'],0.)
        self.assertEqual(f.update(.5,EGO,[x],observations_valid=True)['state'],'CLOCK_INVALID_STOP')

    def test_bad_footprint_or_covariance_stops(self):
        e=copy.deepcopy(EGO);e['footprint']=[[float('nan'),0.]]
        self.assertEqual(LeaderFollower().update(1.,e,[track()],observations_valid=True)['speed_cap_mps'],0.)
        x=track();x['covariance'][0]=-.1
        self.assertEqual(LeaderFollower().update(1.,EGO,[x],observations_valid=True)['speed_cap_mps'],0.)
    def test_selected_leader_turning_across_stops(self):
        f=LeaderFollower();f.update(1.,EGO,[track()],observations_valid=True)
        x=track(1.1);x['velocity']=[.01,.2]
        self.assertEqual(f.update(1.1,EGO,[x],observations_valid=True)['state'],'LEADER_DIRECTION_CHANGED_STOP')

if __name__=='__main__':unittest.main()
