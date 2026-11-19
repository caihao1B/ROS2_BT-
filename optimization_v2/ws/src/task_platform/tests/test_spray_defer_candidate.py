import copy
import json
from pathlib import Path
import sys
import unittest

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from spray_defer_policy_r10 import selected_points,exhausted_dock,defer_admission
from spray_defer_candidate_r10 import build_mission

class CandidateAdmission(unittest.TestCase):
    def test_defer_needs_two_executed_failed_motion_attempts(self):
        point='target';rows=[]
        for i in range(2):
            rows+= [dict(t=i*4,event='LOCAL_DOCK_GOAL',point=point,attempt=i,path_sha256='x'),dict(t=i*4+2,event='LOCAL_DOCK_NAV_FAILED',point=point)]
        rows.append(dict(t=9,event='DOCKING_SETTLED_CHECK',point=point,attempt=2,stable=False))
        self.assertTrue(exhausted_dock(rows,point,2)['eligible'])
        for clipped in (rows[:-1],rows[1:],rows[:1]+rows[2:]):self.assertFalse(exhausted_dock(clipped,point,2)['eligible'])
    def test_candidate_preserves_original_order(self):
        originals=[dict(id=str(i)) for i in range(6)];self.assertEqual(selected_points(originals,['2','3']),originals[2:4]);
        with self.assertRaises(ValueError):selected_points(originals,['2','4'])
        self.assertIn('PRECISION_DOCK',build_mission().read_text())
    def test_no_permission_after_missing_modalities_or_without_native_dispatch(self):
        b=dict(t=5.,session='s',task_version='v',action='DEFER',reason_code='TRY_OTHER_OBLIGATION_RETAIN_RESIDUAL',stopped_confirmation=True,manual_latched=False,dispatch_id=3,evidence=dict(bindings_valid=True,stale=False,alternative=True,nav_idle=True,stopped=True,tool_stopped=True));st=dict(t=5.,nav_idle=True,bt=b,loc=dict(t=5.,valid=True,localization_epoch=2,native_supervisor=dict(plan_revalidation_required=False)),safety=dict(t=5.,valid=True,boundary_valid=True,localization_epoch=2),gate=dict(t=5.,stopped=True,tool_stopped=True,nav_idle=True),tool=dict(t=5.,stopped_confirmed=True),feedback=dict(t=5.,healthy=True,request_on=False,tool_revision='rev',rpm_rad_s=.01,flow=0.))
        args=[dict(version='v'),dict(session='s'),dict(feedback_age_sim_s=.1,tool_revision='rev',stopped_max_rad_s=.1),False,['other']]
        self.assertTrue(all(defer_admission(st,*args).values()))
        for group,key,value in [('safety','valid',False),('safety','localization_epoch',9),('loc','valid',False),('gate','stopped',False),('tool','stopped_confirmed',False),('feedback','t',4.8),('feedback','flow',1.),('feedback','healthy',False),('bt','action','REPLAN'),('bt','task_version','other'),('bt','stopped_confirmation',False)]:
            blocked=copy.deepcopy(st);blocked[group][key]=value;self.assertFalse(all(defer_admission(blocked,*args).values()),(group,key))
        self.assertFalse(all(defer_admission(st,*args[:3],True,['other']).values()));self.assertFalse(all(defer_admission(st,*args[:4],[]).values()))

if __name__=='__main__':unittest.main()
