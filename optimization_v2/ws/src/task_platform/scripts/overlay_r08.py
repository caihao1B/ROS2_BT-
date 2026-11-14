"""R08 correspondence selection: search the eligible planar reference surface."""
from common import *
from overlay_r07 import overlay as base_overlay

def overlay(task,out):
 base_overlay(task,out);p=out/'regional_math.py';s=p.read_text();old='    def align(self,raw,seed,coarse=True):';new='        self.planar_ids=np.flatnonzero(self.normal_valid);self.planar_tree=cKDTree(self.points[self.planar_ids])\n'+old;assert s.count(old)==1;s=s.replace(old,new);old='ds,ix=self.tree.query(fit);mask=(ds<.6)&self.normal_valid[ix]';new='ds,indices=self.planar_tree.query(fit);ix=self.planar_ids[indices];mask=ds<.6';assert s.count(old)==1;s=s.replace(old,new);s=s.replace('(np.linalg.norm(raw,axis=1)<35.0)','(np.linalg.norm(raw,axis=1)<48.0)')
 old='        r=rotation(seed[2]);p=np.array(seed[:2],dtype=float);fit=raw[::2].copy();fit[:,:2]=fit[:,:2]@r.T+p'
 new='''        r=rotation(seed[2]);p=np.array(seed[:2],dtype=float)
        predicted=raw.copy();predicted[:,:2]=predicted[:,:2]@r.T+p
        raw_count=len(raw);known=self.tree.query(predicted)[0]<(1.8 if coarse else .6);raw=raw[known]
        if len(raw)<200:return dict(valid=False,reason='INSUFFICIENT_MAPPED_SURFACES',raw_count=raw_count,selected_count=len(raw))
        fit=raw[::2].copy();fit[:,:2]=fit[:,:2]@r.T+p'''
 assert s.count(old)==1;s=s.replace(old,new);s=s.replace("heldout_points=len(ds),translation_from_seed_m", "heldout_points=len(ds),raw_surface_points=raw_count,mapped_candidate_points=len(raw),mapped_fraction=len(raw)/raw_count,overlap_scope='held-out mapped landmark candidates; unknown objects retained by perception',translation_from_seed_m");p.write_text(s);d=read(out/'adaptation.json');d['generated']['regional_math.py']=sha(p);d['r08_adaptation']=dict(reference_query='Nearest eligible normal; original 100 match minimum, .6 m gate, Hessian and independent holdout unchanged');
 p=out/'icp_gate.py';s=p.read_text();s='import sys\nsys.path.insert(0,'+repr(str(PKG/'scripts'))+')\nfrom motion_consistency_r08 import assess\n'+s
 s=s.replace("history=[];increments=", "history=[];gnss_fixes=[];wheel_samples=[];increments=")
 s=s.replace("def wheel_observed(m):motion.update(wheel_t=stamp(m),wheel_v=m.twist.twist.linear.x)", "def wheel_observed(m):\n        motion.update(wheel_t=stamp(m),wheel_v=m.twist.twist.linear.x);wheel_samples.append((stamp(m),m.twist.twist.linear.x));wheel_samples[:]=[x for x in wheel_samples if x[0]>stamp(m)-3.]")
 s=s.replace("    def stationary(t):", "    def fix_observed(m):\n        g=json.loads(m.data);gnss_fixes.append(g);gnss_fixes[:]=[x for x in gnss_fixes if x['t']>g['t']-3.]\n    n.create_subscription(String,'/s08/gnss',fix_observed,20)\n    def stationary(t):")
 s=s.replace("stationary_conflict=False;quiet=stationary(t)","stationary_conflict=False;quiet=stationary(t);independent_motion=dict(available=False,icp_valid=False,wheel_valid=False)")
 s=s.replace("                if np.linalg.norm(velocity)<1.2", "                independent_motion=assess(history,gnss_fixes,wheel_samples,t)\n                if independent_motion['icp_valid'] and np.linalg.norm(velocity)<1.2")
 s=s.replace("row=dict(window_start=", "row=dict(independent_motion=independent_motion,window_start=")
 p.write_text(s);d['generated']['icp_gate.py']=sha(p)
 p=out/'input_adapter.py';s=p.read_text();s=s.replace("if fresh and nis is not None and nis>9.21 and abs(residual)>c['mismatch_mps']:","if fresh and nis is not None and nis>9.21 and abs(residual)>c['mismatch_mps'] and self.icp_data.get('independent_motion',{}).get('available') and not self.icp_data.get('independent_motion',{}).get('wheel_valid'):");old="        out=copy.deepcopy(m);out.twist.twist.linear.y=out.twist.twist.linear.z=0.;"
 new="        independent=self.icp_data.get('independent_motion',{})\n        if 0<=t-self.icp_data.get('t',-100)<.5 and independent.get('wheel_valid'):self.slipping=False;self.mismatch_since=None;self.recovery_since=None\n"+old
 assert s.count(old)==1;s=s.replace(old,new);p.write_text(s);d['generated']['input_adapter.py']=sha(p);d['r08_adaptation']['local_motion_integrity']='Independent capture-aligned GNSS displacement adjudicates ICP versus wheel; original NIS 9.21, all stop gates unchanged'
 dump(out/'adaptation.json',d);return out
