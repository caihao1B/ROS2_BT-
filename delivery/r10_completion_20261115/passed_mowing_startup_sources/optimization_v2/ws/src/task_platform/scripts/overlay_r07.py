"""Rig overlay plus balanced, measured surface selection for local ICP."""
from rig_r05 import overlay as upstream_overlay,replace_one
from common import *
def overlay(task,out):
    upstream_overlay(task,out)
    p=out/'cloud_filter.py';s=p.read_text()
    old="ground=np.abs(xyz[:,2])<c['ground_band_m'];keep=~ground;keep[np.flatnonzero(ground)[::c['ground_stride']]]=True;xyz=xyz[keep]\n        if len(xyz)>c['max_points']:xyz=xyz[np.linspace(0,len(xyz)-1,c['max_points'],dtype=int)]"
    new="""ground=np.abs(xyz[:,2])<c['ground_band_m'];elevated=np.flatnonzero(~ground);ground_ids=np.flatnonzero(ground)
        if len(elevated)>2200:elevated=elevated[np.linspace(0,len(elevated)-1,2200,dtype=int)]
        if len(ground_ids)>300:ground_ids=ground_ids[np.linspace(0,len(ground_ids)-1,300,dtype=int)]
        xyz=xyz[np.sort(np.r_[elevated,ground_ids])]"""
    p.write_text(replace_one(s,old,new));d=read(out/'adaptation.json');d['r07_adaptation']=dict(purpose='Keep observed upright structures; cap floor points at 300 to prevent ground-dominated ICP',max_elevated=2200,max_ground=300,original_quality_gates_unchanged=True);d['generated']['cloud_filter.py']=sha(p);
    p=out/'icp_gate.py';s=p.read_text();s=replace_one(s,'consistency_v=None;increment_valid=False;', 'consistency_v=None;increment_valid=False;window_start=None;window_end=None;increment_variance=None;')
    s=replace_one(s,'velocity=Rotation.from_quat(q).inv().apply(p-pp)/span', 'middle=Rotation.from_quat(pq)*Rotation.from_rotvec((Rotation.from_quat(pq).inv()*Rotation.from_quat(q)).as_rotvec()*.5)\n                velocity=middle.inv().apply(p-pp)/span')
    s=replace_one(s,"inc.header.frame_id='base_link'", "inc.header.frame_id='base_link';mid=(pt+t)/2;inc.header.stamp.sec=int(mid);inc.header.stamp.nanosec=round((mid-int(mid))*1e9)")
    s=replace_one(s,'increments.publish(inc);consistency_v=float(velocity[0]);increment_valid=True', 'increments.publish(inc);consistency_v=float(velocity[0]);increment_valid=True;window_start=pt;window_end=t;increment_variance=float(variance[0])')
    s=replace_one(s,'row=dict(stationary_window=', 'row=dict(window_start=window_start,window_end=window_end,increment_variance=increment_variance,stationary_window=');p.write_text(s)
    source=BASE/'ws/src/localization_guard/scripts/input_adapter.py';s='import sys\nsys.path.insert(0,'+repr(str(source.parent))+')\n'+source.read_text()
    s=replace_one(s,"self.icp_data.get('t',-100)-.8<=ts<=self.icp_data.get('t',-100)", "(self.icp_data.get('window_start') or self.icp_data.get('t',-100)-.8)<=ts<=(self.icp_data.get('window_end') or self.icp_data.get('t',-100))")
    s=replace_one(s,"        if fresh and abs(residual)>c['mismatch_mps']:", "        ivar=self.icp_data.get('increment_variance');nis=residual**2/(c['velocity_variance']+ivar) if fresh and ivar is not None and ivar>0 else None\n        if fresh and nis is not None and nis>9.21 and abs(residual)>c['mismatch_mps']:")
    s=replace_one(s,"nis=None if residual is None else residual**2/(c['velocity_variance']+.02)", "nis=nis")
    s=replace_one(s,'elif fresh:self.mismatch_since=None;self.recovery_since=None;self.slipping=False;self.wheel_history=[]','elif fresh:self.mismatch_since=None;self.recovery_since=None;self.slipping=False')
    (out/'input_adapter.py').write_text(s);d['upstream'][str(source.relative_to(ROOT))]=sha(source);d['r07_adaptation'].update(icp_window_epoch='midpoint with midpoint body orientation',ekf_lag_history_s=2.,wheel_slip_gate='matched actual window, amplitude plus covariance NIS > 9.21')
    for name in ['icp_gate.py','input_adapter.py']:d['generated'][name]=sha(out/name)
    dump(out/'adaptation.json',d);return out
