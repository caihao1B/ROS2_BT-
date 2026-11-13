"""Read-only operational diagnosis and bounded escalation through the mission owner.

A planner failure is never treated as a mathematical proof of no path. Only a
fresh hard-occupancy grid with disconnected point reachability earns the more
specific map-blocked label. Neither outcome grants a motion permission.
"""
from common_showcase import *
import math
from collections import deque
import numpy as np
from scipy.ndimage import label

class ProgressMonitor:
    def __init__(self):self.history=deque();self.job=None;self.last=-1.
    def update(self,t,xy,job,moving_task,working,yaw=None):
        if t<self.last or job!=self.job or not moving_task or working:self.history.clear()
        self.last=t;self.job=job
        if not moving_task or working or xy is None:return dict(stalled_s=0.,manual=False)
        self.history.append((t,list(xy),yaw))
        def progressed(row):
            angular=yaw is not None and row[2] is not None and abs(math.atan2(math.sin(yaw-row[2]),math.cos(yaw-row[2])))>.05
            return math.dist(row[1],xy)>.10 or angular
        while self.history and progressed(self.history[0]):self.history.popleft()
        while len(self.history)>1000:self.history.popleft()
        elapsed=t-self.history[0][0] if self.history else 0.
        return dict(stalled_s=elapsed,manual=elapsed>=60.,warning=elapsed>=20.)

def connectivity(values,res,origin,xy,goal,fresh,localization_valid):
    if not fresh or not localization_valid:return dict(code='REFERENCE_OR_MAP_NOT_VALID',proves_map_blocked=False)
    if xy is None or goal is None:return dict(code='NO_CURRENT_GOAL',proves_map_blocked=False)
    values=np.asarray(values);h,w=values.shape
    ij=np.floor((np.array([xy,goal])[:,:2]-origin)/res).astype(int)
    if np.any(ij<0) or np.any(ij[:,0]>=w) or np.any(ij[:,1]>=h):return dict(code='GOAL_OUTSIDE_OBSERVED_WINDOW',proves_map_blocked=False)
    # 8-connected point search is deliberately more permissive than either
    # vehicle. Failure proves disconnection only in this observed grid.
    free=(values>=0)&(values<100);labels,_=label(free,np.ones((3,3)));s,g=[int(labels[y,x]) for x,y in ij];connected=s>0 and s==g
    return dict(code='POINT_CONNECTED_KINEMATICS_UNRESOLVED' if connected else 'OBSERVED_GRID_DISCONNECTED',point_connected=connected,proves_map_blocked=not connected,scope='current admitted grid only; unknown is forbidden, physical world outside observations is not proven blocked')

def current_goal(m,j):
    if not j:return None
    parts=j.get('reference_parts',[])
    active=(m.st.get('gate') or {}).get('action_id','')
    part=next((p for p in parts if active==p['id'] or active.startswith(p['id']+'_resume_')),None)
    if part is None and j.get('kind')=='INSPECTION':
        done=set(getattr(m,'inspection_completed_parts',[]))
        part=next((p for p in parts if p['id'] not in done),None)
    part=part or j
    poses=getattr(m,'inspection_executed_parts',{}).get(part['id'],part.get('poses',[]))
    return poses[-1][:2] if poses else None

def snapshot(m,terminal=False):
    if getattr(m,'showcase_terminal',False):return
    t=m.st['t'];last=getattr(m,'showcase_monitor_last',-100.)
    if not terminal and t-last<.5:return
    m.showcase_monitor_last=t
    if not hasattr(m,'showcase_progress'):m.showcase_progress=ProgressMonitor()
    xy=m.api['xy']().tolist() if m.st.get('pose') is not None else None
    heading=None
    if m.st.get('pose') is not None:
        q=m.st['pose'].pose.pose.orientation;heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    j=m.pending[0] if m.pending else None;goal=current_goal(m,j)
    stationary_service=bool(m.working and m.a.vehicle=='diff' and j and j.get('kind')=='POINT_SERVICE')
    progress=m.showcase_progress.update(t,xy,j['id'] if j else None,bool(m.initialized and j and not m.complete and not m.stopping),stationary_service,heading)
    if progress['manual'] and not terminal and not m.stopping:
        # Existing owner + BT enforce cancellation and tool stop. No second
        # cmd_vel, tool command, lease or directive publisher is introduced.
        m.showcase_manual_latched=True
        m.unsafe=True;m.error='SUPERVISOR_NO_PROGRESS_60S_MANUAL_CHECK'
    terminal_manual=terminal and not m.complete
    grid=m.st.get('global_map');diag=getattr(m,'showcase_connectivity',dict(code='NOT_REQUESTED',proves_map_blocked=False))
    if (terminal_manual or m.blocked or progress.get('warning')) and grid is not None and (terminal or t-getattr(m,'showcase_diag_t',-100.)>=3.):
        stamp=grid.header.stamp.sec+grid.header.stamp.nanosec*1e-9
        diag=connectivity(np.asarray(grid.data).reshape(grid.info.height,grid.info.width),grid.info.resolution,[grid.info.origin.position.x,grid.info.origin.position.y],xy,goal,grid.header.frame_id=='map' and 0<=t-stamp<.6,m.platform_ok);m.showcase_diag_t=t;m.showcase_connectivity=diag
        if terminal or not getattr(m,'showcase_grid_saved',False):
            np.savez_compressed(m.a.out/'blocked_grid.npz',data=np.asarray(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width),resolution=grid.info.resolution,origin=[grid.info.origin.position.x,grid.info.origin.position.y],frame=grid.header.frame_id,t=stamp)
            dump(m.a.out/'blocked_snapshot.json',dict(t=t,position_map=xy,goal_map=goal,diagnosis=diag,scope='observed occupancy, no simulator truth'))
            m.showcase_grid_saved=True
    tool=m.st.get('tool') or {};gate=m.st.get('gate') or {};glob=m.st.get('global_') or {};perception=m.st.get('perception') or {}
    manual=terminal_manual or getattr(m,'showcase_manual_latched',False);state='COMPLETED' if m.complete else 'MANUAL_CHECK_REQUIRED' if manual else 'WAITING_OR_REPLANNING' if m.blocked or not m.platform_ok or progress.get('warning') else 'WORKING' if m.working else 'NAVIGATING' if m.busy else 'READY'
    fresh_stop=all(-.05<=t-packet.get('t',-100)<.6 for packet in [gate,tool])
    data=dict(t=t,state=state,terminal=terminal,requires_manual=manual,reason=m.error or m.reason,bt_reason=m.directive.get('reason_code'),job=j['id'] if j else None,position_map=xy,goal_map=goal,completed_jobs=len(m.done),remaining_jobs=len(m.pending),replans=m.replans,deferrals=m.deferrals,progress=progress,route_diagnosis=diag,localization=dict(state=glob.get('state'),reason=glob.get('reason'),navigation_allowed=glob.get('navigation_allowed')),perception=dict(fresh=perception.get('geometry_fresh'),boundary_valid=perception.get('boundary_valid'),reason=perception.get('reason')),tool=dict(phase=tool.get('phase'),stopped_confirmed=tool.get('stopped_confirmed'),active=tool.get('active')),vehicle=dict(reason=gate.get('reason'),measured_standstill=gate.get('measured_standstill'),v=gate.get('measured_v'),w=gate.get('measured_w')),stop_confirmed=bool(fresh_stop and gate.get('measured_standstill') and tool.get('stopped_confirmed') and not tool.get('active')),automatic_retry_after_manual=False,operator_checks=['检查当前障碍与可通行空间','检查定位、传感器时间及地图版本','确认车辆与工具已停止','处理后从保存进度显式恢复，禁止直接删除残余'])
    dump(m.a.out/'motion_monitor.json',data)
    if manual:dump(m.a.out/'manual_handoff.json',data)
    with (m.a.out/'motion_monitor.jsonl').open('a') as f:f.write(json.dumps(data,ensure_ascii=False)+'\n')
    if terminal:m.showcase_terminal=True
