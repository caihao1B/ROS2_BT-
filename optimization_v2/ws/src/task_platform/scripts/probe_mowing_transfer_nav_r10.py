"""Real Nav2 map/planner services on an isolated ROS domain for the frozen B1 diagnostic transfer."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from tempfile import TemporaryDirectory

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

from common import APP, ROOT, edition, sha
from mowing_verified_transfer_r10 import checked_plan, TRANSFER


def exercise():
    plan=checked_plan();settings=edition('R10')/'mowing/nav2.yaml';prior=edition('R07')/'prior.yaml'
    domain=178
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain))
    env['AMENT_PREFIX_PATH']=str(APP/'build/r08_nav_install')+':'+env.get('AMENT_PREFIX_PATH','')
    env['LD_LIBRARY_PATH']=str(APP/'build/r08_nav_install/lib')+':'+env.get('LD_LIBRARY_PATH','')
    rclpy.init(domain_id=domain)
    n=rclpy.create_node('mowing_transfer_preflight',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    clk=n.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    observed=n.create_publisher(OccupancyGrid,'/r07/risk_local',5)
    static=StaticTransformBroadcaster(n);moving=TransformBroadcaster(n)
    identity=TransformStamped();identity.header.frame_id='map';identity.child_frame_id='odom';identity.transform.rotation.w=1.
    static.sendTransform(identity)
    state=dict(pose=plan['qualified_original_entry'],blocked=None,last=0.,local_map=None)
    n.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:state.update(local_map=m),qos_profile_sensor_data)
    begin=time.monotonic();results=[]
    def tick():
        elapsed=time.monotonic()-begin
        if elapsed>75.:raise TimeoutError('Private Nav2 component startup/verification deadline')
        if elapsed-state['last']>.035:
            stamp=Clock();stamp.clock.sec,stamp.clock.nanosec=divmod(round((10+elapsed)*1e9),1000000000)
            clk.publish(stamp);state['last']=elapsed
            tr=TransformStamped();tr.header.stamp=stamp.clock;tr.header.frame_id='odom';tr.child_frame_id='base_link'
            x,y,h=state['pose'];tr.transform.translation.x=float(x);tr.transform.translation.y=float(y)
            tr.transform.rotation.z=math.sin(h/2);tr.transform.rotation.w=math.cos(h/2);moving.sendTransform(tr)
            g=OccupancyGrid();g.header.frame_id='odom';g.header.stamp=stamp.clock;g.info.resolution=.15
            g.info.width=160;g.info.height=160;g.info.origin.position.x=72.;g.info.origin.position.y=22.;g.info.origin.orientation.w=1.
            val=bytearray(g.info.width*g.info.height)
            if state['blocked'] is not None:
                bx,by=state['blocked'];ix=int((bx-72.)/.15);iy=int((by-22.)/.15)
                for iy0 in range(iy-3,iy+4):
                    for ix0 in range(ix-3,ix+4):
                        val[iy0*160+ix0]=100
            g.data=list(val);observed.publish(g)
        rclpy.spin_once(n,timeout_sec=.012)
    def wait(test,seconds):
        deadline=time.monotonic()+seconds
        while not test():
            if time.monotonic()>deadline:raise TimeoutError(str(test))
            tick()
    def call(client,request):
        wait(client.service_is_ready,12)
        future=client.call_async(request);wait(future.done,14);return future.result()
    def transition(name,number):
        client=n.create_client(ChangeState,'/'+name+'/change_state');msg=ChangeState.Request()
        msg.transition.id=number
        if not call(client,msg).success:raise RuntimeError(name+' lifecycle transition failed')
    def published_cost(x,y):
        m=state['local_map']
        if m is None:return None
        ix=math.floor((x-m.info.origin.position.x)/m.info.resolution)
        iy=math.floor((y-m.info.origin.position.y)/m.info.resolution)
        if not (0<=ix<m.info.width and 0<=iy<m.info.height):return None
        return m.data[iy*m.info.width+ix]
    children=[]
    with TemporaryDirectory(prefix='r10_mowing_nav_') as tmp:
        log=open(Path(tmp)/'nav.log','w')
        try:
            for package,name in [('nav2_map_server','map_server'),('nav2_planner','planner_server')]:
                children.append(subprocess.Popen(['/opt/ros/jazzy/lib/'+package+'/'+name,'--ros-args','-r','__node:='+name,
                    '--params-file',str(settings),'--log-level','warn'],env=env,stdout=log,stderr=log))
            for name in ('map_server','planner_server'):
                transition(name,Transition.TRANSITION_CONFIGURE);transition(name,Transition.TRANSITION_ACTIVATE)
            validity=n.create_client(IsPathValid,'/is_path_valid')
            for seg in plan['segments']:
                state['pose']=seg['poses'][0];state['blocked']=None
                mid=seg['poses'][len(seg['poses'])//2][:2]
                wait(lambda:published_cost(*mid) is not None and 0<=published_cost(*mid)<100,6)
                req=IsPathValid.Request();req.path.header.frame_id='map';req.path.header.stamp=n.get_clock().now().to_msg()
                for x,y,h in seg['poses']:
                    point=PoseStamped();point.header=req.path.header;point.pose.position.x=x;point.pose.position.y=y
                    point.pose.orientation.z=math.sin(h/2);point.pose.orientation.w=math.cos(h/2);req.path.poses.append(point)
                follow=FollowPath.Goal();follow.controller_id='TurnRPP';follow.goal_checker_id='goal';follow.progress_checker_id='progress';follow.path=req.path
                clear=call(validity,req)
                state['blocked']=mid;wait(lambda:published_cost(*mid)==100,6)
                obstructed=call(validity,req)
                results.append(dict(gear=seg['id'],direction=seg['direction'],path_poses=len(req.path.poses),
                                    real_clear_service=clear.is_valid,real_blocked_service=obstructed.is_valid,
                                    invalid_pose_indices=list(obstructed.invalid_pose_indices),wire_goal_bytes=len(serialize_message(follow))))
                state['blocked']=None;wait(lambda:published_cost(*mid) is not None and 0<=published_cost(*mid)<100,6)
                results[-1]['clear_again_after_removal']=call(validity,req).is_valid
            for name in ('planner_server','map_server'):
                transition(name,Transition.TRANSITION_DEACTIVATE);transition(name,Transition.TRANSITION_CLEANUP)
        except Exception:
            log.flush();print((Path(tmp)/'nav.log').read_text()[-3500:])
            raise
        finally:
            for child in reversed(children):
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:child.kill();child.wait()
            log.close();n.destroy_node();rclpy.try_shutdown()
    checks=dict(two_exact_gears=len(results)==2,
                clear_paths_accepted=all(r['real_clear_service'] and r['wire_goal_bytes']>0 for r in results),
                actual_obstacle_rejected=all(not r['real_blocked_service'] and r['clear_again_after_removal'] for r in results),
                clean_processes=all(p.returncode==0 for p in children))
    return dict(passed=all(checks.values()),checks=checks,gears=results,
                scope='Real Jazzy IsPathValid, frozen map_server + Smac planner/global costmap. Private domain with '
                      'synthetic TF and blank versus injected measured-grid fixtures. No actuator, no Gazebo and no driving credit.',
                domain=domain,prior_sha256=sha(prior),nav2_sha256=sha(settings),plan_sha256=sha(TRANSFER),
                probe_sha256=sha(Path(__file__)),wall_seconds=time.monotonic()-begin)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path)
    args=parser.parse_args();r=exercise();text=json.dumps(r,indent=2)+'\n'
    if args.out:args.out.write_text(text)
    print(text,end='');raise SystemExit(0 if r['passed'] else 2)
