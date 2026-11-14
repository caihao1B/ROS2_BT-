"""Exercise the installed exact-goal planner, including sampled straight connectors."""
import unittest,tempfile,subprocess,os,signal,time,sys,json,math
from pathlib import Path
import yaml,rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from lifecycle_msgs.srv import ChangeState
class PlannerContract(unittest.TestCase):
 def test_real_plugin_spacing_exact_goal_and_blocked_goal(self):
  root=Path(__file__).resolve().parents[5];prefix=root/'optimization_v2/three_tasks/build/r07_nav_install';env=os.environ.copy();env.update(ROS_DOMAIN_ID='127',AMENT_PREFIX_PATH=str(prefix)+':'+env.get('AMENT_PREFIX_PATH',''),LD_LIBRARY_PATH=str(prefix/'lib')+':'+env.get('LD_LIBRARY_PATH',''))
  with tempfile.TemporaryDirectory() as temp:
   folder=Path(temp);cfg={'planner_server':{'ros__parameters':{'use_sim_time':False,'expected_planner_frequency':1.,'planner_plugins':['GridBased'],'GridBased':{'plugin':'r07::ExactGoalPlanner','tolerance':.04,'allow_unknown':False}}},'global_costmap':{'global_costmap':{'ros__parameters':{'use_sim_time':False,'global_frame':'map','robot_base_frame':'map','rolling_window':False,'width':8,'height':8,'resolution':.1,'origin_x':-4.,'origin_y':-4.,'update_frequency':10.,'publish_frequency':5.,'footprint':'[[-0.3,-0.2],[0.3,-0.2],[0.3,0.2],[-0.3,0.2]]','footprint_padding':.05,'track_unknown_space':True,'plugins':['static'],'static':{'plugin':'nav2_costmap_2d::StaticLayer','map_topic':'/r07_test/map','map_subscribe_transient_local':True}}}}}
   config=folder/'nav.yaml';config.write_text(yaml.safe_dump(cfg));log=(folder/'planner.log').open('w');proc=subprocess.Popen(['/opt/ros/jazzy/lib/nav2_planner/planner_server','--ros-args','--params-file',str(config)],env=env,stdout=log,stderr=subprocess.STDOUT);rclpy.init(domain_id=127);n=rclpy.create_node('r07_planner_contract_test');qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE);pub=n.create_publisher(OccupancyGrid,'/r07_test/map',qos);grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=80;grid.info.resolution=.1;grid.info.origin.position.x=grid.info.origin.position.y=-4.;grid.info.origin.orientation.w=1.;grid.data=[0]*6400
   def drive(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:grid.header.stamp=n.get_clock().now().to_msg();pub.publish(grid);rclpy.spin_once(n,timeout_sec=.01);time.sleep(.005)
   def wait(f,limit=8):
    end=time.monotonic()+limit
    while not f.done() and time.monotonic()<end:drive(.02)
    self.assertTrue(f.done(),'planner response timeout');return f.result()
   try:
    client=n.create_client(ChangeState,'/planner_server/change_state');self.assertTrue(client.wait_for_service(timeout_sec=8));drive(.2)
    for transition in [1,3]:
     req=ChangeState.Request();req.transition.id=transition;self.assertTrue(wait(client.call_async(req)).success)
    action=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');self.assertTrue(action.wait_for_server(timeout_sec=3));drive(.3)
    def query(x,y):
     goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=True
     for pose,px,py in [(goal.start,-2.,0.),(goal.goal,x,y)]:pose.header.frame_id='map';pose.pose.position.x=px;pose.pose.position.y=py;pose.pose.orientation.w=1.
     handle=wait(action.send_goal_async(goal));self.assertTrue(handle.accepted);return wait(handle.get_result_async())
    result=query(1.03,.02);self.assertEqual(result.status,4);points=[(p.pose.position.x,p.pose.position.y) for p in result.result.path.poses];gap=max(math.dist(a,b) for a,b in zip(points,points[1:]));self.assertLessEqual(gap,.050001);self.assertLess(math.dist(points[-1],[1.03,.02]),1e-8);self.assertGreater(len(points),50)
    # Occupy goal and its entire footprint; a direct connector must not bypass it.
    data=list(grid.data)
    for y in range(32,49):
     for x in range(43,58):data[y*80+x]=100
    grid.data=data;drive(.5);blocked=query(1.03,.02);self.assertNotEqual(blocked.status,4)
    print(json.dumps(dict(real_planner_plugin=True,straight_samples=len(points),max_gap_m=gap,exact_endpoint=points[-1],blocked_status=blocked.status)))
   finally:
    proc.send_signal(signal.SIGINT)
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:proc.kill();proc.wait()
    log.close();n.destroy_node();rclpy.try_shutdown()
   self.assertEqual(proc.returncode,0,(folder/'planner.log').read_text()[-2000:])
if __name__=='__main__':unittest.main()
