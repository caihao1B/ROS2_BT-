"""Bounded best-effort STOP_WORK during owned-session shutdown; hardware watchdog remains independent."""
import time,rclpy
from rclpy.action import ActionClient
from mission_interfaces.action import ToolTask
rclpy.init();n=rclpy.create_node('s10_shutdown_client');c=ActionClient(n,ToolTask,'/s10/tool_task');deadline=time.monotonic()+7.;ok=False
try:
 if c.wait_for_server(timeout_sec=2.5):
  g=ToolTask.Goal();g.operation=3;f=c.send_goal_async(g)
  while not f.done() and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.03)
  if f.done() and f.result().accepted:
   r=f.result().get_result_async()
   while not r.done() and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.03)
   if r.done():print(r.result().result.state);ok=r.result().result.state=='STOPPED'
finally:n.destroy_node();rclpy.try_shutdown()
raise SystemExit(0 if ok else 2)
