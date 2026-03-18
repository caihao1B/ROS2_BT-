import rclpy
from rclpy.node import Node
from coverage_msgs.srv import ComputeDetour

class LocalReplanner(Node):
    def __init__(self):
        super().__init__('local_replanner')
        self.srv = self.create_service(ComputeDetour, '/coverage/compute_detour', self.cb)
        self.get_logger().info("Service /coverage/compute_detour ready (STUB: returns failure)")

    def cb(self, req, resp):
        resp.success = False
        resp.message = "STUB: detour not implemented yet"
        resp.detour_path = req.nominal_path
        return resp

def main():
    rclpy.init()
    n = LocalReplanner()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
