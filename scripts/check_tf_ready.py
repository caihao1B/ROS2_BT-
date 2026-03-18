#!/usr/bin/env python3
import argparse, time, sys
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", required=True)  # e.g. map
    ap.add_argument("--source", required=True)  # e.g. odom
    ap.add_argument("--timeout", type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("check_tf_ready")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    deadline = time.time() + args.timeout
    ok = False
    while time.time() < deadline:
        try:
            buf.lookup_transform(args.target, args.source, Time())
            ok = True
            break
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if ok:
        print("READY")
        sys.exit(0)
    else:
        print("NOT_READY")
        sys.exit(1)

if __name__ == "__main__":
    main()
