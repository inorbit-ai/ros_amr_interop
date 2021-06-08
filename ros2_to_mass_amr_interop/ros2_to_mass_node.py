#!/usr/bin/env python

import rclpy
from . import MassAMRInteropNode

# config_file_path = Path(__file__).resolve().parent / "config.yaml"
config_file_path = "/home/leandro/dev_ws/src/ros2-to-mass-amr-interop/config.yaml"


def main(args=None):
    rclpy.init(args=args)
    node = MassAMRInteropNode(config_file_path)
    node.send_identity_report()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
