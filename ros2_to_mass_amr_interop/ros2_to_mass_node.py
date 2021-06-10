#!/usr/bin/env python

import rclpy
from . import MassAMRInteropNode

# Interesting example https://github.com/clalancette/mtexec_example


def main(args=None):
    rclpy.init(args=args)
    node = MassAMRInteropNode()
    node.send_identity_report()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
