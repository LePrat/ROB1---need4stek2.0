#!/usr/bin/env python3

import rclpy

from need4stek.obstacle_detection import Need4StekNode

def main(args=None):
    rclpy.init(args=args)
    n4sNode = Need4StekNode()
    rclpy.spin(n4sNode)

    n4sNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
