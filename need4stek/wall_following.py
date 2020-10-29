#!/usr/bin/env python3

import numpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

import sys

CENTER = 0
LEFT = 1
RIGHT = 2

class Need4StekNode(Node):

    def __init__(self):
        super().__init__('need4stek_wall_following')

        self.linear_velocity = 0.3
        self.angular_velocity = 0.0
        self.scan_ranges = [sys.maxsize, sys.maxsize, sys.maxsize]
        self.init_scan_state = False

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos)

        self.update_timer = self.create_timer(
            0.010,
            self.update_callback)

        self.get_logger().info("Need4Stek wall-following algorithm initialised.")

        twist = Twist()
        twist.angular.z = -0.5
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)

    def scan_callback(self, msg: LaserScan):
        self.get_logger().info("Scan callback.")
        ranges = [0, 30, 330]
        for i in range(0, 3):
            self.scan_ranges[i] = msg.ranges[ranges[i]]
        self.init_scan_state = True

    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_callback(self):
        if self.init_scan_state is True:
            self.detect_obstacle()
        
    def turn_right(self):
        twist = Twist()
        twist.angular.z = -0.5
        twist.linear.x = 0.1
        return twist

    def turn_left(self):
        twist = Twist()
        twist.angular.z = 0.5
        twist.linear.x = 0.1
        return twist

    def detect_obstacle(self):
        twist = Twist()
        safety_distance = 0.7
        side_dist = 0.6

        if self.scan_ranges[CENTER] > safety_distance:
            if self.scan_ranges[LEFT] < side_dist:
                twist = self.turn_right()
            elif self.scan_ranges[RIGHT] < side_dist:
                twist = self.turn_left()
            else:
                twist.linear.x = self.linear_velocity
                twist.angular.z = self.angular_velocity
        if self.scan_ranges[CENTER] < safety_distance:
            twist = self.turn_right()

        self.cmd_vel_pub.publish(twist)
