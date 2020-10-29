#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='need4stek',
            namespace='need4stek',
            executable='capability1',
            name='wall_following',
            remappings=[
                ('/need4stek/scan', '/scan'),
                ('/need4stek/cmd_vel', '/cmd_vel')
            ]
        )
    ])