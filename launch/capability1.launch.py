#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='need4stek',
            namespace='',
            executable='capability1',
            name='wall_following'
        )
    ])