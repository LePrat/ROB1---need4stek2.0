#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='need4stek',
            namespace='need4stek1',
            executable='need4stek_node',
            name='sim'
        )
    ])