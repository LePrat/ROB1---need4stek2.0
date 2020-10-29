#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = 'burger' if 'TURLEBOT_MODEL' not in os.environ else os.environ['TURTLEBOT3_MODEL']

model_path = 'models/turtlebot3_' + TURTLEBOT3_MODEL + '/model.sdf'

model_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), model_path)

cmd = ['gz',
    'model',
    '-m',
    model_dir,
    '-f',
    model_dir
]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    world = os.path.join(get_package_share_directory('need4stek'), 'challenge_maze.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
            output='screen'),

        ExecuteProcess(
            cmd=cmd,
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])