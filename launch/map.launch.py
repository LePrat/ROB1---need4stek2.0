# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam', default='True')
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='False')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'))
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename', default=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'))

    autostart = LaunchConfiguration('autostart', default='true')

    rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'))
    use_simulator = LaunchConfiguration('use_simulator', default='True')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub', default='True')
    use_rviz = LaunchConfiguration('use_rviz', default='True')
    headless = LaunchConfiguration('headless', default='False')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    world = os.path.join(get_package_share_directory('need4stek'), 'challenge_maze.world')

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
            cwd=[launch_dir],
            output='screen')
    
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

    start_gazebo_client_cmd = ExecuteProcess(
            cmd=cmd,
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'
        )

    urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    return ld
