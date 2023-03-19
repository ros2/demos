# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share = FindPackageShare('dummy_robot_bringup').find('dummy_robot_bringup')

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/dummy_robot.rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    urdf_file = os.path.join(pkg_share, 'launch', 'single_rrbot.urdf')

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=default_rviz_config_path
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    return LaunchDescription([
        rviz_config_file_arg,
        Node(package='dummy_map_server', executable='dummy_map_server', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[rsp_params]),
        Node(package='dummy_sensors', executable='dummy_joint_states', output='screen'),
        Node(package='dummy_sensors', executable='dummy_laser', output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_config_file])

    ])
