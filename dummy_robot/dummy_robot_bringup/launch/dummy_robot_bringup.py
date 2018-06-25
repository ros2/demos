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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO(wjwwood): Use a substitution to find share directory once this is implemented in launch
    urdf = os.path.join(get_package_share_directory('dummy_robot_bringup'),
                        'launch', 'single_rrbot.urdf')
    return LaunchDescription([
        Node(package='dummy_map_server', node_executable='dummy_map_server', output='screen'),
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',
             output='screen', arguments=[urdf]),
        Node(package='dummy_sensors', node_executable='dummy_joint_states', output='screen'),
        Node(package='dummy_sensors', node_executable='dummy_laser', output='screen')
    ])
