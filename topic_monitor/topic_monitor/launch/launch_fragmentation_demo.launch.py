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

"""Launch data publishers with different reliability configruations."""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='topic_monitor', node_executable='data_publisher', output='screen',
            arguments=['small', '--payload-size', '1', '--period', '4']),
        launch_ros.actions.Node(
            package='topic_monitor', node_executable='data_publisher', output='screen',
            arguments=['medium', '--payload-size', '50000', '--period', '4']),
        launch_ros.actions.Node(
            package='topic_monitor', node_executable='data_publisher', output='screen',
            arguments=['large', '--payload-size', '100000', '--period', '4']),
        launch_ros.actions.Node(
            package='topic_monitor', node_executable='data_publisher', output='screen',
            arguments=['xlarge', '--payload-size', '150000', '--period', '4']),
    ])
