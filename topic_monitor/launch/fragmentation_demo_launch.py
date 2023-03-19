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

"""Launch data publishers with different payload sizes."""

from launch import LaunchDescription
import launch.actions
from launch_ros.substitutions import ExecutableInPackage


def generate_launch_description():
    executable = ExecutableInPackage(package='topic_monitor', executable='data_publisher')
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[executable, 'small', '--payload-size', '1', '--period', '4'],
            output='screen'),
        launch.actions.ExecuteProcess(
            cmd=[executable, 'medium', '--payload-size', '50000', '--period', '4'],
            output='screen'),
        launch.actions.ExecuteProcess(
            cmd=[executable, 'large', '--payload-size', '100000', '--period', '4'],
            output='screen'),
        launch.actions.ExecuteProcess(
            cmd=[executable, 'xlarge', '--payload-size', '150000', '--period', '4'],
            output='screen'),
    ])
