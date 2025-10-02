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

"""Launch data publishers with different depth configruations."""

from launch import LaunchDescription
import launch.actions
from launch_ros.substitutions import ExecutableInPackage


def create_data_publisher_action(size, depth):
    name = '{0}_depth_{1}'.format(size, depth)
    payload = 0 if size == 'small' else 100000
    executable = ExecutableInPackage(package='topic_monitor', executable='data_publisher')

    return launch.actions.ExecuteProcess(
        cmd=[executable, name, '--depth', str(depth), '--payload-size', str(payload)],
        output='screen',
    )


def generate_launch_description():
    return LaunchDescription([
        create_data_publisher_action('small', 1),
        create_data_publisher_action('small', 50),
        create_data_publisher_action('large', 1),
        create_data_publisher_action('large', 50),
    ])
