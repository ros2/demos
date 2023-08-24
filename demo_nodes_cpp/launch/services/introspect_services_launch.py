# Copyright 2023 Open Source Robotics Foundation, Inc.
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

"""Launch the introspection service and client."""

import launch
import launch_ros.actions


def generate_launch_description():
    server = launch_ros.actions.Node(
        package='demo_nodes_cpp', executable='introspection_service', output='screen')
    client = launch_ros.actions.Node(
        package='demo_nodes_cpp', executable='introspection_client', output='screen')
    return launch.LaunchDescription([
        server,
        client,
    ])
