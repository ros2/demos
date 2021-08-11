# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Launch a producer and a consumer in a component container, enabling intraprocess comms."""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            node_namespace='/my_ns',
            node_name='my_container',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='intra_process_demo',
                    node_namespace='/my_ns', node_name='my_producer',
                    node_plugin='intra_process_demo::two_node_pipeline::Producer',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='intra_process_demo',
                    node_namespace='/my_ns', node_name='my_consumer',
                    node_plugin='intra_process_demo::two_node_pipeline::Consumer',
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
            output='screen',
        )
    ])
