# Copyright 2017 Open Source Robotics Foundation, Inc.
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
from ros2run.api import get_executable_path

file_path = os.path.dirname(os.path.realpath(__file__))


def launch(launch_descriptor, argv):
    ld = launch_descriptor

    package = 'dummy_map_server'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='dummy_map_server')],
    )

    package = 'robot_state_publisher'
    ld.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='robot_state_publisher'),
            os.path.join(
                get_package_share_directory('dummy_robot_bringup'), 'launch', 'single_rrbot.urdf')
        ],
    )

    package = 'dummy_sensors'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='dummy_laser')],
    )

    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='dummy_joint_states')],
    )
