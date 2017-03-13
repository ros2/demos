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

from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher

file_path = os.path.dirname(os.path.realpath(__file__))


def launch():
    ld = LaunchDescriptor()

    ld.add_process(
        cmd=['dummy_laser'],
    )
    ld.add_process(
        cmd=['dummy_map_server'],
    )

    ld.add_process(
        cmd=['robot_state_publisher', os.path.join(file_path, 'single_rrbot.urdf')]
    )

    ld.add_process(
        cmd=['dummy_joint_states']
    )

    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(ld)
    rc = launcher.launch()

    assert rc == 0, "The launch file failed with exit code '" + str(rc) + "'. "


if __name__ == "__main__":
    launch()
