# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at # #     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescriptor
from launch.exit_handler import primary_exit_handler
from launch.launcher import DefaultLauncher

from os import environ
import sys


def launch(rosmaster_uri = ''):
    ld = LaunchDescriptor()

    if rosmaster_uri != '':
        environ['ROS_MASTER_URI'] = rosmaster_uri
#    ld.add_process(
#        cmd=['dynamic_bridge', '--bridge-all-2to1-topics'],
#        )
    ld.add_process(
        cmd=['static_transform_publisher', '0', '0', '1', '0', '0', '1.57', 'base_link', 'link1' ],
        output_handlers=()
    )
    ld.add_process(
        cmd=['static_transform_publisher', '0', '0', '1', '0', '0', '1.57', 'link1', 'link2' ],
        output_handlers=()
    )
    ld.add_process(
        cmd=['static_transform_publisher', '0', '0', '0.1', '1.57', '-1.57', '0', 'link2', 'dummy_laser_link' ],
        output_handlers=()
    )

    ld.add_process(
        cmd=['dummy_laser'],
    )
    ld.add_process(
        cmd=['dummy_map_server'],
    )

    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(ld)
    rc = launcher.launch()

    assert rc == 0, "The launch file failed with exit code '" + str(rc) + "'. "


if __name__ == "__main__":
    rosmaster_uri = ''
    if len(sys.argv) > 1:
        rosmaster_uri = 'http://%s:11311' % sys.argv[1]
        print("setting rosmaster uri to %s" % rosmaster_uri)
    launch(rosmaster_uri)
