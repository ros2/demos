# Copyright 2016 Open Source Robotics Foundation, Inc.
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
import sys

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher


def main():
    launcher = DefaultLauncher()
    launch_descriptor = LaunchDescriptor()
    package = 'topic_monitor'
    executable = os.path.join(get_package_prefix(package), 'lib', package, 'data_publisher')

    launch_descriptor.add_process(
        cmd=[executable, 'sensor', '--best-effort'],
        name='sensor',
    )
    launch_descriptor.add_process(
        cmd=[executable, 'critical'],
        name='critical',
    )
    launcher.add_launch_descriptor(launch_descriptor)

    rc = launcher.launch()
    if rc != 0:
        print('Something went wrong. Return code: ' + str(rc), file=sys.stderr)
        exit(rc)


if __name__ == '__main__':
    main()
