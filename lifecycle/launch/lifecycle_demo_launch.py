#!/usr/bin/env python3

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

from launch import LaunchDescriptor
from launch.exit_handler import primary_exit_handler
from launch.launcher import DefaultLauncher


def lifecycle_demo():
    ld = LaunchDescriptor()

    ld.add_process(
        cmd=['lifecycle_talker'],
    )

    ld.add_process(
        cmd=['lifecycle_listener'],
    )

    ld.add_process(
        cmd=['lifecycle_service_client'],
        exit_handler=primary_exit_handler,
    )

    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(ld)
    rc = launcher.launch()

    assert rc == 0, "The launch file failed with exit code '" + str(rc) + "'. "


if __name__ == '__main__':
    lifecycle_demo()
