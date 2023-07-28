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
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction

import launch_testing
import launch_testing.asserts
import launch_testing_ros


def generate_test_description(ready_fn):
    launch_description = LaunchDescription()
    publisher_executable_args = ['-r', '1', '-s', '0', '-b', '-f', '5']
    subscriber_executable_args = ['-r', '1', '-s', '0', '-b']

    env = dict(os.environ)
    env['OSPL_VERBOSITY'] = '8'  # 8 = OS_NONE
    # bare minimum formatting for console output matching
    env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'
    env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

    # Launch the process that will receive the images.
    # This is the process that gets to decide when to tear the launcher down.
    # It will exit when the regex for receiving images is matched.
    showimage_executable = '/home/utkarsh/clearbot/demos_custom/image_tools/.obj-x86_64-linux-gnu/showimage'
    showimage_name = 'test_showimage'

    command = [showimage_executable]
    command.extend(subscriber_executable_args)
    showimage_process = ExecuteProcess(
        cmd=command,
        name=showimage_name,
        env=env,
        output='screen'
    )
    launch_description.add_action(showimage_process)

    # Launch the process that will publish the images.
    # This process will be exited when the launcher is torn down.
    cam2image_executable = '/home/utkarsh/clearbot/demos_custom/image_tools/.obj-x86_64-linux-gnu/cam2image'
    cam2image_name = 'test_cam2image'

    command = [cam2image_executable]
    command.extend(publisher_executable_args)
    cam2image_process = ExecuteProcess(
        cmd=command,
        name=cam2image_name,
        env=env,
        output='screen'
    )
    launch_description.add_action(cam2image_process)

    launch_description.add_action(
        OpaqueFunction(function=lambda context: ready_fn())
    )

    return launch_description, locals()


class TestExecutablesDemo(unittest.TestCase):

    def test_reliable_qos(self, proc_output, showimage_process, cam2image_process):
        """Test QoS settings for both processes work as expected."""
        output_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_rmw_implementation='rmw_fastrtps_cpp'
        )
        proc_output.assertWaitFor(
            expected_output=launch_testing.tools.expected_output_from_file(
                path='/home/utkarsh/clearbot/demos_custom/image_tools/test/showimage'
            ), process=showimage_process, output_filter=output_filter, timeout=10
        )
        proc_output.assertWaitFor(
            expected_output=launch_testing.tools.expected_output_from_file(
                path='/home/utkarsh/clearbot/demos_custom/image_tools/test/cam2image'
            ), process=cam2image_process, output_filter=output_filter, timeout=10
        )
