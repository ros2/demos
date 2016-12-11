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

from launch.main import main as ros2_launcher

def launch(launch_descriptor, argv):
    executable = 'topic_monitor_data_publisher'
    launch_descriptor.add_process(
        cmd=[executable, 'sensor'],
    )
    launch_descriptor.add_process(
        cmd=[executable, 'critical', '--best-effort'],
    )

def main():
    ros2_launcher([__file__])
