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

import argparse
import sys

import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data

from std_msgs.msg import String

class RobotMonitor:
    def __init__(self):
        self.robot_statuses = {}

    def robot_status_callback(self, msg):
        print('Received: [%s]' % msg.data)
        robot_name = msg.data
        self.robot_statuses[robot_name] = 'Alive'

    def output_status(self):
        print(self.robot_statuses)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--reliable', dest='reliable', action='store_true',
        help='set qos profile to reliable')
    parser.set_defaults(reliable=False)
    args = parser.parse_args(argv)
    rclpy.init()

    node = rclpy.create_node('robot_monitor')
    robot_monitor = RobotMonitor()

    if args.reliable:
        qos_profile = qos_profile_default
        print('Reliable listener')
    else:
        qos_profile = qos_profile_sensor_data
        print('Best effort listener')

    sub = node.create_subscription(
        String, 'robot_status', robot_monitor.robot_status_callback, qos_profile)

    while rclpy.ok():
        rclpy.spin_once(node)
        robot_monitor.output_status()

if __name__ == '__main__':
    main()
