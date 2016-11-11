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
import time

import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data

from std_msgs.msg import String


stale_time = 3  # time in seconds after which a status is considered stale


class RobotMonitor:
    def __init__(self):
        self.robot_statuses = {}
        self.times_of_last_status = {}
        self.status_changed = False

    def robot_status_callback(self, msg):
        print('Received: [%s]' % msg.data)
        # TODO: deal with multiple robots
        robot_id = 'robot1'
        status = msg.data
        self.times_of_last_status[robot_id] = time.time()  # TODO: time stamp of msg
        self.update_robot_status(robot_id, status)

    def update_robot_status(self, robot_id, status):
        previous_status = self.robot_statuses.get(robot_id, None)
        self.status_changed = status != previous_status
        self.robot_statuses[robot_id] = status

    def output_status(self):
        print(self.robot_statuses)

    def check_status(self):
        current_time = time.time()
        for robot_id, time_of_last_status in self.times_of_last_status.items():
            elapsed_time = current_time - time_of_last_status 
            if elapsed_time > stale_time:
                robot_status = self.robot_statuses[robot_id]
                if robot_status != 'Offline':
                    self.update_robot_status(robot_id, 'Stale')

        if self.status_changed:
            self.output_status()
            self.status_changed = False


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

    # TODO: deal with multiple robots
    sub = node.create_subscription(
        String, 'robot1_status', robot_monitor.robot_status_callback, qos_profile)

    while rclpy.ok():
        rclpy.spin_once(node)
        robot_monitor.check_status()

if __name__ == '__main__':
    main()
