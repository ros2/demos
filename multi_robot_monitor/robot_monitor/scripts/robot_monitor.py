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
import re
import sys
import time

import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data

from std_msgs.msg import String


stale_time = 3  # time in seconds after which a status is considered stale


class RobotMonitor:
    def __init__(self, robot_status_qos_profile):
        self.robot_status_qos_profile = robot_status_qos_profile
        self.robot_statuses = {}
        self.times_of_last_status = {}
        self.status_changed = False
        self.status_topic_pattern = re.compile("(\w*)_status")
        self.monitored_robots = set()

    def add_callback(self, robot_name):
        callback_name =  'robot_%s_status' % robot_name

        def robot_status_callback(msg):
            print('Received: [%s]' % msg.data)
            robot_id = robot_name
            status = msg.data
            self.times_of_last_status[robot_id] = time.time()  # TODO: time stamp of msg
            self.update_robot_status(robot_id, status)

        setattr(self, callback_name, robot_status_callback)
        return robot_status_callback

    def add_monitored_robot(self, robot_id, node):
        # Create a subscription to the robot's status topic
        robot_name = robot_id
        sub = node.create_subscription(
            String,
            '%s_status' % robot_name,
            self.add_callback(robot_name),
            self.robot_status_qos_profile)

        self.monitored_robots.add(robot_id)

    def get_robot_from_topic_name(self, topic_name):
        match = re.search(self.status_topic_pattern, topic_name)
        if match and match.groups():
            robot_name = match.groups()[0]
        return robot_name

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

    if args.reliable:
        qos_profile = qos_profile_default
        print('Reliable listener')
    else:
        qos_profile = qos_profile_sensor_data
        print('Best effort listener')

    robot_monitor = RobotMonitor(qos_profile)

    while rclpy.ok():
        # Check if there is a new robot online
        topic_names_and_types = node.get_topic_names_and_types()
        for topic_name in topic_names_and_types.topic_names:
            robot_name = robot_monitor.get_robot_from_topic_name(topic_name)
            is_new_robot = robot_name and robot_name not in robot_monitor.monitored_robots
            if is_new_robot:
                # Register new robot with the monitor
                robot_monitor.add_monitored_robot(robot_name, node)

        if robot_monitor.monitored_robots:
            rclpy.spin_once(node, 1)
            robot_monitor.check_status()

if __name__ == '__main__':
    main()
