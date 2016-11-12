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
from time import sleep

import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data

from std_msgs.msg import String


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'robot_name',
        nargs='?',
        default='robot1',
        help='name of the robot (must comply with ROS topic rules)')

    parser.add_argument(
        '--reliable',
        dest='reliable',
        action='store_true',
        default=False,
        help='set QoS profile to reliable')

    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node('robot_status_pub')

    if args.reliable:
        qos_profile = qos_profile_default
        print('Reliable publisher')
    else:
        qos_profile = qos_profile_sensor_data
        print('Best effort publisher')

    status_pub = node.create_publisher(String, '{0}_status'.format(args.robot_name), qos_profile)

    msg = String()
    cycle_count = 0

    while rclpy.ok():
        msg.data = 'Alive'
        status_pub.publish(msg)
        print('Publishing: "{0}"'.format(msg.data))
        cycle_count += 1
        try:
            sleep(1)
        except KeyboardInterrupt:
            msg.data = 'Offline'
            status_pub.publish(msg)
            print('Publishing: "{0}"'.format(msg.data))
            raise

if __name__ == '__main__':
    main()
