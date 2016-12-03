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

from std_msgs.msg import Int64


time_between_data = 0.3  # time in seconds between data publications


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'data_name',
        nargs='?',
        default='topic1',
        help='Name of the data (must comply with ROS topic rules)')

    parser.add_argument(
        '--best-effort',
        action='store_true',
        default=False,
        help='Set QoS reliability option to "best effort"')

    parser.add_argument(
        '--end-after',
        type=int,
        action='store',
        help='Script will exit after publishing this many messages')

    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('%s_data_pub' % args.data_name)

    if args.best_effort:
        qos_profile = qos_profile_sensor_data
        print('Best effort publisher')
    else:
        qos_profile = qos_profile_default
        print('Reliable publisher')

    topic_name = '{0}_data{1}'.format(
        args.data_name, '_best_effort' if args.best_effort else '')
    data_pub = node.create_publisher(Int64, topic_name, qos_profile)

    msg = Int64()
    cycle_count = 0

    def publish_msg(val):
        msg.data = val
        data_pub.publish(msg)
        print('Publishing: "{0}"'.format(msg.data))
        sys.stdout.flush()  # this is to get the output to show immediately when using Launch

    while rclpy.ok():
        if args.end_after is not None and cycle_count >= args.end_after:
            publish_msg(-1)
            exit(0)

        publish_msg(cycle_count)
        cycle_count += 1

        try:
            sleep(time_between_data)
        except KeyboardInterrupt:
            publish_msg(-1)
            raise

if __name__ == '__main__':
    main()
