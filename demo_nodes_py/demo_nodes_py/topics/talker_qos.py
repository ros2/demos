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


class TalkerQos(rclpy.Node):

    def __init__(self, qos_profile):
        super().__init__('talker_qos')
        self.i = 0
        self.pub = self.create_publisher(String, 'chatter', qos_profile=qos_profile)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        print('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--reliable', dest='reliable', action='store_true',
        help='set qos profile to reliable')
    parser.set_defaults(reliable=False)
    parser.add_argument(
        '-n', '--number_of_cycles', type=int, default=20,
        help='number of sending attempts')
    args = parser.parse_args(argv)
    rclpy.init()

    if args.reliable:
        custom_qos_profile = qos_profile_default
        print('Reliable publisher')
    else:
        custom_qos_profile = qos_profile_sensor_data
        print('Best effort publisher')

    node = TalkerQos(custom_qos_profile)

    cycle_count = 0
    while rclpy.ok() and cycle_count < args.number_of_cycles:
        rclpy.spin_once(node)
        cycle_count += 1

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
