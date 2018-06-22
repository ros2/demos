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
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String


class ListenerQos(Node):

    def __init__(self, qos_profile):
        super().__init__('listener_qos')
        if qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE:
            self.get_logger().info('Reliable listener')
        else:
            self.get_logger().info('Best effort listener')
        self.sub = self.create_subscription(
            String, 'chatter', self.chatter_callback, qos_profile=qos_profile)

    def chatter_callback(self, msg):
        self.get_logger().info('I heard: [%s]' % msg.data)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--reliable', dest='reliable', action='store_true',
        help='set qos profile to reliable')
    parser.set_defaults(reliable=False)
    parser.add_argument(
        '-n', '--number_of_cycles', type=int, default=20,
        help='max number of spin_once iterations')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args(argv)
    rclpy.init(args=args.argv)

    if args.reliable:
        custom_qos_profile = qos_profile_default
    else:
        custom_qos_profile = qos_profile_sensor_data

    node = ListenerQos(custom_qos_profile)

    cycle_count = 0
    while rclpy.ok() and cycle_count < args.number_of_cycles:
        rclpy.spin_once(node)
        cycle_count += 1

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
