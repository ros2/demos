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
from time import sleep

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from std_msgs.msg import Header

default_depth = 10


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        'data_name', nargs='?', default='topic1',
        help='Name of the data (must comply with ROS topic rules)')

    parser.add_argument(
        '--best-effort', action='store_true', default=False,
        help="Set QoS reliability option to 'best effort'")

    parser.add_argument(
        '--transient-local', action='store_true', default=False,
        help="Set QoS durability option to 'transient local'")

    parser.add_argument(
        '--depth', type=int, default=default_depth, help='Size of the QoS history depth')

    parser.add_argument(
        '--keep-all', action='store_true', default=False,
        help="Set QoS history option to 'keep all' (unlimited depth, subject to resource limits)")

    parser.add_argument(
        '--payload-size', type=int, default=0, help='Size of data payload to send')

    parser.add_argument(
        '--period', type=float, default=0.5, help='Time in seconds between messages')

    parser.add_argument(
        '--end-after', type=int, help='Script will exit after publishing this many messages')

    args = parser.parse_args()

    reliability_suffix = '_best_effort' if args.best_effort else ''
    topic_name = '{0}_data{1}'.format(args.data_name, reliability_suffix)

    rclpy.init()
    node = rclpy.create_node('%s_pub' % topic_name)
    node_logger = node.get_logger()

    qos_profile = QoSProfile(depth=args.depth)

    if args.best_effort:
        node_logger.info('Reliability: best effort')
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
    else:
        node_logger.info('Reliability: reliable')
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE

    if args.keep_all:
        node_logger.info('History: keep all')
        qos_profile.history = QoSHistoryPolicy.KEEP_ALL
    else:
        node_logger.info('History: keep last')
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST

    node_logger.info('Depth: {0}'.format(args.depth))

    if args.transient_local:
        node_logger.info('Durability: transient local')
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    else:
        node_logger.info('Durability: volatile')
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

    data_pub = node.create_publisher(
        Header, topic_name, qos_profile)
    node_logger.info('Publishing on topic: {0}'.format(topic_name))

    node_logger.info('Payload size: {0}'.format(args.payload_size))
    data = 'a' * args.payload_size

    msg = Header()
    cycle_count = 0

    def publish_msg(val):
        msg.frame_id = '{0}_{1}'.format(val, data)
        data_pub.publish(msg)
        node_logger.info('Publishing: "{0}"'.format(val))

    while rclpy.ok():
        if args.end_after is not None and cycle_count >= args.end_after:
            publish_msg(-1)
            sleep(0.1)  # allow reliable messages to get re-sent if needed
            return

        publish_msg(cycle_count)
        cycle_count += 1

        try:
            sleep(args.period)
        except KeyboardInterrupt:
            publish_msg(-1)
            sleep(0.1)
            raise


if __name__ == '__main__':
    main()
