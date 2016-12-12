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
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from std_msgs.msg import Header


default_depth = 10


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
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
        '--transient-local',
        action='store_true',
        default=False,
        help='Set QoS durability option to "transient local"')

    parser.add_argument(
        '--depth',
        type=int,
        default=default_depth,
        action='store',
        help='Size of the QoS history depth')

    parser.add_argument(
        '--keep-all',
        action='store_true',
        default=False,
        help='Set QoS history option to "keep all" (unlimited depth, subject to resource limits)')

    parser.add_argument(
        '--payload-size',
        type=int,
        action='store',
        default=0,
        help='Size of data payload to send')

    parser.add_argument(
        '--period',
        type=float,
        default=0.5,
        action='store',
        help='Time in seconds between messages')

    parser.add_argument(
        '--end-after',
        type=int,
        action='store',
        help='Script will exit after publishing this many messages')

    args = parser.parse_args()

    rclpy.init()

    qos_profile = QoSProfile()

    qos_profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_BEST_EFFORT if args.best_effort \
        else QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABLE

    qos_profile.history = QoSHistoryPolicy.RMW_QOS_POLICY_KEEP_ALL_HISTORY if args.keep_all \
        else QoSHistoryPolicy.RMW_QOS_POLICY_KEEP_LAST_HISTORY

    qos_profile.depth = args.depth

    qos_profile.durability = \
        QoSDurabilityPolicy.RMW_QOS_POLICY_TRANSIENT_LOCAL_DURABILITY if args.transient_local \
        else QoSDurabilityPolicy.RMW_QOS_POLICY_VOLATILE_DURABILITY


    depth_suffix = '_depth_{0}'.format(args.depth) if args.depth != default_depth else ''
    reliability_suffix = '_best_effort' if args.best_effort else ''
    topic_name = '{0}{1}_data{2}'.format(
        args.data_name, depth_suffix, reliability_suffix)
    print(topic_name)

    node = rclpy.create_node('%s_pub' % topic_name)
    data_pub = node.create_publisher(Header, topic_name, qos_profile)

    msg = Header()
    data = 'a' * args.payload_size
    cycle_count = 0

    def publish_msg(val):
        msg.frame_id = '%i_%s' % (val, data)
        data_pub.publish(msg)
        print('Publishing: "{0}"'.format(val))
        sys.stdout.flush()  # this is to get the output to show immediately when using Launch

    while rclpy.ok():
        if args.end_after is not None and cycle_count >= args.end_after:
            publish_msg(-1)
            sleep(0.1)  # allow reliable messages to get re-sent if needed
            exit(0)

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
