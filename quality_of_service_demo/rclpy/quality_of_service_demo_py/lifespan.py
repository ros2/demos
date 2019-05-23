# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

from quality_of_service_demo_py.common_nodes import Listener
from quality_of_service_demo_py.common_nodes import Talker

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'lifespan', type=int,
        help='Duration in positive integer milliseconds of the Lifespan QoS setting.')
    parser.add_argument(
        '--history', type=int, default=10,
        help="The depth of the Publisher's history queue - "
             'the maximum number of messages it will store for late-joining subscriptions.')
    parser.add_argument(
        '--publish-count', type=int, default=10,
        help='How many messages to publish before stopping.')
    parser.add_argument(
        '--subscribe-after', type=int, default=2500,
        help='The Subscriber will be created this long (in positive integer milliseconds) '
             'after application startup.')
    return parser.parse_args()


def main(args=None):
    parsed_args = parse_args()
    rclpy.init(args=args)

    topic = 'qos_lifespan_chatter'
    lifespan = Duration(seconds=parsed_args.lifespan / 1000.0)

    qos_profile = QoSProfile(
        depth=parsed_args.history,
        # Guaranteed delivery is needed to send messages to late-joining subscription.
        reliability=QoSReliabilityPolicy.RELIABLE,
        # Store messages on the publisher so that they can be affected by Lifespan.
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        lifespan=lifespan)

    listener = Listener(
        topic, qos_profile, event_callbacks=None, defer_subscribe=True)
    talker = Talker(
        topic, qos_profile, event_callbacks=None, publish_count=parsed_args.publish_count)
    subscribe_timer = listener.create_timer(  # noqa: F841
        parsed_args.subscribe_after / 1000.0,
        lambda: listener.start_listening())

    executor = SingleThreadedExecutor()
    executor.add_node(listener)
    executor.add_node(talker)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
