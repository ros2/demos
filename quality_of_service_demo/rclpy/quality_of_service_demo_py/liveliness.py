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
import sys

from quality_of_service_demo_py.common_nodes import Listener
from quality_of_service_demo_py.common_nodes import Talker

import rclpy
from rclpy.duration import Duration
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSProfile

POLICY_MAP = {
    'AUTOMATIC': QoSLivelinessPolicy.AUTOMATIC,
    'MANUAL_BY_TOPIC': QoSLivelinessPolicy.MANUAL_BY_TOPIC,
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'liveliness_lease_duration', type=int,
        help='Duration in positive integer milliseconds of the Liveliness lease_duration '
             'QoS setting.')
    parser.add_argument(
        '--policy', type=str, choices=POLICY_MAP.keys(), default='AUTOMATIC',
        help='The Liveliness policy type.')
    parser.add_argument(
        '--topic-assert-period', type=int, default=0,
        help='How often (in positive integer milliseconds) the Talker will manually assert the '
             'liveliness of its Publisher.')
    parser.add_argument(
        '--kill-publisher-after', type=int, default=3000,
        help='Shuts down the Talker after this duration, in positive integer milliseconds.')
    return parser.parse_args()


def main(args=None):
    parsed_args = parse_args()
    rclpy.init(args=args)

    topic = 'qos_liveliness_chatter'
    liveliness_lease_duration = Duration(seconds=parsed_args.liveliness_lease_duration / 1000.0)
    liveliness_policy = POLICY_MAP[parsed_args.policy]

    qos_profile = QoSProfile(
        depth=10,
        liveliness=liveliness_policy,
        liveliness_lease_duration=liveliness_lease_duration)

    def sub_liveliness_event(event):
        get_logger('listener').info('Liveliness changed event:')
        get_logger('listener').info(f'  alive_count: {event.alive_count}')
        get_logger('listener').info(f'  not_alive_count: {event.not_alive_count}')
        get_logger('listener').info(f'  alive_count_change: {event.alive_count_change}')
        get_logger('listener').info(f'  not_alive_count_change: {event.not_alive_count_change}')

    subscription_callbacks = SubscriptionEventCallbacks(liveliness=sub_liveliness_event)
    listener = Listener(topic, qos_profile, event_callbacks=subscription_callbacks)

    def pub_liveliness_event(event):
        get_logger('talker').info('Liveliness changed event:')
        get_logger('talker').info(f'  alive_count: {event.alive_count}')
        get_logger('talker').info(f'  not_alive_count: {event.not_alive_count}')
        get_logger('talker').info(f'  alive_count_change: {event.alive_count_change}')
        get_logger('talker').info(f'  not_alive_count_change: {event.not_alive_count_change}')

    publisher_callbacks = PublisherEventCallbacks(liveliness=pub_liveliness_event)
    talker = Talker(
        topic, qos_profile,
        event_callbacks=publisher_callbacks,
        assert_topic_period=parsed_args.topic_assert_period / 1000.0)

    executor = SingleThreadedExecutor()

    def kill_talker():
        if liveliness_policy == QoSLivelinessPolicy.AUTOMATIC:
            executor.remove_node(talker)
            talker.destroy_node()
        elif liveliness_policy == QoSLivelinessPolicy.MANUAL_BY_TOPIC:
            talker.stop()
        kill_timer.cancel()

    if parsed_args.kill_publisher_after > 0:
        kill_timer = listener.create_timer(  # noqa: F841
            parsed_args.kill_publisher_after / 1000.0,
            kill_talker)

    executor.add_node(listener)
    executor.add_node(talker)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
