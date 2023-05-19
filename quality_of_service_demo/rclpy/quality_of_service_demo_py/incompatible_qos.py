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
from rclpy.event_handler import UnsupportedEventTypeError
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'incompatible_qos_policy_name',
        type=str,
        choices=['durability', 'deadline', 'liveliness_policy', 'liveliness_lease_duration',
                 'reliability'],
        help='The QoS Policy that should be incompatible between the publisher and subscription.')
    return parser


def main(args=None):
    # Argument parsing and usage
    parser = get_parser()
    parsed_args = parser.parse_args()

    # Configuration variables
    qos_policy_name = parsed_args.incompatible_qos_policy_name
    qos_profile_publisher = QoSProfile(depth=10)
    qos_profile_subscription = QoSProfile(depth=10)

    if qos_policy_name == 'durability':
        print(
            'Durability incompatibility selected.\n'
            'Incompatibility condition: publisher durability kind < '
            'subscription durability kind.\n'
            'Setting publisher durability to: VOLATILE\n'
            'Setting subscription durability to: TRANSIENT_LOCAL\n'
        )
        qos_profile_publisher.durability = \
            QoSDurabilityPolicy.VOLATILE
        qos_profile_subscription.durability = \
            QoSDurabilityPolicy.TRANSIENT_LOCAL
    elif qos_policy_name == 'deadline':
        print(
            'Deadline incompatibility selected.\n'
            'Incompatibility condition: publisher deadline > subscription deadline.\n'
            'Setting publisher durability to: 2 seconds\n'
            'Setting subscription durability to: 1 second\n'
        )
        qos_profile_publisher.deadline = Duration(seconds=2)
        qos_profile_subscription.deadline = Duration(seconds=1)
    elif qos_policy_name == 'liveliness_policy':
        print(
            'Liveliness Policy incompatibility selected.\n'
            'Incompatibility condition: publisher liveliness policy <'
            'subscripition liveliness policy.\n'
            'Setting publisher liveliness policy to: AUTOMATIC\n'
            'Setting subscription liveliness policy to: MANUAL_BY_TOPIC\n'
        )
        qos_profile_publisher.liveliness = \
            QoSLivelinessPolicy.AUTOMATIC
        qos_profile_subscription.liveliness = \
            QoSLivelinessPolicy.MANUAL_BY_TOPIC
    elif qos_policy_name == 'liveliness_lease_duration':
        print(
            'Liveliness lease duration incompatibility selected.\n'
            'Incompatibility condition: publisher liveliness lease duration >'
            'subscription liveliness lease duration.\n'
            'Setting publisher liveliness lease duration to: 2 seconds\n'
            'Setting subscription liveliness lease duration to: 1 second\n'
        )
        qos_profile_publisher.liveliness_lease_duration = Duration(seconds=2)
        qos_profile_subscription.liveliness_lease_duration = Duration(seconds=1)
    elif qos_policy_name == 'reliability':
        print(
            'Reliability incompatibility selected.\n'
            'Incompatibility condition: publisher reliability < subscripition reliability.\n'
            'Setting publisher reliability to: BEST_EFFORT\n'
            'Setting subscription reliability to: RELIABLE\n'
        )
        qos_profile_publisher.reliability = \
            QoSReliabilityPolicy.BEST_EFFORT
        qos_profile_subscription.reliability = \
            QoSReliabilityPolicy.RELIABLE
    else:
        print('{name} not recognised.'.format(name=qos_policy_name))
        parser.print_help()
        return 1

    # Initialization and configuration
    rclpy.init(args=args)
    topic = 'incompatible_qos_chatter'
    num_msgs = 5

    def sub_incompatible_qos_event(event):
        count = event.total_count
        delta = event.total_count_change
        policy = event.last_policy_kind
        get_logger('listener').info(
            f'Requested incompatible qos - total {count} delta {delta} last_policy_kind: {policy}')

    def pub_incompatible_qos_event(event):
        count = event.total_count
        delta = event.total_count_change
        policy = event.last_policy_kind
        get_logger('talker').info(
            f'Offered incompatible qos - total {count} delta {delta} last_policy_kind: {policy}')

    publisher_callbacks = PublisherEventCallbacks(incompatible_qos=pub_incompatible_qos_event)
    subscription_callbacks = SubscriptionEventCallbacks(
        incompatible_qos=sub_incompatible_qos_event)

    try:
        talker = Talker(
            topic, qos_profile_publisher, event_callbacks=publisher_callbacks,
            publish_count=num_msgs)
        listener = Listener(
            topic, qos_profile_subscription, event_callbacks=subscription_callbacks)
    except UnsupportedEventTypeError as exc:
        print()
        print(exc, end='\n\n')
        print('Please try this demo using a different RMW implementation')
        return 1

    executor = SingleThreadedExecutor()
    executor.add_node(listener)
    executor.add_node(talker)

    try:
        while talker.publish_count < num_msgs:
            executor.spin_once()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
