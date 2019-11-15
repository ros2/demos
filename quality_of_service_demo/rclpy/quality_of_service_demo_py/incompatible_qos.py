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
from rclpy.logging import get_logger
from rclpy.qos import QoSProfile
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos_event import SubscriptionEventCallbacks


def main(args=None):
    rclpy.init(args=args)

    topic = 'incompatible_chatter'

    offered_qos_profile = QoSProfile(
        depth=10,
        liveliness= QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE,
        deadline=Duration(seconds=5))

    requested_qos_profile = QoSProfile(
        depth=10,
        deadline=Duration(seconds=2))

    subscription_callbacks = SubscriptionEventCallbacks(
        incompatible_qos=lambda event: get_logger('Listener').info(str(event)))

    listener = Listener(topic, requested_qos_profile, event_callbacks=subscription_callbacks)

    publisher_callbacks = PublisherEventCallbacks(
        incompatible_qos=lambda event: get_logger('Talker').info(str(event)))
    talker = Talker(topic, offered_qos_profile, event_callbacks=publisher_callbacks)

    publish_for_seconds = 1
    pause_for_seconds = 0.5
    pause_timer = talker.create_timer(  # noqa: F841
        publish_for_seconds,
        lambda: talker.pause_for(pause_for_seconds))

    executor = SingleThreadedExecutor()
    executor.add_node(listener)
    executor.add_node(talker)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
