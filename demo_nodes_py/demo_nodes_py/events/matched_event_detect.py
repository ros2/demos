# Copyright 2023 Sony Group Corporation.
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

import rclpy
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.event_handler import QoSPublisherMatchedInfo
from rclpy.event_handler import QoSSubscriptionMatchedInfo
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

'''
 This demo program shows detected matched event.
 Matched event occur while connection between publisher and subscription is done.
 Run this demo at one console by below command
 $ ros2 run demo_nodes_py matched_event_detect
 It will create 2 topics.
 - 'pub_matched_event_detect' for detecting matched event of publisher
 - 'sub_matched_event_detect' for detecting matched event of subscription

 For checking matched event of publisher
 On another console, run below command
 $ ros2 topic echo /pub_matched_event_detect
 You can run above command many times on different consoles to check the number of connected
 subscription by the output of demo program.

 For checking matched event of subscription
 On another console, run below command
 ros2 topic pub -r 1 /sub_matched_event_detect std_msgs/String "{data: '123'}"
 You can run above command many times on different consoles to check the number of connected
 publisher by the output of demo program.
'''


class MatchedEventDetectNode(Node):

    def __init__(self):
        super().__init__('matched_event_detection_node')
        self.connect_subscription = False  # used for publisher event
        self.connect_publisher = False  # used for subscription event

        pub_event_callback = PublisherEventCallbacks(matched=self.pub_matched_event_callback)
        self.pub = self.create_publisher(String, 'pub_matched_event_detect', 10,
                                         event_callbacks=pub_event_callback)

        sub_event_callback = SubscriptionEventCallbacks(matched=self.sub_matched_event_callback)
        self.sub = self.create_subscription(String, 'sub_matched_event_detect', self.sub_callback,
                                            10, event_callbacks=sub_event_callback)

    def pub_matched_event_callback(self, info: QoSPublisherMatchedInfo):
        if self.connect_subscription:
            if info.current_count == 0:
                self.get_logger().info('Last subscription is disconnected.')
                self.connect_subscription = False
            else:
                self.get_logger().info('Current number of connected subscription is '
                                       + str(info.current_count))
        else:
            if info.current_count != 0:
                self.get_logger().info('First subscription is connected.')
                self.connect_subscription = True

    def sub_matched_event_callback(self, info: QoSSubscriptionMatchedInfo):
        if self.connect_subscription:
            if info.current_count == 0:
                self.get_logger().info('Last publisher is disconnected.')
                self.connect_subscription = False
            else:
                self.get_logger().info('Current number of connected publisher is '
                                       + str(info.current_count))
        else:
            if info.current_count != 0:
                self.get_logger().info('First publisher is connected.')
                self.connect_subscription = True

    def sub_callback(self, msg):
        True


def main(args=None):
    rclpy.init(args=args)

    node = MatchedEventDetectNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
