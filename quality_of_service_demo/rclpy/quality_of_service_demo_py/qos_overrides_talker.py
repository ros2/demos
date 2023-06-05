# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos_overriding_options import QosCallbackResult
from rclpy.qos_overriding_options import QoSOverridingOptions

from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__('qos_overrides_talker')
        self.i = 0
        self.pub = self.create_publisher(
            String, 'qos_overrides_chatter', 10,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(
                callback=self.qos_callback,
                #  entity_id='my_custom_id',  # Use this if you want a custo qos override id.
            ))
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)

    def qos_callback(self, qos):
        result = QosCallbackResult()
        if qos.depth <= 10:
            result.successful = True
            return result
        result.successful = False
        result.reason = 'expected qos depth less than 10'
        return result


def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
