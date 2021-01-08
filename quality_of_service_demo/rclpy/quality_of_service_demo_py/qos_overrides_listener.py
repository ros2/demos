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

import rclpy
from rclpy.node import Node
from rclpy.qos_overriding_options import QosCallbackResult
from rclpy.qos_overriding_options import QoSOverridingOptions

from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('qos_overrides_listener')
        self.sub = self.create_subscription(
            String, 'qos_overrides_chatter', self.chatter_callback, 10,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(
                callback=self.qos_callback,
                #  entity_id='my_custom_id',  # Use this if you want a custo qos override id.
            ))

    def chatter_callback(self, msg):
        self.get_logger().info('I heard: [%s]' % msg.data)

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

    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
