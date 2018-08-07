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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.pub = self.create_publisher(String, 'chatter')
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
