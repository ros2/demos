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

import sys

import rclpy

from std_msgs.msg import String


class Listener(rclpy.Node):

    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'chatter', self.chatter_callback)

    def chatter_callback(self, msg):
        print('I heard: [%s]' % msg.data)


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = Listener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
