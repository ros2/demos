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

import time

from example_interfaces.srv import AddTwoInts

import rclpy


def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node('add_two_ints_client')

    cli = node.create_client(AddTwoInts, 'add_two_ints')

    req = AddTwoInts.Request()
    req.a = 2
    req.b = 3
    # TODO(mikaelarguedas) No wait for service in Python
    # need to leave some time for the connection to be established
    time.sleep(1)
    cli.call(req)
    while rclpy.ok():
        rclpy.spin_once(node)
        if cli.response is not None:
            print('Result of add_two_ints: %d' % cli.response.sum)
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
