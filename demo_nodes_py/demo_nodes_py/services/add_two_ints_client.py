#!/usr/bin/env python3
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

from typing import Union

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.executors import ExternalShutdownException


def main(args: Union[list[str], None] = None) -> None:
    try:
        with rclpy.init(args=args):
            node = rclpy.create_node('add_two_ints_client')

            cli = node.create_client(AddTwoInts, 'add_two_ints')
            while not cli.wait_for_service(timeout_sec=1.0):
                print('service not available, waiting again...')
            req = AddTwoInts.Request()
            req.a = 2
            req.b = 3
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            result = future.result()
            if result is not None:
                res_sum = result.sum
                node.get_logger().info('Result of add_two_ints: %d' % res_sum)
            else:
                node.get_logger().error('Exception while calling service: %r' % future.exception())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
