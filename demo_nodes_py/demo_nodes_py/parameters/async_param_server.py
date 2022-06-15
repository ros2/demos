# Copyright 2022 Open Source Robotics Foundation, Inc.
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


def main(args=None):
    rclpy.init(args=args)
    target_node = rclpy.create_node('param_test_target')

    target_node.declare_parameter('zero', 1)
    target_node.declare_parameter('one', 0)
    target_node.declare_parameter('zero/one', 10)
    target_node.declare_parameter('one/two', 12)
    target_node.declare_parameter('true', False)
    target_node.declare_parameter('string string', 'string')

    rclpy.spin(target_node)
    target_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
