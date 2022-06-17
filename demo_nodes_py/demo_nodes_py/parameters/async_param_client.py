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
import os
from tempfile import NamedTemporaryFile

import rclpy
from rclpy import Parameter
from rclpy.parameter_client import AsyncParameterClient


def foo(val):
    print('callback function called')


def main(args=None):
    rclpy.init()
    node = rclpy.create_node('async_param_client')
    target_node_name = 'param_test_target'
    client = AsyncParameterClient(node, target_node_name)
    params = ['zero/one', 'one', 'string string']

    print('----------------------- List Parameters ----------------------')

    future = client.list_parameters(['zero', 'one', 'true'], 10, callback=foo)
    rclpy.spin_until_future_complete(node, future)
    initial_parameters = future.result()
    if initial_parameters:
        node.get_logger().info(f'Parameters: {initial_parameters}')
    else:
        node.get_logger().info(f'Error listing parameters: {future.exception()}')

    print('----------------------- Get Parameters #1 ----------------------')

    future = client.get_parameters(params, callback=foo)
    rclpy.spin_until_future_complete(node, future)
    parameter_values = future.result()
    if parameter_values:
        for v in parameter_values.values:
            node.get_logger().info(f'Parameters: {v}')
    else:
        node.get_logger().info(f'Error getting parameters: {future.exception()}')

    print('----------------------- Set Parameters ----------------------')

    future = client.set_parameters([
        Parameter('zero/one', Parameter.Type.INTEGER, 88).to_parameter_msg(),
        Parameter('one', Parameter.Type.INTEGER, 1).to_parameter_msg(),
        Parameter('true', Parameter.Type.BOOL, True).to_parameter_msg(),
        Parameter('string string', Parameter.Type.STRING, 'a string').to_parameter_msg(),
    ])

    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info(f'Set parameters: {future.result()}')

    print('----------------------- Get Parameters # 2----------------------')
    future = client.get_parameters(params, callback=foo)
    rclpy.spin_until_future_complete(node, future)

    parameter_values = future.result()
    if parameter_values:
        for v in parameter_values.values:
            node.get_logger().info(f'Parameters: {v}')
    else:
        node.get_logger().info(f'Error getting parameters: {future.exception()}')

    print('----------------------- Describe Parameters ----------------------')

    future = client.describe_parameters(['zero'])
    rclpy.spin_until_future_complete(node, future)
    parameter_descriptions = future.result()
    if parameter_descriptions:
        node.get_logger().info(f'Parameters Description: {parameter_descriptions}')
    else:
        node.get_logger().info(f'Error describing parameters: {future.exception()}')

    print('----------------------- Get Parameter Types ----------------------')

    future = client.get_parameter_types(params)
    rclpy.spin_until_future_complete(node, future)
    parameter_types = future.result()
    if parameter_types:
        node.get_logger().info(f'Parameters types: {parameter_types}')
    else:
        node.get_logger().info(f'Error getting parameter types: {future.exception()}')

    print('----------------------- Set Parameters Atomically ----------------------')

    future = client.set_parameters_atomically([
        Parameter('zero/one', Parameter.Type.INTEGER, 99).to_parameter_msg(),
        Parameter('one', Parameter.Type.INTEGER, 2).to_parameter_msg(),
    ])

    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info(f'Set parameters: {future.result()}')

    print('----------------------- Delete Parameters ----------------------')
    future = client.delete_parameters(params)
    rclpy.spin_until_future_complete(node, future)
    print(future.result())

    print('----------------------- Load Parameters ----------------------')
    yaml_string = """
    /param_test_target:
        ros__parameters:
            one: 0
            one/two: 12
            string string: string
            'true': false
            use_sim_time: false
            zero: 1
            zero/one: 10 """
    with NamedTemporaryFile(mode='w', delete=False) as f:
        f.write(yaml_string)
        f.flush()
        f.close()
        future = client.load_parameter_file(f.name)
        rclpy.spin_until_future_complete(node, future)
        print(future.result())
        os.unlink(f.name)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
