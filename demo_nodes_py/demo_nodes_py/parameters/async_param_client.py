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
import threading

from ament_index_python import get_package_share_directory
import rclpy
import rclpy.context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


def example_callback(future):
    print(f'Callback called with future {future}')


def main(args=None):
    target_node_name = 'param_test_target'

    context = rclpy.context.Context()
    rclpy.init(context=context)

    target_executor = SingleThreadedExecutor(context=context)
    client_executor = SingleThreadedExecutor(context=context)

    node = rclpy.create_node('async_param_client', context=context)
    target_node = rclpy.create_node(target_node_name,
                                    allow_undeclared_parameters=True,
                                    context=context)
    target_node.declare_parameters('', [
        ('zero', 1),
        ('one', 0),
        ('zero/one', 10),
        ('one/two', 12),
        ('true', False),
        ('string string', 'string'),
    ])

    target_executor.add_node(target_node)
    client_executor.add_node(node)

    thread = threading.Thread(target=target_executor.spin)
    thread.start()

    client = AsyncParameterClient(node, 'param_test_target')
    params = ['zero/one', 'one', 'string string']

    node.get_logger().info('----------------------- List Parameters ----------------------')

    future = client.list_parameters(['zero', 'one', 'true'], 10, callback=example_callback)
    client_executor.spin_until_future_complete(future)
    initial_parameters = future.result()
    if initial_parameters:
        node.get_logger().info(f'Parameters: {initial_parameters}')
    else:
        node.get_logger().error(f'Error listing parameters: {future.exception()}')

    node.get_logger().info('----------------------- Get Parameters #1 ----------------------')

    future = client.get_parameters(params, callback=example_callback)
    client_executor.spin_until_future_complete(future)
    parameter_values = future.result()
    if parameter_values:
        for v in parameter_values.values:
            node.get_logger().info(f'Parameters: {v}')
    else:
        node.get_logger().error(f'Error getting parameters: {future.exception()}')

    node.get_logger().info('----------------------- Set Parameters ----------------------')

    future = client.set_parameters([
        Parameter('zero/one', Parameter.Type.INTEGER, 88),
        Parameter('one', Parameter.Type.INTEGER, 1),
        Parameter('true', Parameter.Type.BOOL, True),
        Parameter('string string', Parameter.Type.STRING, 'a string'),
    ])

    client_executor.spin_until_future_complete(future)
    node.get_logger().info(f'Set parameters: {future.result()}')

    node.get_logger().info('----------------------- Get Parameters # 2----------------------')
    future = client.get_parameters(params, callback=example_callback)
    client_executor.spin_until_future_complete(future)

    parameter_values = future.result()
    if parameter_values:
        for v in parameter_values.values:
            node.get_logger().info(f'Parameters: {v}')
    else:
        node.get_logger().error(f'Error getting parameters: {future.exception()}')

    node.get_logger().info('----------------------- Describe Parameters ----------------------')

    future = client.describe_parameters(['zero'])
    client_executor.spin_until_future_complete(future)
    parameter_descriptions = future.result()
    if parameter_descriptions:
        node.get_logger().info(f'Parameters Description: {parameter_descriptions}')
    else:
        node.get_logger().error(f'Error describing parameters: {future.exception()}')

    node.get_logger().info('----------------------- Get Parameter Types ----------------------')

    future = client.get_parameter_types(params)
    client_executor.spin_until_future_complete(future)
    parameter_types = future.result()
    if parameter_types:
        node.get_logger().info(f'Parameters types: {parameter_types}')
    else:
        node.get_logger().error(f'Error getting parameter types: {future.exception()}')

    node.get_logger().info('----------------------- Set Parameters Atomically -------------------')

    future = client.set_parameters_atomically([
        Parameter('zero/one', Parameter.Type.INTEGER, 99),
        Parameter('one', Parameter.Type.INTEGER, 2),
    ])

    client_executor.spin_until_future_complete(future)
    set_resuts = future.result()
    if set_resuts:
        node.get_logger().info(f'Set parameters: {future.result()}')
    else:
        node.get_logger().error(f'Error setting parameters: {future.exception()}')

    node.get_logger().info('----------------------- Delete Parameters ----------------------')
    future = client.delete_parameters(params)
    client_executor.spin_until_future_complete(future)
    delete_results = future.result()
    if delete_results:
        node.get_logger().info(f'Delete parameters: {delete_results}')
    else:
        node.get_logger().error(f'Error deleting parameters: {future.exception()}')

    node.get_logger().info('----------------------- Load Parameters ----------------------')
    param_dir = get_package_share_directory('demo_nodes_py')
    future = client.load_parameter_file(os.path.join(param_dir, 'params.yaml'))
    client_executor.spin_until_future_complete(future)
    load_results = future.result()
    if load_results:
        node.get_logger().info(f'Load parameters: {load_results}')
    else:
        node.get_logger().error(f'Error loading parameters: {future.exception()}')

    client_executor.shutdown()
    target_executor.shutdown()
    rclpy.shutdown(context=context)


if __name__ == '__main__':
    main()
