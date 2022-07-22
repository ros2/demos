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
from rclpy.parameter import parameter_dict_from_yaml_file
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient


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
        ('int_parameter', 1),
        ('bool_parameter', False),
        ('string_parameter', 'Hello World'),
    ])
    target_executor.add_node(target_node)
    client_executor.add_node(node)
    thread = threading.Thread(target=target_executor.spin)
    thread.start()
    client = AsyncParameterClient(node, 'param_test_target')
    param_list = ['int_parameter', 'bool_parameter', 'string_parameter']

    node.get_logger().info('Listing Parameters:')
    future = client.list_parameters(param_list, 10)
    client_executor.spin_until_future_complete(future)
    list_parameters_result = future.result()
    if list_parameters_result is not None:
        for param_name in list_parameters_result.result.names:
            node.get_logger().info(f'    - {param_name}')
    else:
        node.get_logger().error(f'Error listing parameters: {future.exception()}')

    node.get_logger().info('Setting parameters:')
    future = client.set_parameters([
        Parameter('int_parameter', Parameter.Type.INTEGER, 10),
        Parameter('string_parameter', Parameter.Type.STRING, 'Fee Fi Fo Fum'),
    ])
    client_executor.spin_until_future_complete(future)
    set_parameters_result = future.result()
    if set_parameters_result is not None:
        for i, v in enumerate(set_parameters_result.results):
            node.get_logger().info(f'    {param_list[i]}:')
            node.get_logger().info(f'        successful: {v.successful}')
    else:
        node.get_logger().error(f'Error setting parameters: {future.exception()}')

    node.get_logger().info('Getting parameters:')
    future = client.get_parameters(param_list)
    client_executor.spin_until_future_complete(future)
    get_parameters_result = future.result()
    if get_parameters_result is not None:
        for i, v in enumerate(get_parameters_result.values):
            node.get_logger().info(f'    - {param_list[i]}: {parameter_value_to_python(v)}')
    else:
        node.get_logger().error(f'Error getting parameters: {future.exception()}')

    node.get_logger().info('Loading parameters: ')
    param_dir = get_package_share_directory('demo_nodes_py')
    param_list = os.path.join(param_dir, 'params.yaml')
    future = client.load_parameter_file(param_list)
    client_executor.spin_until_future_complete(future)
    load_parameter_results = future.result()
    if load_parameter_results is not None:
        param_file_dict = parameter_dict_from_yaml_file(param_list)
        for i, v in enumerate(param_file_dict.keys()):
            node.get_logger().info(f'    {v}:')
            node.get_logger().info(f'        value: '
                                   f'{parameter_value_to_python(param_file_dict[v].value)}')
    else:
        node.get_logger().error(f'Error loading parameters: {future.exception()}')

    node.get_logger().info('Deleting parameters: ')
    params_to_delete = ['other_int_parameter', 'other_string_parameter', 'string_parameter']
    future = client.delete_parameters(params_to_delete)
    client_executor.spin_until_future_complete(future)
    delete_parameters_result = future.result()
    if delete_parameters_result is not None:
        for i, v in enumerate(delete_parameters_result.results):
            node.get_logger().info(f'    {params_to_delete[i]}:')
            node.get_logger().info(f'        successful: {v.successful}')
            node.get_logger().info(f'        reason: {v.reason}')
    else:
        node.get_logger().error(f'Error deleting parameters: {future.exception()}')

    client_executor.shutdown()
    target_executor.shutdown()
    thread.join()
    rclpy.shutdown(context=context)


if __name__ == '__main__':
    main()
