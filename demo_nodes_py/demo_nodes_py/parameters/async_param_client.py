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
    param_file = ['int_parameter', 'bool_parameter', 'string_parameter']

    node.get_logger().info('Listing Parameters:')
    future = client.list_parameters(param_file, 10)
    client_executor.spin_until_future_complete(future)
    initial_parameters = future.result()
    if initial_parameters:
        for param_name in initial_parameters.result.names:
            node.get_logger().info(f'    - {param_name}')
    else:
        node.get_logger().error(f'Error listing parameters: {future.exception()}')

    node.get_logger().info('Getting parameters:')
    future = client.get_parameters(param_file)
    client_executor.spin_until_future_complete(future)
    parameter_values = future.result()
    if parameter_values:
        for i, v in enumerate(parameter_values.values):
            node.get_logger().info(f'    - {param_file[i]}: {parameter_value_to_python(v)}')
    else:
        node.get_logger().error(f'Error getting parameters: {future.exception()}')

    node.get_logger().info('Setting parameters:')
    # We can do this since client.set_parameters takes
    # List[Union[rclpy.parameter.Parameter, rcl_interfaces.msg.Parameter]]
    future = client.set_parameters([
        Parameter('int_parameter', Parameter.Type.INTEGER, 10).to_parameter_msg(),
        Parameter('string_parameter', Parameter.Type.STRING, 'Fee Fi Fo Fum'),
        Parameter('bool_parameter', Parameter.Type.BOOL, True),
    ])
    client_executor.spin_until_future_complete(future)
    parameter_values = future.result()
    if parameter_values:
        for i, v in enumerate(parameter_values.results):
            node.get_logger().info(f'    {param_file[i]}:')
            node.get_logger().info(f'        successful: {v.successful}')
    else:
        node.get_logger().error(f'Error setting parameters: {future.exception()}')

    node.get_logger().info('Getting parameters:')
    future = client.get_parameters(param_file)
    client_executor.spin_until_future_complete(future)
    parameter_values = future.result()
    if parameter_values:
        for i, v in enumerate(parameter_values.values):
            node.get_logger().info(f'    {param_file[i]}: {parameter_value_to_python(v)}')
    else:
        node.get_logger().error(f'Error getting parameters: {future.exception()}')

    node.get_logger().info('Describing parameters:')
    future = client.describe_parameters(param_file)
    client_executor.spin_until_future_complete(future)
    parameter_descriptions = future.result()
    if parameter_descriptions:
        for i, v in enumerate(parameter_descriptions.descriptors):
            node.get_logger().info(f'    {param_file[i]}:')
            for s in v.__slots__:
                node.get_logger().info(f'        {s}: {getattr(v, s)}')
    else:
        node.get_logger().error(f'Error describing parameters: {future.exception()}')

    node.get_logger().info('Getting parameter types:')
    future = client.get_parameter_types(param_file)
    client_executor.spin_until_future_complete(future)
    parameter_types = future.result()
    if parameter_types:
        for i, v in enumerate(parameter_types.types):
            node.get_logger().info(f'    {param_file[i]}: {v}')
    else:
        node.get_logger().error(f'Error getting parameter types: {future.exception()}')

    node.get_logger().info('Setting parameters atomically:')
    future = client.set_parameters_atomically([
        Parameter('zero/one', Parameter.Type.INTEGER, 99),
        Parameter('one', Parameter.Type.INTEGER, 2),
    ])
    client_executor.spin_until_future_complete(future)
    set_resuts = future.result()
    if set_resuts:
        node.get_logger().info('    Set parameters atomically:')
        node.get_logger().info(f'        {set_resuts.result.successful}')
    else:
        node.get_logger().error(f'Error setting parameters: {future.exception()}')

    node.get_logger().info('Loading parameters: ')
    param_dir = get_package_share_directory('demo_nodes_py')
    param_file = os.path.join(param_dir, 'params.yaml')
    future = client.load_parameter_file(param_file)
    client_executor.spin_until_future_complete(future)
    load_results = future.result()
    if load_results:
        param_file_dict = parameter_dict_from_yaml_file(param_file)
        for i, v in enumerate(param_file_dict.keys()):
            node.get_logger().info(f'    {v}:')
            node.get_logger().info(f'        successful: {load_results.results[i].successful}')
            node.get_logger().info(f'        value: '
                                   f'{parameter_value_to_python(param_file_dict[v].value)}')
    else:
        node.get_logger().error(f'Error loading parameters: {future.exception()}')

    node.get_logger().info('Deleting parameters: ')
    params_to_delete = ['other_int_parameter', 'other_string_parameter', 'string_parameter']
    future = client.delete_parameters(params_to_delete)
    client_executor.spin_until_future_complete(future)
    delete_results = future.result()
    if delete_results:
        for i, v in enumerate(delete_results.results):
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
