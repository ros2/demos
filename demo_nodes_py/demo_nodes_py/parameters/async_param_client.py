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

from ament_index_python import get_package_share_directory
import rclpy
import rclpy.context
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.parameter import parameter_dict_from_yaml_file
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient


def main(args=None):
    try:
        with rclpy.init():
            node = rclpy.create_node('async_param_client')
            client = AsyncParameterClient(node, 'parameter_blackboard')
            if not client.wait_for_services(3.0):
                raise RuntimeError(
                    'parameter_blackboard is not available, start parameter blackboard with '
                    '`ros2 run demo_nodes_cpp parameter_blackboard` before running this demo')
            param_list = ['int_parameter', 'bool_parameter', 'string_parameter']

            logger = node.get_logger()

            logger.info('Setting parameters:')
            future = client.set_parameters([
                Parameter('int_parameter', Parameter.Type.INTEGER, 10),
                Parameter('bool_parameter', Parameter.Type.BOOL, False),
                Parameter('string_parameter', Parameter.Type.STRING, 'Fee Fi Fo Fum'), ])
            rclpy.spin_until_future_complete(node, future)
            set_parameters_result = future.result()
            if set_parameters_result is not None:
                for i, v in enumerate(set_parameters_result.results):
                    logger.info(f'    {param_list[i]}:')
                    logger.info(f'        successful: {v.successful}')
            else:
                logger.error(f'Error setting parameters: {future.exception()}')

            logger.info('Listing Parameters:')
            future = client.list_parameters(param_list, 10)
            rclpy.spin_until_future_complete(node, future)
            list_parameters_result = future.result()
            if list_parameters_result is not None:
                for param_name in list_parameters_result.result.names:
                    logger.info(f'    - {param_name}')
            else:
                logger.error(f'Error listing parameters: {future.exception()}')

            logger.info('Getting parameters:')
            future = client.get_parameters(param_list)
            rclpy.spin_until_future_complete(node, future)
            get_parameters_result = future.result()
            if get_parameters_result is not None:
                for i, v in enumerate(get_parameters_result.values):
                    logger.info(f'    - {param_list[i]}: {parameter_value_to_python(v)}')
            else:
                logger.error(f'Error getting parameters: {future.exception()}')

            logger.info('Loading parameters: ')
            param_dir = get_package_share_directory('demo_nodes_py')
            param_file_path = os.path.join(param_dir, 'params.yaml')
            future = client.load_parameter_file(param_file_path)
            rclpy.spin_until_future_complete(node, future)
            load_parameter_results = future.result()
            if load_parameter_results is not None:
                param_file_dict = parameter_dict_from_yaml_file(
                    param_file_path, False, target_nodes=['parameter_blackboard'])
                for i, v in enumerate(param_file_dict.keys()):
                    logger.info(f'    {v}:')
                    logger.info(f'        successful: '
                                f'{load_parameter_results.results[i].successful}')
                    logger.info(f'        value: '
                                f'{parameter_value_to_python(param_file_dict[v].value)}')
            else:
                logger.error(f'Error loading parameters: {future.exception()}')

            logger.info('Deleting parameters: ')
            params_to_delete = ['other_int_parameter', 'other_string_parameter',
                                'string_parameter']
            future = client.delete_parameters(params_to_delete)
            rclpy.spin_until_future_complete(node, future)
            delete_parameters_result = future.result()
            if delete_parameters_result is not None:
                for i, v in enumerate(delete_parameters_result.results):
                    logger.info(f'    {params_to_delete[i]}:')
                    logger.info(f'        successful: {v.successful}')
                    logger.info(f'        reason: {v.reason}')
            else:
                logger.error(f'Error deleting parameters: {future.exception()}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
