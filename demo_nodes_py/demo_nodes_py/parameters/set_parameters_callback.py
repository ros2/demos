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

from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter


# Example usage: changing param1 successfully will result in setting of param2.
# ros2 service call /set_parameters_callback/set_parameters rcl_interfaces/srv/SetParameters
#       "{parameters: [{name: "param1", value: {type: 3, double_value: 1.0}}]}"

# node for demonstrating correct usage of pre_set, on_set
# and post_set parameter callbacks
class SetParametersCallback(Node):

    def __init__(self):
        super().__init__('set_parameters_callback')

        # tracks 'param1' value
        self.internal_tracked_param_1 = self.declare_parameter('param1', 0.0).value
        # tracks 'param2' value
        self.internal_tracked_param_2 = self.declare_parameter('param2', 0.0).value

        # setting another parameter from the callback is possible
        # we expect the callback to be called for param2
        def pre_set_parameter_callback(parameter_list):
            modified_parameters = parameter_list.copy()
            for param in parameter_list:
                if param.name == 'param1':
                    modified_parameters.append(Parameter('param2', Parameter.Type.DOUBLE, 4.0))

            return modified_parameters

        # validation callback
        def on_set_parameter_callback(parameter_list):
            result = SetParametersResult()
            for param in parameter_list:
                if param.name == 'param1':
                    result.successful = True
                    result.reason = 'success param1'
                elif param.name == 'param2':
                    result.successful = True
                    result.reason = 'success param2'

            return result

        # can change internally tracked class attributes
        def post_set_parameter_callback(parameter_list):
            for param in parameter_list:
                if param.name == 'param1':
                    self.internal_tracked_param_1 = param.value
                elif param.name == 'param2':
                    self.internal_tracked_param_2 = param.value

        self.add_pre_set_parameters_callback(pre_set_parameter_callback)
        self.add_on_set_parameters_callback(on_set_parameter_callback)
        self.add_post_set_parameters_callback(post_set_parameter_callback)


def main(args=None):
    rclpy.init(args=args)

    node = SetParametersCallback()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
