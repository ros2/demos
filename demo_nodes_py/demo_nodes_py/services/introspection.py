# Copyright 2023 Open Source Robotics Foundation, Inc.
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

from example_interfaces.srv import AddTwoInts
from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rclpy.service_introspection import ServiceIntrospectionState


# This demo program shows how to configure client and service introspection
# on the fly, by hooking it up to a parameter.  This program consists of both
# a client node (IntrospectionClientNode) and a service node (IntrospectionServiceNode).
# At startup time, one of each type of node is created, and added to an executor.
# The IntrospectionServiceNode listens on the '/add_two_ints' service for clients;
# when one connects and sends a request, it adds the two integers and returns the result.
# The IntrospectionClientNode is slightly more complicated.  It has a timer callback that
# runs every 500 milliseconds.  If the service is not yet ready, no further work
# is done.  If the client doesn't currently have a future outstanding, then it
# creates a new AddTwoInts service request, and asynchronously sends it to the service.
# If the client does have a future outstanding, then we check if it is done.
# Once the future has been completed, we forget about the future so we can send
# another request.
#
# The above is a fairly common ROS 2 client and service, but what this program
# is trying to demonstrate is introspection capabilities.
# The IntrospectionClientNode has a string parameter called 'client_configure_introspection',
# and the IntrospectionServiceNode has a string parameter called 'service_configure_introspection'.
# If these are set to 'disabled' (the default), then no introspection happens.
# If these are set to 'metadata' (see details on how to set the parameters below),
# then essential metadata (timestamps, sequence numbers, etc) are sent to a
# hidden topic called /add_two_ints/_service_event.
#
# To see this in action, this program can be run in a couple of different ways:
#
# ros2 run demo_nodes_py introspection
#   Since the default for introspection is 'disabled', this is no different than
#   a normal client and server.  No additional topics will be made, and
#   no introspection data will be sent.  However, changing the introspection
#   configuration dynamically is fully supported.  This can be seen by
#   running 'ros2 param set /introspection_client client_configure_introspection metadata'
#   which will configure the client to start sending service introspection
#   metadata to /add_two_ints/_service_event.
#
# ros2 run demo_nodes_py introspection \
#     --ros-args -p client_configure_introspection:=metadata \
#     -p service_configure_introspection:=metadata
#   Here we've set both the client and service introspection to metadata,
#   so the /add_two_ints/_service_event will be created.  Additionally,
#   every client request and response and every service acceptance and
#   response will be sent to that topic.
#
# In either case, service introspection data can be seen by running:
#   ros2 topic echo /add_two_ints/_service_event

def check_parameter(parameter_list, parameter_name):
    result = SetParametersResult()
    result.successful = True
    for param in parameter_list:
        if param.name != parameter_name:
            continue

        if param.type_ != Parameter.Type.STRING:
            result.successful = False
            result.reason = 'must be a string'
            break

        if param.value not in ('disabled', 'metadata', 'contents'):
            result.successful = False
            result.reason = "must be one of 'disabled', 'metadata', or 'contents"
            break

    return result


class IntrospectionClientNode(Node):

    def on_set_parameters_callback(self, parameter_list):
        return check_parameter(parameter_list, 'client_configure_introspection')

    def on_post_set_parameters_callback(self, parameter_list):
        for param in parameter_list:
            if param.name != 'client_configure_introspection':
                continue

            introspection_state = ServiceIntrospectionState.OFF
            if param.value == 'disabled':
                introspection_state = ServiceIntrospectionState.OFF
            elif param.value == 'metadata':
                introspection_state = ServiceIntrospectionState.METADATA
            elif param.value == 'contents':
                introspection_state = ServiceIntrospectionState.CONTENTS

            self.cli.configure_introspection(self.get_clock(), qos_profile_system_default,
                                             introspection_state)
            break

    def __init__(self):
        super().__init__('introspection_client')

        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        self.add_on_set_parameters_callback(self.on_set_parameters_callback)
        self.add_post_set_parameters_callback(self.on_post_set_parameters_callback)
        self.declare_parameter('client_configure_introspection', 'disabled')

        self.timer = self.create_timer(0.5, self.timer_callback)
        self.future = None

    def timer_callback(self):
        if not self.cli.service_is_ready():
            return

        if self.future is None:
            req = AddTwoInts.Request()
            req.a = 2
            req.b = 3

            self.future = self.cli.call_async(req)

            return

        if not self.future.done():
            return

        if self.future.result() is not None:
            self.get_logger().info('Result of add_two_ints: %d' % self.future.result().sum)
        else:
            self.get_logger().error('Exception calling service: %r' % self.future.exception())

        self.future = None


class IntrospectionServiceNode(Node):

    def on_set_parameters_callback(self, parameter_list):
        return check_parameter(parameter_list, 'service_configure_introspection')

    def on_post_set_parameters_callback(self, parameter_list):
        for param in parameter_list:
            if param.name != 'service_configure_introspection':
                continue

            introspection_state = ServiceIntrospectionState.OFF
            if param.value == 'disabled':
                introspection_state = ServiceIntrospectionState.OFF
            elif param.value == 'metadata':
                introspection_state = ServiceIntrospectionState.METADATA
            elif param.value == 'contents':
                introspection_state = ServiceIntrospectionState.CONTENTS

            self.srv.configure_introspection(self.get_clock(), qos_profile_system_default,
                                             introspection_state)
            break

    def __init__(self):
        super().__init__('introspection_service')

        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        self.add_on_set_parameters_callback(self.on_set_parameters_callback)
        self.add_post_set_parameters_callback(self.on_post_set_parameters_callback)
        self.declare_parameter('service_configure_introspection', 'disabled')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    service_node = IntrospectionServiceNode()

    client_node = IntrospectionClientNode()

    executor = SingleThreadedExecutor()
    executor.add_node(service_node)
    executor.add_node(client_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        executor.remove_node(client_node)
        executor.remove_node(service_node)
        executor.shutdown()
        service_node.destroy_node()
        client_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
