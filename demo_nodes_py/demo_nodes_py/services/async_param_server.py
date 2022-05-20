import rclpy
from rclpy import Parameter
from rclpy.parameter_client_async import AsyncParameterClient


def main(args=None):
    rclpy.init(args=args)
    target_node = rclpy.create_node('param_test_target')

    target_node.declare_parameter('zero', 1)
    target_node.declare_parameter('one', 0)
    target_node.declare_parameter('zero/one', 10)
    target_node.declare_parameter('one/two', 12)
    target_node.declare_parameter('true', False)
    target_node.declare_parameter('string string', "string")

    rclpy.spin(target_node)
    target_node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()















































