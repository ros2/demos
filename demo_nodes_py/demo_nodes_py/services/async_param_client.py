import rclpy
from rclpy import Parameter
from rclpy.parameter_client_async import AsyncParameterClient

def foo(val):
    print('callback function called')


def main(args=None):
    rclpy.init()
    target_node_name = 'param_test_target'
    client = AsyncParameterClient(target_node_name)

    print('----------------------- List Parameters ----------------------')

    future = client.list_parameters(["zero", "one", "true"], 10, callback=foo)
    rclpy.spin_until_future_complete(client.node, future)
    initial_parameters = future.result()
    if initial_parameters:
        client.node.get_logger().info(f"Parameters: {initial_parameters}")
    else:
        client.node.get_logger().info(f"Error listing parameters: {future.exception()}")

    print('----------------------- Get Parameters ----------------------')

    params_to_get = ['zero/one', 'one', 'string string']
    future = client.get_parameters(params_to_get, callback=foo)
    rclpy.spin_until_future_complete(client.node, future)
    
    parameter_values = future.result()
    if parameter_values:
        client.node.get_logger().info(f"Parameters: {parameter_values}")
    else:
        client.node.get_logger().info(f"Error getting parameters: {future.exception()}")

    # future = client.set_parameters([
    #         Parameter("zero", Parameter.Type.INTEGER, 0),
    #         Parameter("one", Parameter.Type.INTEGER, 1),
    #         Parameter("true", Parameter.Type.BOOL, True),
    #         Parameter("string string", Parameter.Type.STRING, "string string")
    #     ]) 
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()















































