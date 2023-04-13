# ROS 2 Action Tutorials

This tutorial demonstrates implementing ROS action servers and action clients.

The action server in this case takes in an integer value between 0 and 45 named *order* and returns the Fibonacci sequence, a sequence with the form:
$$F_0 = 0$$
$$F_1 = 1$$
$$F_{order}=F_{order-1} + F_{order-2}$$

The action server calculates each number in the sequence one at a time and returns a partial sequence as feedback at each iteration.
If the action is cancelled before the entire sequence is calculated, the server stops calculating the sequence and no result is returned.
The action client in this tutorial sends a goal to the action server with an order of 10.
It logs each partial sequence returned as feedback.
Once the action is finished executing, the action client logs the resulting sequence.

## Packages

- [action_tutorials_cpp](./action_tutorials_cpp) implements the described action server and client using the rclcpp library in C++.
- [action_tutorials_py](./action_tutorials_py) implements the described action server and client using the rclpy library in Python.
- [action_tutorials_interfaces](./action_tutorials_interfaces) defines the interface for the Fibonacci action.
This interface takes an *order* as a goal, returns a *partial sequence* as feedback and a *sequence* as a result.
