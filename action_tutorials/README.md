# ROS2 Actions Tutorial

This tutorial demonstrates implementing ROS action servers and action clients. The action server in this case takes in an integer value between 0 and 45 named *order* and returns the Fibonacci sequence, a sequence with the form: 
$$F_0 = 0$$
$$F_1 = 1$$
$$F_{order}=F_{order-1} + F_{order-2}$$

The action server calculates each number in the sequence one at a time and at returns a partial sequence as feedback at each iteration. If the action is cancelled before the entire sequence is calculated, a partial sequence will be returned.

The action client in this tutorial sends a goal to the action server with an order of 10. It logs each partial sequence returned as feedback. Once the action is finished executing, the action client logs the resulting sequence.
