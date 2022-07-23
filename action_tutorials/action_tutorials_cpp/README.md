# ROS2 Actions Tutorial C++

This tutorial demonstrates implementing ROS action servers and action clients in C++. The action server in this case takes in an integer value between 0 and 45 named *order* and returns the Fibonacci sequence, a sequence with the form: 
$$F_0 = 0$$
$$F_1 = 1$$
$$F_{order}=F_{order-1} + F_{order-2}$$
