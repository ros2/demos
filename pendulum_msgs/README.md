## **What Is This?**

The **pendulum_msgs** ROS 2 package is a dependency of **pendulum_control** ROS 2 package.
It contains `JointCommand.msg`, `JointState.msg` and `RttestResults.msg`

Please refer to [pendulum_control](https://github.com/ros2/demos/tree/rolling/pendulum_control) for more details.

### **JointCommand.msg**

```msg
float64 position
```

### **JointState.msg**

```msg
float64 position
float64 velocity
float64 effort
```

### **RttestResults.msg**

```msg
builtin_interfaces/Time stamp

JointCommand command
JointState state

uint64 cur_latency
float64 mean_latency
uint64 min_latency
uint64 max_latency
uint64 minor_pagefaults
uint64 major_pagefaults
```


## References

- Real-time Jitter Measurements under ROS 2: The Inverted Pendulum Case: https://www.researchgate.net/publication/350353690_Real-time_Jitter_Measurements_under_ROS2_the_Inverted_Pendulum_case
