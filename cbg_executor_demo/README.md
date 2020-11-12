# cbg_executor_demo

This package provides a small demo for the use of the Callback-group-level Executor concept. This concept was developed in 2018 and has been integrated in ROS 2 mainline in 2020, i.e. it is available in ROS 2 Rolling.

The Callback-group-level Executor leverages the callback-group concept by refining the Executor API to callback-group-level granularity. This allows a single node to have callbacks with different real-time requirements assigned to different Executor instances – within one process. Thus, an Executor instance can be dedicated to one or few specific callback groups and the Executor’s thread (or threads) can be prioritized according to the real-time requirements of these groups. For example, all time-critical callbacks may be handled by an "RT-CRITICAL" Executor instance running at the highest scheduler priority.

## Introduction to demo

As a proof of concept, we implemented a small demo and test bench in the present package *cbg_executor_demo*. The demo comprises a _Ping Node_ and a _Pong Node_ which exchange messages on two communication paths simultaneously. There are two topics _high\_ping_ and _high\_pong_ for the high priority path and _low\_ping_ and _low\_pong_ for the low priority path. Each class of messages is handled with a dedicated Executor, as illustrated in the following figure.

![](doc/ping_pong_diagram.png)

The Ping Node can be configured to send messages at a configured rate. The Pong Node takes these ping messages and replies each of them. Before sending the reply, it can be configured to burn cycles (thereby varying the processor load) to simulate some message processing.

The callbacks for the two paths are distributed to two different callback groups, which are again served by two different executors and threads. Both threads are pinned to the same CPU and thus share its processing power, but with different scheduler priorities following the names _high_ and _low_.

## Running the demo

After building the demo with [colcon](https://github.com/ros2/ros2/wiki/Colcon-Tutorial), the Ping and Pong Nodes may be either started in one process or in two processes. Please note that the demo requires sudo privileges to be able to change the thread priorities using `pthread_setschedparam(..)`.

Running the two nodes in one process:

```
sudo bash
source /opt/ros/rolling/setup.bash
source install/local_setup.bas
ros2 run cbg_executor_demo ping_and_pong_node
```

Running the two nodes in separate processes:

```
sudo bash
...
ros2 run cbg_executor_demo ping_node_only
```

```
sudo bash
...
ros2 run cbg_executor_demo pong_node_only
```

You should start the two processes simultaneously as the experiment runtime is just 10 seconds.

## Parameters

There are four parameters to configure the experiment:

* `high_ping_period` -- period (double value in seconds) for sending out pings on topic _high\_ping_ in the Ping Node.
* `low_ping_period` -- same for sending on _low\_ping_.
* `high_busyloop` -- duration (double value in seconds) for burning CPU cycles on receiving a message from _high\_ping_ in the Pong Node.
* `low_busyloop` -- same for receiving from _low\_ping_.

The default values are 0.01 seconds for all four parameters.

Example for changing the values on the command line:

```bash
ros2 run cbg_executor_demo ping_and_pong_node --ros-args -p high_busyloop:=0.001 -p high_ping_period:=0.1
```

## Implementation Details

TODO(Ralph): Rewrite this ...

The algorithms of the Ping node and of the Pong node are factored out into classes [_PingNode_](include/PingNode.hpp) and [_PongNode_](include/PongNode.hpp) - configurable with regard to the real-time profile and the topic prefix. Thus, the Ping node contains two instances of the _PingNode_ and the Pong node contains two instances of _PongNode_.

The PingNode contains a timer for sending the ping messages and a subscription for the corresponding pong messages. Also, it records the number of messages being sent and received and measures the roundtrip time.

The PongNode contains a subscription for the ping messages and a publisher for the corresponding pong messages. On receiving a ping message, it calls the `PongNode::burn_cpu_cycles()` functions to simulate a given processing time before replying with a pong.

The Ping and Pong nodes, the two executors, etc. are composed and configured in the `main(..)` function of [main.cpp](main.cpp). This function also starts and ends the experiment for a predefined duration and prints out the throughput and latency statistics.
