# Topic monitor demo

This demo provides a “topic monitor” that can be used to visualize the statistics of ROS 2 topics that are publishing sequential data.

### How to start publishers

To start one such publisher, assuming you have already installed ROS 2 and sourced your setup file, run:
```
topic_monitor_data_publisher critical
```
This will start a ROS 2 publisher on the topic `critical_data` that will use the default QoS profile, which includes “reliable” reliability settings. A message of type `std_msgs/Int64` will be sent periodically, with incrementing values that act as sequence numbers. When the topic monitor receives one of these messages, it will maintain an internal counter that is used to keep track of what value should be being received from the publisher. This enables the “reception rate” of the topic to be calculated based on how many of the expected values have been received from the publisher. For a publisher that is communicating with the topic monitor with “reliable” reliability settings, the reception rate is expected to be 100%, even if the topic monitor is running on a different machine, unless that machine is out-of-range of the topic monitor.

To start a publisher with “best effort” reliability settings, run:
```
topic_monitor_data_publisher sensor --best-effort
```
This will start a ROS 2 publisher on the topic `sensor_data_best_effort` that will use the “best effort” setting for the QoS reliability policy. This publisher will not check if sent data is acknowledged by any subscriptions, and therefore the reception rate may drop below 100% if the network is congested or there is a poor connection between the monitor and the publisher.

When the data publisher script is terminated with `Ctrl + C`, the publisher will attempt to send `-1` as a “last breath” message that signals the topic monitor that it is going offline gracefully. Otherwise, if the topic monitor doesn't receive a message from a publisher for a pre-determined amount of time, it will consider the topic "stale".

For a full list of options for the data publisher, type `topic_monitor_data_publisher --help`.

### How to start the topic monitor

To start the topic monitor, run:
```
topic_monitor
```

This will start a topic monitor that will subscribe to any ROS 2 topics that match the structure “<name>_data” or “<name>_data_best_effort”, where <name> can be something such as “topic1”. It will subscribe to the topics with matching QoS settings (best effort in the case of best effort, and vice-versa).
Monitored topics have the following properties monitored:
- The status: “Alive” if a message has been received from the topic in the last `stale-time` (`s`) seconds, otherwise it is “Stale”. If a message of `-1` is received, the topic is considered “Offline” which represents that the topic has been gracefully disconnected as opposed to lost.
- The reception rate: If a topic is not “Offline”, its reception rate will be calculated based on the last `window-size` (`n`) values that are expected to have been received. The expected values are a sequence of increasing numbers every `expected-period` (`t`) seconds. To account for latency, `allowed-latency` (`l`) seconds are allowed between expected and received values. The reception rate is calculated every `stats-calc-period` (`c`) seconds.


The properties of the topic monitor can be modified from the command line with the following options:
```
$ topic_monitor --help
usage: topic_monitor [-h] [-d] [-t [EXPECTED_PERIOD]] [-s [STALE_TIME]]
                     [-l [ALLOWED_LATENCY]] [-c [STATS_CALC_PERIOD]]
                     [-n [WINDOW_SIZE]]

optional arguments:
  -h, --help            show this help message and exit
  -d, --display         Display the reception rate of topics (requires
                        matplotlib) (default: False)
  -t [EXPECTED_PERIOD], --expected-period [EXPECTED_PERIOD]
                        Expected time in seconds between received messages on
                        a topic (default: 0.5)
  -s [STALE_TIME], --stale-time [STALE_TIME]
                        Time in seconds without receiving messages before a
                        topic is considered stale (default: 1.0)
  -l [ALLOWED_LATENCY], --allowed-latency [ALLOWED_LATENCY]
                        Allowed latency in seconds between receiving expected
                        messages (default: 1.0)
  -c [STATS_CALC_PERIOD], --stats-calc-period [STATS_CALC_PERIOD]
                        Time in seconds between calculating topic statistics
                        (default: 1.0)
  -n [WINDOW_SIZE], --window-size [WINDOW_SIZE]
                        Number of messages in calculation of topic statistics
                        (default: 20)
```


If you have the Python3 `matplotlib` package installed, you can run the following to start a plot that will show the reception rate every `stats-calc-period` seconds:
```
topic_monitor --display
```
Alternatively, if you have ROS 1 installed, you can use the ROS 1 - ROS 2 bridge to plot the reception rate using ROS 1 tools such as `rqt`, or log it using `rosbag`. Be sure to run the bridge with `--bridge-all-2to1-topics` so that all topics will be bridged, that way `rqt` will be able to list the topics before it has subscribed to them.


## Run some experiments
For all invocations that follow make sure the same `ROS_DOMAIN_ID` has been set.

### Comparing reliability QoS settings
You will need two machines with ROS 2: one mobile and one stationary.

1. Run the `topic_monitor_launch_reliability_demo` executable on the stationary machine. This will start two nodes: one publishing in “reliable” mode, and one in “best effort”.
1. Start the monitor on a mobile machine such as a laptop. Use `topic_monitor --display --allowed-latency 5` to account for any latency that may occur re-sending the reliable messages.
1. Take the mobile machine out of range of the monitor, and observe how the reception rates differ for the different topics.
You should see something like this:

![reception rates plot](https://github.com/ros2/demos/raw/multi_robot_monitor/topic_monitor/doc/reliability_comparison.png "Sample plot of reception rates")

Note that the "reliable" topic has a reception rate that is almost always either 0 or 100%, while the "best effort" topic has a reception rate that fluctuates based on the strength of the connection.

### Comparing durability QoS settings
You will need two terminals running ROS 2 (on the same machine or different machines).

1. Run the `topic_monitor_launch_durability_demo` executable in one terminal. This will start two publishers: one with the "volatile" durability setting, and one with "transient local".
1. Wait 10 seconds.
1. Run `topic_monitor --display --expected-period 0.1` to start the topic monitor.

You should see that the volatile publisher starts with a reception rate of 0, as it does not keep any messages for the late-starting topic monitor, while the transient local publisher starts with a reception rate of 1.0 as it sends queued messages to the topic monitor once it starts up.

### Comparing the effect of data size
You will need two machines running ROS 2: one stationary and one mobile.
**Warning**: this demo causes a lot of network traffic.

1. Run the `topic_monitor_launch_fragmentation_demo` executable on the stationary machine.
1. Run `topic_monitor --display -expected-period 4` on the mobile machine. This will launch four publishers publishing messages of strings of different lengths: small (1), medium (50000), large (100000) and xlarge (150000).
1. Take the mobile machine out of range of the monitor, and observe how the reception rates differ for the different topics.
You should see something like this:

![message size reception rates plot](https://github.com/ros2/demos/raw/multi_robot_monitor/topic_monitor/doc/message_size_comparison.png "Sample plot of reception rates when comparing message size")

Why is the difference between the small and medium message sizes not the same as that for the medium and large message sizes? The maximum message size in UDP (the underlying transport) is 64KB, after which messages get fragmented into smaller parts and sent individually. The large message requires two fragments to send it (the xlarge message three), and unless both fragments are received, the reception rate goes down. As the publishers are publishing with "reliable" reliability policy, when the two machines are out of range it will try to resend any lost fragments: if they were publishing in "best effort", the loss of any fragment would cause the whole message to be discarded by the subscription.

