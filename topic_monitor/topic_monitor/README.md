# Topic monitor demo

The demo is written using the `rclpy` Python client library.
**Please note that the `rclpy` client library API is currently under heavy development and likely to change.**

Some launch files are provided for comparing different QoS policies.

### How to start publishers

To start one such publisher, assuming you have already installed ROS 2 and sourced your setup file, run:
```
ros2 run topic_monitor data_publisher -- critical
```
This will start a ROS 2 publisher on the topic `critical_data` that will use the default QoS profile, which includes “reliable” reliability settings.
A message will be sent periodically, with incrementing values that act as sequence numbers.
When the topic monitor receives one of these messages, it will maintain an internal counter that is used to keep track of what value should be being received from the publisher.
This enables the “reception rate” of the topic to be calculated based on how many of the expected values have been received from the publisher.

To start a publisher with “best effort” reliability settings, run:
```
ros2 run topic_monitor data_publisher -- sensor --best-effort
```
This will start a ROS 2 publisher on the topic `sensor_data_best_effort` that will use the “best effort” setting for the QoS reliability policy.
This publisher will not check if sent data is acknowledged by any subscriptions, and therefore the reception rate may drop below 100% if the network is congested or there is a poor connection between the monitor and the publisher.

When the data publisher script is terminated with `Ctrl + C`, the publisher will attempt to send `-1` as a “last breath” message that signals the topic monitor that it is going offline gracefully.
Otherwise, if the topic monitor doesn't receive a message from a publisher for a pre-determined amount of time, it will consider the topic "stale".

For a full list of options for the data publisher, type `ros2 run topic_monitor data_publisher -- --help`.

### How to start the topic monitor

To start the topic monitor, run:
```
ros2 run topic_monitor topic_monitor
```

This will start a topic monitor that will subscribe to any ROS 2 topics that match the structure “<name>_data” or “<name>_data_best_effort”, where <name> can be something such as “topic1”.
It will subscribe to the topics with matching QoS settings (best effort in the case of best effort, and vice-versa).

Monitored topics have the following properties monitored:
- The status: “Alive” if a message has been received from the topic in the last `stale-time` (`s`) seconds, otherwise it is “Stale”.
If a message of `-1` is received, the topic is considered “Offline” which represents that the topic has been gracefully disconnected as opposed to lost.

- The reception rate: If a topic is not “Offline”, its reception rate will be calculated based on the last `window-size` (`n`) values that are expected to have been received.
The expected values are a sequence of increasing numbers every `expected-period` (`t`) seconds.
To account for latency, `allowed-latency` (`l`) seconds are allowed between expected and received values.
The reception rate is calculated every `stats-calc-period` (`c`) seconds.

The properties of the topic monitor can be modified from the command line. Run `topic_monitor --help` for details.
