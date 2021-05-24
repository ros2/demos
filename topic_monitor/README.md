# Visualizing the effect of QoS policies

This demo uses a “topic monitor” that can be used to visualize the statistics of ROS 2 topics that are publishing sequential data.

## Background
Please read the [About Quality of Service Settings](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html) page for background information about the Quality of Service settings available in ROS 2.

## Running the demo
To visualize the reception rate, we will use a “topic monitor” that periodically calculates the reception rate of topics with publishers of periodic, sequential data.

If you have the Python3 `matplotlib` and `tkinter` packages installed, you can use the `--display` option to plot the reception rate of topics:
```
ros2 run topic_monitor topic_monitor --display
```

**For all invocations that follow make sure the same `ROS_DOMAIN_ID` has been set to the same value on the respective machines.**


### Comparing reliability QoS settings
You will need two machines with ROS 2: one mobile and one stationary.

1. Run the `ros2 launch topic_monitor reliability_demo.launch.py` executable on the stationary machine.
This will start two nodes: one publishing in “reliable” mode, and one in “best effort”.
1. Start the monitor on a mobile machine such as a laptop.
Use `ros2 run topic_monitor topic_monitor --display --allowed-latency 5` to account for any latency that may occur re-sending the reliable messages.
1. Take the mobile machine out of range of the monitor, and observe how the reception rates differ for the different topics.

You should see that the "reliable" topic has a reception rate that is almost always either 0 or 100%, while the "best effort" topic has a reception rate that fluctuates based on the strength of the connection.

Here’s an example plot:

![reception rates plot](https://github.com/ros2/demos/raw/master/topic_monitor/doc/reliability_comparison.png "Sample plot of reception rates")

#### Comparison with ROS 1
The behavior shown here is comparable to the difference between that of TCPROS and UDPROS.
One difference is that even the “reliable” reliability in ROS 2 uses UDP underneath, which allows for a range of behavior in between that of TCP and UDP.
Another key difference is that UDPROS has currently only been implemented in the C++ ROS 1 client library, and is not an option for ROS 1 nodes written in Python.
By contrast, the QoS settings available in ROS 2 are implemented in a core library that language-specific client libraries then make use of, meaning that these features only have to be implemented once and then just exposed through the different language interfaces.
See [ROS 2 Client Libraries](https://docs.ros.org/en/rolling/Concepts/About-ROS-2-Client-Libraries.html) for more information.


### Comparing the latency of reliability QoS settings
Repeat the previous demo with a lower allowed latency on the topic monitor.
That is, in step 2, run `ros2 run topic_monitor topic_monitor --display --allowed-latency 1`.

You should see that the “best effort” reception rate is unaffected, while the “reliable” reception rate drops sooner when the machines are going out of range of each other.
This is because a “reliable” message may be re-sent multiple times in order to be delivered.

### Comparing the effect of queue depth
As you saw in the previous demo, “reliable” data can take a while to be successfully sent if the network connection is not robust.
If calls to `publish` are being made faster than the data can be acknowledged by the subscriber, the queue depth comes into play.
A queue depth of 10 (the default) means that 10 messages will be kept in the publisher’s buffer before they get overwritten.
If the 10th latest message has not been successfully acknowledged by the time the next message is published, it will be overwritten and the subscriber will not receive it, causing the reception rate to decline.

You will need two machines with ROS 2: one mobile and one stationary.

1. Run the `ros2 launch topic_monitor depth_demo.launch.py` executable on the stationary machine.
This will start some nodes publishing both small (payload of 1-character string) and large data (payload of 100,000-character string) with different depths: a pair publishing with a depth of 1, and a pair with a larger depth (50).
1. Start the monitor on a mobile machine such as a laptop with `ros2 run topic_monitor topic_monitor --display`.
1. Take the mobile machine out of range of the monitor, and observe how the reception rates differ for the different topics.

You should see that the reception rate of the publishers with the higher depth is better than those with the depth of 1, because messages are less likely to get overwritten when data is taking longer to be acknowledged.


### Comparing the effect of data size
The maximum message size in UDP (the underlying transport for ROS 2) is 64KB, after which messages get fragmented into smaller parts and sent individually.
Larger message requires multiple fragments to be sent, and unless all fragments are received, the reception rate of the data declines.
For connections with "reliable" reliability policy, lost fragments will be re-sent.
For connections with "best effort" reliability, the loss of any fragment causes the whole message to be discarded by the subscriber.

You will need two machines running ROS 2: one stationary and one mobile.

**Warning**: this demo causes a lot of network traffic.
You should not use the Fast RTPS ROS middleware implementation for this part.
See [DDS and ROS Middleware Implementations](https://docs.ros.org/en/rolling/Concepts/About-Different-Middleware-Vendors.html) for instructions on how to change the vendor.

1. Run the `ros2 launch topic_monitor fragmentation_demo.launch.py` executable on the stationary machine.
1. Run `ros2 run topic_monitor topic_monitor --display --expected-period 4` on the mobile machine.
This will launch four publishers publishing messages of strings of different lengths: small (1), medium (50000), large (100000) and xlarge (150000).
1. Take the mobile machine out of range of the monitor, and observe how the reception rates differ for the different topics.

You should see that the difference between the reception rate of the small and medium message sizes not the same as that for the medium and large message sizes.
The large message requires two fragments to send it (the xlarge message three), and unless both fragments are received, the reception rate goes down.
