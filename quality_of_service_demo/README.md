# Quality of Service Demos

The demos in this package show the various Quality of Service settings available in ROS 2.
The intent is to provide a dedicated application for each QoS type, in order to highlight that single feature in isolation.

Each demo is implemented in C++ and Python.
This document provides instructions for running the demos in Python, but you can easily try the C++ demos by replacing the package name in each command from `quality_of_service_demo_py` to `quality_of_service_demo_cpp`.

## History

No History QoS demo app exists yet in this package.

## Reliability

No Reliability QoS demo app exists yet in this package.

## Durability

No Durability QoS demo app exists yet in this package.

## Deadline

This demo shows how deadline can be used to enforce a maximum duration between messages.

The demo creates a talker and listener with the provided deadline duration (in milliseconds).
The talker will go through periods of publishing messages regularly and pausing publishing.

When the talker pauses publishing, you should see logs from both the talker and listener that the deadline has been missed.

For usage, run `ros2 run quality_of_service_demo_py deadline -h`

Examples:
* `ros2 run quality_of_service_demo_cpp deadline 600 --publish-for 5000 --pause-for 2000` _or_
* `ros2 run quality_of_service_demo_py deadline 600 --publish-for 5000 --pause-for 2000`
  * The publisher will publish for 5 seconds, then pause for 2 - deadline events will start firing on both participants, until the publisher comes back.
* `ros2 run quality_of_service_demo_cpp deadline 600 --publish-for 5000 --pause-for 0` _or_
* `ros2 run quality_of_service_demo_py deadline 600 --publish-for 5000 --pause-for 0`
  * The publisher doesn't actually pause, no deadline misses should occur.

## Lifespan

This demo shows how lifespan can be used to drop old messages from a publisher's outgoing queue.

The demo creates a talker and listener with the provided lifespan duration (in milliseconds).
The talker will publish a preset number of messages, then stop publishing.
The listener will subscribe after a preset duration (in milliseconds).
The number of messages the listener receieves depends on the the lifespan setting and how long the listener takes to subscribe.

For usage, run `ros2 run quality_of_service_demo_py lifespan -h`

Examples:
* `ros2 run quality_of_service_demo_cpp lifespan 1000 --publish-count 10 --subscribe-after 3000` _or_
* `ros2 run quality_of_service_demo_py lifespan 1000 --publish-count 10 --subscribe-after 3000`
  * After a few seconds, you should see (approximately) messages 4-9 printed from the subscriber.
  * The first few messages, with 1 second lifespan, were gone by the time the subscriber joined after 3 seconds.
* `ros2 run quality_of_service_demo_cpp lifespan 4000 --publish-count 10 --subscribe-after 3000` _or_
* `ros2 run quality_of_service_demo_py lifespan 4000 --publish-count 10 --subscribe-after 3000`
  * After a few seconds, you should see all of the messages (0-9) printed from the subscriber.
  * All messages, with their 4 second lifespan, survived until the subscriber joined.

## Liveliness

This demo shows how liveliness can be used by publishers to notify subscriptions that they are "alive".

The demo creates a talker and listener with the provided liveliness lease duration (in milliseconds).
The talker will assert its liveliness based on passed in options, and be stopped after some amount of time.
If talker does not notify the listener that it is alive within the lease duration, then you should see liveliness change events on the screen.

With the default `AUTOMATIC` liveliness policy, the publisher is stopped by deleting the talker in order to stop all automatic liveliness assertions in the rmw implementation.
Therefore, only the listener will receive a liveliness change event for `AUTOMATIC`.
With a liveliness policy of `MANUAL_BY_TOPIC`, the publisher is stopped by no longer publishing messages.
Therefore, both the talker and listener will receive liveliness change events for `MANUAL_BY_TOPIC` (if the rmw implementation supports this feature).

For usage, run `ros2 run quality_of_service_demo_py liveliness -h`

Examples:
* `ros2 run quality_of_service_demo_cpp liveliness 1000 --kill-publisher-after 2000` _or_
* `ros2 run quality_of_service_demo_py liveliness 1000 --kill-publisher-after 2000`
  * After 2 seconds, the publisher will be killed, and the subscriber will receive a callback 1 second after that notifying it that the liveliness has changed.

## Incompatible QoS Offered/Requested

This demo shows how your program can be notified when an publisher and a subscription detect that they have incompatible QoS settings.
If a publisher and subscription have incompatible QoS settings, then they will not be able to communicate.

The demo creates a talker and listener such that they have incompatible QoS settings.
Both the talker and listener should output a notification regarding the incompatible QoS settings.

For usage, run `ros2 run quality_of_service_demo_py incompatible_qos -h`

Examples:
* `ros2 run quality_of_service_demo_cpp incompatible_qos durability` _or_
* `ros2 run quality_of_service_demo_py incompatible_qos durability`

## Interactive Quality of Service Demos

These demos allow the user to interactively generate events and see the publisher's and subscriber's responses to these user events.

### Interactive Demo Startup

Start the publisher and subscriber in separate terminals:
```
ros2 run quality_of_service_demo_cpp interactive_publisher
ros2 run quality_of_service_demo_cpp interactive_subscriber
```

The following options are available when starting the demos:
* `--help`:       Prints a help message regarding program usage.
* `--delay`:      The delay in seconds between publishing messages (only applies to the publisher).
* `--deadline`:   The period in seconds of the Deadline QoS Policy
                  (offered period for the publisher, requested period for the subscriber).
* `--liveliness`: The kind of Liveliness QoS Policy, which must be one of `AUTOMATIC` or `MANUAL_BY_TOPIC`
                  (offered kind for the publisher, requested kind for the subscriber).
* `--lease`:      The duration in seconds of the lease for Liveliness QoS Policy
                  (offered duration for the publisher, requested duration for the subscriber).

### Demo Control

While the demo is running, the following commands may be issued in the publisher's or subscriber's terminal:
* Press `p`: Manually assert the liveliness of the publisher, for when Liveliness kind is `MANUAL_BY_TOPIC`
             (only applies to the publisher).
* Press `s`: Toggle enabling/disabling of publishing messages (only applies to the publisher).
* Press `q`: Print the current QoS Policy settings of the publisher or subscriber.
* Press `x`: Stop and exit the demo.

### Example Demo Run

Let's start the publisher sending a message every half second, and the publisher and subscriber with both offered and requested Deadline periods and Liveliness lease durations at 1 second:
```
ros2 run quality_of_service_demo_cpp interactive_publisher --delay 0.5 --deadline 1 --liveliness MANUAL_BY_TOPIC --lease 1
ros2 run quality_of_service_demo_cpp interactive_subscriber --deadline 1 --liveliness MANUAL_BY_TOPIC --lease 1
```

You will see the publisher publishing messages and the subscriber receiving them.

Now **press** `s` in the publisher's terminal to stop publishing messages. After about 1 second,
the subscriber will print "Liveliness changed" to the terminal in response to the publisher losing implied liveliness.
The subscriber will also print "Deadline missed" to the terminal at the regular 1 second Deadline duration,
because by not receiving any messages from the publisher, the deadline is being missed on a regular basis.

Now **press** `p` once in the publisher's terminal to assert that the publisher is alive even though it is not sending any messages.
The subscriber will almost immediately print "Liveliness changed" to the terminal, because the publisher has come alive.
However, about 1 second later, the subscriber will print "Liveliness changed" to the terminal again,
because the publisher has not asserted its liveliness again within 1 second, the lease duration.

If you constantly **press** `p` more frequently than 1 second, the subscriber will print "Liveliness changed" to the terminal once,
because the publisher has come alive without losing liveliness, until you **stop** frequently pressing `p`.

Notice that "Deadline missed" is constantly being printed even though the publisher is alive,
because Deadline and Liveliness QoS Policies are unrelated.
**Press** `s` in the publisher's terminal again to start publishing messages again,
and the "Deadline missed" messages will no longer be printed.

## Message lost status event demo

This demo shows how to get a notification when a subscription loses a message.

This feature is not available in all RMW implementations.
`rmw_cyclonedds_cpp` and  `rmw_connextdds` do support this feature.
CycloneDDS partially implements the feature and it only triggers the event under some limited circumstances, thus it's recommended to try the demo with Connext.

In one terminal, run a listener
```
export RMW_IMPLEMENTATION=rmw_connextdds
ros2 run quality_of_service_demo_cpp message_lost_listener
```

in another terminal, run a talker (use the `-s` option to provide the message size in KiB):
```
export RMW_IMPLEMENTATION=rmw_connextdds
ros2 run quality_of_service_demo_cpp message_lost_talker -s 8192
```

You should see the talker output to the terminal for each message it publishes.
The listener should report each message that it receives as well as any lost message events it detects, for example:
```
[INFO] [1593021635.971678672] [MessageLostListener]: Some messages were lost:
>       Number of new lost messages: 1
>       Total number of messages lost: 1
[INFO] [1593021636.601466840] [MessageLostListener]: I heard an image. Message single trip latency: [3.442907]
```

For information about how to tune QoS settings for large messages see [DDS tuning](https://docs.ros.org/en/rolling/Guides/DDS-tuning.html).

## QoS overrides

You can use QoS overrides parameters for making QoS profiles configurable when starting a node.
Create a parameters yaml file, similar to the examples in the `params_file` folder, and run:

```
ros2 run quality_of_service_demo_py qos_overrides_talker --ros-args --params-file /path/to/yaml/file
```

in another terminal:

```
ros2 run quality_of_service_demo_py qos_overrides_listener --ros-args --params-file /path/to/yaml/file
```

Alternatively, you can run the C++ version of the demo:

```
ros2 run quality_of_service_demo_cpp qos_overrides_talker --ros-args --params-file /path/to/yaml/file
```

in another terminal:

```
ros2 run quality_of_service_demo_cpp qos_overrides_listener --ros-args --params-file /path/to/yaml/file
```

If you don't want to create your own yaml file, you can use `$(ros2 pkg prefix quality_of_service_demo_cpp)/share/quality_of_service_demo_cpp/params_file/example_qos_overrides.yaml` instead of `/path/to/yaml/file` to use the installed example yaml file.
