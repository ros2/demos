# Quality of Service Demos

The demo applications in this package show the various Quality of Service settings available in ROS2.
The intent is to provide a dedicated application for each QoS type, in order to highlight that single feature in isolation.

## History

No History QoS demo app exists yet in this package.

## Reliability

No Reliability QoS demo app exists yet in this package.

## Durability

No Durability QoS demo app exists yet in this package.

## Deadline

`quality_of_service_demo/deadline` demonstrates messages not meeting their required deadline.

The application requires an argument a `deadline_duration` - this is the maximum acceptable time (in positive integer milliseconds) between messages published on the topic.
A `deadline_duration` of `0` means that there is no deadline, so the application will act as a standard Talker/Listener pair.

The publisher in this demo will pause publishing periodically, which will print deadline expirations from the subscriber.

Run `quality_of_service_demo/deadline -h` for more usage information.

Examples:
* `ros2 run quality_of_service_demo_cpp deadline 600 --publish-for 5000 --pause-for 2000` _or_
* `ros2 run quality_of_service_demo_py  deadline 600 --publish-for 5000 --pause-for 2000`
  * The publisher will publish for 5 seconds, then pause for 2 - deadline events will start firing on both participants, until the publisher comes back.
* `ros2 run quality_of_service_demo_cpp deadline 600 --publish-for 5000 --pause-for 0` _or_
* `ros2 run quality_of_service_demo_py  deadline 600 --publish-for 5000 --pause-for 0`
  * The publisher doesn't actually pause, no deadline misses should occur.

## Lifespan

`quality_of_service_demo/lifespan` demonstrates messages being deleted from a publisher's outgoing queue once their configured lifespan expires.

The application requires an argument `lifespan_duration` - this is the maximum time (in positive integer milliseconds) that a message will sit in the publisher's outgoing queue.

The publisher in this demo will publish a preset number of messages, then stop publishing.
A subscriber will start subscribing after a preset amount of time and may receive _fewer_ backfilled messages than were originally published, because some message can outlast their lifespan and be removed.

Run `lifespan -h` for more usage information.

Examples:
* `ros2 run quality_of_service_demo_cpp lifespan 1000 --publish-count 10 --subscribe-after 3000` _or_
* `ros2 run quality_of_service_demo_py  lifespan 1000 --publish-count 10 --subscribe-after 3000`
  * After a few seconds, you should see (approximately) messages 4-9 printed from the subscriber.
  * The first few messages, with 1 second lifespan, were gone by the time the subscriber joined after 3 seconds.
* `ros2 run quality_of_service_demo_cpp lifespan 4000 --publish-count 10 --subscribe-after 3000` _or_
* `ros2 run quality_of_service_demo_py  lifespan 4000 --publish-count 10 --subscribe-after 3000`
  * After a few seconds, you should see all of the messages (0-9) printed from the subscriber.
  * All messages, with their 4 second lifespan, survived until the subscriber joined.

## Liveliness

`quality_of_service_demo/liveliness` demonstrates notification of liveliness changes, based on different Liveliness configurations.

The application requires an argument `lease_duration` that specifies how often (in milliseconds) an entity must "check in" to let the system know it's alive.

The publisher in this demo will assert its liveliness based on passed in options, and be stopped after some amount of time.
When using `AUTOMATIC` liveliness policy, the publisher is deleted at this time, in order to stop all automatic liveliness assertions in the rmw implementation - therefore only the Subscription will receive a Liveliness event for `AUTOMATIC`.
For `MANUAL` liveliness policies, the publisher's assertions are stopped, as well as its message publishing.
Publishing a message implicitly counts as asserting liveliness, so publishing is stopped in order to allow the Liveliness lease to lapse.
Therefore in the `MANUAL` policy types, you will see Liveliness events from both publisher and Subscription (in rmw implementations that implement this feature).

Run `quality_of_service_demo/liveliness -h` for more usage information.

Examples:
* `ros2 run quality_of_service_demo_cpp liveliness 1000 --kill-publisher-after 2000` _or_
* `ros2 run quality_of_service_demo_py  liveliness 1000 --kill-publisher-after 2000`
  * After 2 seconds, the publisher will be killed, and the subscriber will receive a callback 1 second after that notifying it that the liveliness has changed.

## Incompatible QoS Offered/Requested

`quality_of_service_demo/incompatible_qos` demonstrates notification from a publisher and a subscriber upon discovering each other that they are not compatible.

The application requires an argument `incompatible_qos_policy_name` that specifies which QoS policy should be made incompatible between the publisher and subscriber as an example.

The publisher in this demo will publish 5 messages, but also output a notification stating that its offered QoS is incompatible with the subscriber that is also created by this demo.
The subscriber in this demo will similarly output a notification stating that its requested QoS is incompatible with the QoS offered by the publisher.

Run `quality_of_service_demo/incompatible_qos -h` for more usage information.

Examples:
* `ros2 run quality_of_service_demo_cpp incompatible_qos durability` _or_
* `ros2 run quality_of_service_demo_py  incompatible_qos durability`

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

The feature is available in some rmw implementations: rmw_cyclonedds, rmw_connextdds.
CycloneDDS partially implements the feature and it only triggers the event under some limited circumstances, thus it's recommended to try the demo with Connext.

To run the demo:
```
export RMW_IMPLEMENTATION = rmw_connextdds
ros2 run quality_of_service_demo_cpp message_lost_listener &
# -s allows specifying the message size in KiB. This argument is optional, defaults to 8192KiB.
ros2 run quality_of_service_demo_cpp message_lost_talker -s 8192
# ros2 run quality_of_service_demo_py message_lost_talker -s 8192 to run the python listener.
```

Example output:
```
[INFO] [1593021627.113051194] [message_lost_talker]: Publishing an image, sent at [1593021627.113023]
[INFO] [1593021630.114633564] [message_lost_talker]: Publishing an image, sent at [1593021630.114625]
[INFO] [1593021633.158548336] [message_lost_talker]: Publishing an image, sent at [1593021633.158538]
[INFO] [1593021635.971678672] [MessageLostListener]: Some messages were lost:
>       Number of new lost messages: 1
>       Total number of messages lost: 1
[INFO] [1593021636.130686719] [message_lost_talker]: Publishing an image, sent at [1593021636.130677]
[INFO] [1593021636.601466840] [MessageLostListener]: I heard an image. Message single trip latency: [3.442907]
[INFO] [1593021639.129067211] [message_lost_talker]: Publishing an image, sent at [1593021639.129058]
```

For information about how to tune QoS settings for large messages see [DDS tuning](https://docs.ros.org/en/rolling/Guides/DDS-tuning.html).

## Qos overrides

You can use QoS overrides parameters for making QoS profiles configurable when starting a node.
Create a parameters yaml file, similar to the examples in the `params_file` folder, and run:

```
# you can use `$(ros2 pkg prefix quality_of_service_demo_cpp)/share/quality_of_service_demo_cpp/params_file/example_qos_overrides.yaml` instead of `/path/to/yaml/file` to use the example installed yaml file.
ros2 run quality_of_service_demo_cpp qos_overrides_talker --ros-args --params-file /path/to/yaml/file
```

in another terminal:

```
ros2 run quality_of_service_demo_cpp qos_overrides_listener --ros-args --params-file /path/to/yaml/file
```

There are similar qos_overrides_talker/qos_overrides_listener executables in `quality_of_service_demo_py`.
