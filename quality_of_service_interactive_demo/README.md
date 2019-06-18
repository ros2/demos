# Interactive Quality of Service Demos

The demo applications in this package allow the user to interactively generate events
and see the ROS publisher's and subscriber's response to these user events.


## Demo Startup

Start the publisher and subscriber in separate terminals
(it is recommended to run these demos using the RTI Connext RMW implementation):
```
RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run publisher_with_qos_demo publisher_with_qos
RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run subscriber_with_qos_demo subscriber_with_qos
```

The following options are available when starting the demos:
* `--help`:       Prints a help message regarding program usage.
* `--delay`:      The delay in seconds between publishing messages (only applies to the publisher).
* `--deadline`:   The period in seconds of the Deadline QoS Policy
                  (offered period for the publisher, requested period for the subscriber).
* `--liveliness`: The kind of Liveliness QoS Policy, which must be one of `AUTOMATIC`, `MANUAL_BY_NODE`, or `MANUAL_BY_TOPIC`
                  (offered kind for the publisher, requested kind for the subscriber).
* `--lease`:      The duration in seconds of the lease for Liveliness QoS Policy
                  (offered duration for the publisher, requested duration for the subscriber).


## Demo Control

While the demo is running, the following commands may be issued in the publisher's or subscriber's terminal:
* Press `n`: Manually assert the liveliness of the node, for when Liveliness kind is `MANUAL_BY_NODE`
             (only applies to the publisher).
* Press `p`: Manually assert the liveliness of the publisher, for when Liveliness kind is `MANUAL_BY_TOPIC`
             (only applies to the publisher).
* Press `s`: Toggle enabling/disabling of publishing messages (only applies to the publisher).
* Press `q`: Print the current QoS Policy settings of the publisher or subscriber.
* Press `x`: Stop and exit the demo.


## Example Demo Run

Let's start the publisher sending a message every half second, and the publisher and subscriber with both offered and requested
Deadline periods and Liveliness lease durations at 1 second:
```
RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run publisher_with_qos_demo publisher_with_qos --delay 0.5 --deadline 1 --liveliness MANUAL_BY_TOPIC --lease 1
RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run subscriber_with_qos_demo subscriber_with_qos --deadline 1 --liveliness MANUAL_BY_TOPIC --lease 1
```

You will see the publisher publishing "Hello, world!" messages and the subscriber receiving them.

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
